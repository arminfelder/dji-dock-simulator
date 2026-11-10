#include "cloud/redact.hpp"
#include "docksim/app.hpp"
#include "log/log.hpp"
#include "media/photo.hpp"
#include "media/s3.hpp"
#include "net/http.hpp"
#include "sim/clock.hpp"
#include "sim/dock_sim.hpp"
#include "transport/mqtt.hpp"
#include "wayline/wpml.hpp"

#include "control/sim_control.hpp"
#include "control/wire_log.hpp"
#include "transport/tap.hpp"

#ifdef DOCKSIM_WITH_LIVE
#include "live/push.hpp"
#endif
#ifdef DOCKSIM_WITH_API
#include "api/server.hpp"
#endif

#include <asio/co_spawn.hpp>
#include <asio/experimental/awaitable_operators.hpp>
#include <asio/io_context.hpp>
#include <asio/signal_set.hpp>
#include <asio/thread_pool.hpp>

#include <csignal>
#include <format>

namespace docksim {

namespace {

using namespace asio::experimental::awaitable_operators;

constexpr auto connect_timeout = std::chrono::seconds{20};
constexpr auto shutdown_timeout = std::chrono::seconds{10};
constexpr auto kmz_timeout = std::chrono::seconds{60};

const auto& logger() {
    static const auto log = log::get("dock_sim");
    return log;
}

// The streamer exists before the dock that owns the cameras; its feeds find
// the cameras through this slot, which serve() fills once the dock is built.
using CameraSlot = std::shared_ptr<sim::Cameras*>;

std::unique_ptr<live::Streamer> make_streamer([[maybe_unused]] const Config& cfg,
                                              [[maybe_unused]] const CameraSlot& cameras) {
#ifdef DOCKSIM_WITH_LIVE
    return std::make_unique<live::LiveStreamer>(
        cfg.live, [&cfg, cameras](const std::string& video_id) {
            return *cameras != nullptr ? live::camera_feeds(cfg, **cameras)(video_id) : nullptr;
        });
#else
    return nullptr;
#endif
}

sim::Dependencies real_dependencies(const Config& cfg, asio::thread_pool& workers,
                                    const sim::Executor& ex, const CameraSlot& cameras) {
    return {
        .sessions =
            [ex](const BrokerConfig& broker, const std::string& client_id) {
                return std::shared_ptr<transport::Session>(
                    std::make_shared<transport::MqttTransport>(ex, broker, client_id));
            },
        .storage =
            [&cfg](const Json& output) {
                return std::make_unique<s3::S3Storage>(output, cfg.media);
            },
        .photos = {},
        .download = [&cfg, &workers](std::string url) -> asio::awaitable<wayline::Wayline> {
            co_return co_await sim::to_thread(workers, [&cfg, url] {
                logger()->info("download wayline {}", redact::safe_url(url));
                const auto response = http::perform({
                    .url = url,
                    .verify_tls = cfg.http.verify_tls || cfg.http.ca_file.has_value(),
                    .ca_file = cfg.http.ca_file,
                    .timeout = kmz_timeout,
                });
                if (response.status >= 400)
                    throw std::runtime_error(std::format("HTTP {}", response.status));
                return wayline::parse_kmz(http::as_bytes(response.body));
            });
        },
        .streamer = make_streamer(cfg, cameras),
        .verify_license = {},
        .weather_seed = std::nullopt,
        .fetch = [&cfg, &workers](std::string url) -> asio::awaitable<std::vector<std::byte>> {
            co_return co_await sim::to_thread(workers, [&cfg, url] {
                logger()->info("download {}", redact::safe_url(url));
                const auto response = http::perform({
                    .url = url,
                    .verify_tls = cfg.http.verify_tls || cfg.http.ca_file.has_value(),
                    .ca_file = cfg.http.ca_file,
                    .timeout = kmz_timeout,
                });
                if (response.status >= 400)
                    throw std::runtime_error(std::format("HTTP {}", response.status));
                return http::as_bytes(response.body);
            });
        },
    };
}

asio::awaitable<int> serve(const Config& cfg, asio::thread_pool& workers) {
    const auto ex = co_await asio::this_coro::executor;
    transport::MqttTransport transport(ex, cfg.broker, cfg.device.dock_sn);
    control::WireLog wire;
    transport::TappedTransport tapped(transport, wire.sink());
    sim::RealClock clock;
    const auto cameras = std::make_shared<sim::Cameras*>(nullptr);
    sim::DockSim dock(ex, cfg, tapped, clock, workers,
                      real_dependencies(cfg, workers, ex, cameras));
    *cameras = &dock.cameras();
    control::SimControl control(dock, &clock, wire);
#ifdef DOCKSIM_WITH_API
    std::optional<api::ApiServer> api;
    if (cfg.api.enabled) {
        api.emplace(ex, control, workers, cfg.api);
        if (!api->start()) api.reset();
    }
    const auto stop_api = [&api] {
        if (api) api->stop();
    };
#else
    const auto stop_api = [] {};
#endif

    std::exception_ptr session_error;
    sim::Event session_done(ex);
    // Named, so that the closure outlives the coroutine it starts.
    const auto hold_session = [&]() -> asio::awaitable<void> {
        try {
            co_await transport.run(true);
        } catch (const std::system_error& e) {
            if (e.code() != std::errc::operation_canceled) session_error = std::current_exception();
        } catch (...) {
            session_error = std::current_exception();
        }
        session_done.set();
    };
    // Every exit below goes through cancel_all(), so no task outlives this frame.
    sim::TaskGroup session_group(ex, [](const std::string&, const std::exception_ptr&) {});
    session_group.spawn(hold_session(), "mqtt");

    int code = 0;
    asio::steady_timer timer(ex, connect_timeout);
    co_await (transport.wait_connected() || session_done.wait() ||
              timer.async_wait(asio::use_awaitable));
    if (session_error) {
        try {
            std::rethrow_exception(session_error);
        } catch (const std::exception& e) {
            logger()->error("{}", e.what());
        }
        code = 1;
    } else if (!transport.connected()) {
        logger()->error("MQTT connection timed out after {} s", connect_timeout.count());
        code = 1;
    }

    bool started = false;
    if (code == 0) {
        try {
            co_await dock.start(true);
            started = true;
        } catch (const sim::LicenseError& e) {
            logger()->error("{}", e.what());
            code = 1;
        }
    }

    if (started) {
        asio::signal_set signals(ex, SIGINT, SIGTERM);
        co_await (signals.async_wait(asio::use_awaitable) || session_done.wait());
        logger()->info("shutdown");
        stop_api();
        asio::steady_timer limit(ex, shutdown_timeout);
        const auto finished = co_await (dock.stop() || limit.async_wait(asio::use_awaitable));
        if (finished.index() == 1)
            logger()->warn("shutdown did not finish in {} s", shutdown_timeout.count());
    }
    stop_api();
    co_await transport.cancel_handlers();
    co_await dock.ctx().cancel_all();
    co_await dock.mission().shutdown();
    co_await session_group.cancel_all();
    co_return code;
}

}  // namespace

void set_log_level(LogLevel level, bool log_wire) {
    log::set_level(level);
    log::get("dock_sim.wire")->set_level(log_wire ? spdlog::level::debug : spdlog::level::warn);
}

int run_dock(const Config& cfg, const RunOptions& options) {
    set_log_level(cfg.log_level, options.log_wire);
    asio::io_context io;
    asio::thread_pool workers(4);
    int code = 1;
    asio::co_spawn(io, serve(cfg, workers), [&](const std::exception_ptr& e, int rc) {
        if (e) {
            try {
                std::rethrow_exception(e);
            } catch (const std::exception& error) {
                logger()->critical("{}", error.what());
            }
            return;
        }
        code = rc;
    });
    io.run();
    workers.join();
    return code;
}

}  // namespace docksim
