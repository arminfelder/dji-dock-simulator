#include "services/services.hpp"

#include "device/catalog.hpp"
#include "device/gimbal.hpp"
#include "flight/manual.hpp"
#include "log/log.hpp"
#include "protocol/envelope.hpp"
#include "protocol/payloads.hpp"
#include "protocol/routing.hpp"
#include "protocol/topics.hpp"
#include "services/camera.hpp"
#include "sim/context.hpp"
#include "wayline/engine.hpp"

#include <asio/experimental/awaitable_operators.hpp>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <format>
#include <tuple>

namespace docksim::services {

namespace {

using namespace asio::experimental::awaitable_operators;

// camera_screen_drag repeats about every 0.5 s. A longer gap means a new drag.
constexpr double drag_interval_s = 0.5;
constexpr double drag_interval_max_s = 0.6;
constexpr auto drc_connect_timeout = std::chrono::seconds{15};

const auto& logger(std::string_view name) {
    static std::map<std::string, std::shared_ptr<spdlog::logger>, std::less<>> loggers;
    auto it = loggers.find(name);
    if (it == loggers.end())
        it = loggers.emplace(std::string(name), log::get(std::string(name))).first;
    return it->second;
}

Json result_only(code::Result code) {
    return {{"result", code}};
}

std::optional<std::string> optional_text(const Json& data, std::string_view key) {
    const auto it = data.find(key);
    if (it == data.end() || it->is_null()) return std::nullopt;
    return it->is_string() ? it->get<std::string>() : it->dump();
}

}  // namespace

void register_flighttask(Registry& registry, wayline::MissionEngine& mission,
                         const std::string& dock_model) {
    const auto prepare = [&mission](const Json& data) -> asio::awaitable<Reply> {
        co_return result_only(co_await mission.prepare(data));
    };
    registry.add("flighttask_prepare", prepare);
    // The cloud sends flighttask_create in place of prepare for some task types. The data is the
    // same. Dock 2 firmware does not route it. Dock 3 firmware is not known.
    if (dock_model == "dock1") registry.add("flighttask_create", prepare);
    registry.add("flighttask_execute", [&mission](const Json& data) -> asio::awaitable<Reply> {
        co_return result_only(mission.execute(text(data, "flight_id", "")));
    });
    registry.add("flighttask_pause", [&mission](const Json&) -> asio::awaitable<Reply> {
        co_return result_only(mission.pause());
    });
    registry.add("flighttask_recovery", [&mission](const Json&) -> asio::awaitable<Reply> {
        co_return result_only(mission.recovery());
    });
    registry.add("flighttask_stop", [&mission](const Json&) -> asio::awaitable<Reply> {
        co_return result_only(mission.cancel());
    });
    registry.add("flighttask_undo", [&mission](const Json& data) -> asio::awaitable<Reply> {
        std::vector<std::string> ids;
        if (data.contains("flight_ids")) {
            for (const auto& id : data.at("flight_ids"))
                ids.push_back(id.is_string() ? id.get<std::string>() : id.dump());
        }
        co_return result_only(mission.undo(ids));
    });
}

void PayloadControl::register_services(Registry& registry) {
    auto& drone = ctx_.drone;
    const auto lens_dfov = [&drone](const std::string& lens) {
        return gimbal::lens_dfov(drone, gimbal::lens_from_name(lens), catalog::wide_dfov);
    };
    registry.add("camera_screen_drag", [this, &drone](const Json& data) -> asio::awaitable<Reply> {
        const double now = ctx_.clock().monotonic();
        const double last = std::exchange(last_drag_, now);
        const double seconds =
            last != 0.0 ? std::min(drag_interval_max_s, now - last) : drag_interval_s;
        gimbal::drag(drone, number(data, "pitch_speed", 0.0), number(data, "yaw_speed", 0.0),
                     flag(data, "locked", false), seconds);
        co_return ok();
    });
    registry.add(
        "camera_aim", [this, &drone, lens_dfov](const Json& data) -> asio::awaitable<Reply> {
            gimbal::aim(drone, number(data, "x", 0.5), number(data, "y", 0.5),
                        flag(data, "locked", false), lens_dfov(text(data, "camera_type", "wide")),
                        ctx_.cfg().live.frame());
            co_return ok();
        });
    registry.add("camera_look_at", [this, &drone](const Json& data) -> asio::awaitable<Reply> {
        gimbal::look_at(drone, drone.home.altitude, number(data, "latitude"),
                        number(data, "longitude"), number(data, "height", 0.0),
                        flag(data, "locked", false));
        co_return ok();
    });
    registry.add("gimbal_reset", [&drone](const Json& data) -> asio::awaitable<Reply> {
        gimbal::reset(drone, static_cast<int>(integer(data, "reset_mode", 0)));
        co_return ok();
    });
    registry.add("camera_focal_length_set", [&drone](const Json& data) -> asio::awaitable<Reply> {
        gimbal::zoom(drone, number(data, "zoom_factor", 2.0),
                     gimbal::lens_from_name(text(data, "camera_type", "")));
        co_return ok();
    });
}

void LiveService::register_services(Registry& registry) {
    registry.add("live_start_push", [this](const Json& data) -> asio::awaitable<Reply> {
        const auto default_id =
            std::format("{}/{}/normal-0", ctx_.drone_sn(), ctx_.drone.payload_index);
        const auto video_id = text(data, "video_id", default_id);
        const auto url = text(data, "url", "");
        const auto type = static_cast<code::UrlType>(
            integer(data, "url_type", std::to_underlying(code::UrlType::rtmp)));
        const auto code = co_await sim::to_thread(
            ctx_.workers(), [&] { return streamer_->start(video_id, url, type); });
        // The output must be a JSON string. The cloud reads it as
        // ServicesReplyData<String>; an object makes the cloud throw.
        co_return Json{{"result", code}, {"output", url.empty() ? video_id : url}};
    });
    registry.add("live_stop_push", [this](const Json& data) -> asio::awaitable<Reply> {
        const auto video_id = text(data, "video_id", "");
        const auto code =
            co_await sim::to_thread(ctx_.workers(), [&] { return streamer_->stop(video_id); });
        co_return Json{{"result", code}, {"output", Json::object()}};
    });
    registry.add("live_lens_change", [this](const Json& data) -> asio::awaitable<Reply> {
        streamer_->set_lens(text(data, "video_id", ""), text(data, "video_type", "wide"));
        co_return ok();
    });
    // The streamers have no quality levels and one camera per video_id.
    registry.stub({"live_set_quality", "live_camera_change"});
}

asio::awaitable<void> LiveService::close() {
    co_await sim::to_thread(ctx_.workers(), [this] { streamer_->stop_all(); });
}

BrokerAddress split_broker(std::string_view address, bool enable_tls) {
    struct Scheme {
        std::string_view name;
        bool tls;
        BrokerTransport transport;
    };
    static constexpr std::array schemes{
        Scheme{.name = "ssl", .tls = true, .transport = BrokerTransport::tcp},
        Scheme{.name = "mqtts", .tls = true, .transport = BrokerTransport::tcp},
        Scheme{.name = "tcp", .tls = false, .transport = BrokerTransport::tcp},
        Scheme{.name = "mqtt", .tls = false, .transport = BrokerTransport::tcp},
        Scheme{.name = "wss", .tls = true, .transport = BrokerTransport::websockets},
        Scheme{.name = "ws", .tls = false, .transport = BrokerTransport::websockets},
    };
    BrokerAddress out{.tls = enable_tls};
    std::string_view raw = address;
    while (!raw.empty() && std::isspace(static_cast<unsigned char>(raw.front())) != 0)
        raw.remove_prefix(1);
    while (!raw.empty() && std::isspace(static_cast<unsigned char>(raw.back())) != 0)
        raw.remove_suffix(1);

    if (const auto sep = raw.find("://"); sep != std::string_view::npos) {
        std::string scheme(raw.substr(0, sep));
        std::ranges::transform(scheme, scheme.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (const auto* s = std::ranges::find(schemes, scheme, &Scheme::name); s != schemes.end()) {
            out.tls = s->tls;
            out.transport = s->transport;
            raw = raw.substr(sep + 3);
        }
    }
    const auto slash = raw.find('/');
    const auto hostport = raw.substr(0, slash);
    if (slash != std::string_view::npos && slash + 1 < raw.size())
        out.path = std::format("/{}", raw.substr(slash + 1));
    const auto colon = hostport.find(':');
    out.host = std::string(hostport.substr(0, colon));
    const int default_port =
        out.transport == BrokerTransport::websockets ? 443 : (out.tls ? 8883 : 1883);
    if (out.host.empty()) return out;
    out.port = default_port;
    if (colon != std::string_view::npos) {
        const auto text = hostport.substr(colon + 1);
        int port = 0;
        const auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), port);
        if (ec == std::errc{} && ptr == text.data() + text.size()) out.port = port;
    }
    return out;
}

Json redact_passwords(const Json& value) {
    if (!value.is_object()) return value;
    Json out = Json::object();
    for (const auto& [key, item] : value.items())
        out[key] = key == "password" ? Json("***") : redact_passwords(item);
    return out;
}

void DrcControl::register_services(Registry& registry) {
    registry.add("drc_mode_enter", [this](const Json& data) -> asio::awaitable<Reply> {
        co_return co_await enter(data);
    });
    registry.add("drc_mode_exit",
                 [this](const Json&) -> asio::awaitable<Reply> { co_return co_await exit(); });
}

void DrcControl::set_frequencies(const Json& data) {
    osd_hz_ = static_cast<int>(std::clamp<std::int64_t>(
        integer(data, "osd_frequency", drc_default_osd_hz), 1, drc_max_osd_hz));
    hsi_hz_ = static_cast<int>(
        std::clamp<std::int64_t>(integer(data, "hsi_frequency", drc_default_hsi_hz), 1, osd_hz_));
}

asio::awaitable<Reply> DrcControl::enter(const Json& data) {
    const auto& log = logger("dock_sim.drc");
    const Json& broker = data.contains("mqtt_broker") && data["mqtt_broker"].is_object()
                             ? data["mqtt_broker"]
                             : data;
    auto address = optional_text(broker, "address").value_or("");
    if (address.empty()) address = optional_text(broker, "broker").value_or("");
    const auto where = split_broker(address, flag(broker, "enable_tls", false));
    const auto& root = ctx_.cfg().broker;
    Link link{
        .broker =
            {
                .host = where.host,
                .port = where.port,
                .tls = where.tls,
                .ca_file = root.ca_file,
                .insecure = root.insecure,
                .transport = where.transport,
                .ws_path = where.path,
                .username = optional_text(broker, "username"),
                .password = optional_text(broker, "password"),
            },
        .client_id = optional_text(broker, "client_id").value_or(""),
    };
    if (link.client_id.empty()) link.client_id = std::format("{}-drc", ctx_.dock_sn());
    // The cloud sends drc_mode_enter again for each web UI tab, about once a second,
    // and each one carries a new JWT as password (seen on a test deployment). An enter to
    // the session that is up keeps it and changes only the push rates. A new password
    // alone must not reconnect: the disconnect notify makes the cloud send enter again.
    auto same_target = link_;
    if (same_target) same_target->broker.password = link.broker.password;
    if (session_ && session_->connected() && same_target == link) {
        link_ = link;
        set_frequencies(data);
        log->debug("drc_mode_enter: session is up already");
        co_return result(code::Result::ok);
    }
    log->info("drc_mode_enter: {}", redact_passwords(data).dump());
    const auto attempt = ++attempt_;
    co_await close();
    if (where.host.empty()) {
        log->error("drc_mode_enter carried no usable broker: {}", address);
        co_return result(code::Result::drc_bad_broker);
    }
    set_frequencies(data);

    auto session = sessions_(link.broker, link.client_id);
    session->on("/drc/down", [this](std::string, Json message) -> asio::awaitable<void> {
        co_await on_down(std::move(message));
    });
    // The DRC JWT grants sub on this exact topic. A wildcard is denied, and
    // EMQX then drops the session.
    session->subscribe(topic::drc_down(ctx_.dock_sn()));
    auto runner = ctx_.spawn(run(session), "drc-session");

    asio::steady_timer timer(ctx_.executor(), drc_connect_timeout);
    co_await (session->wait_connected() || runner.wait() || timer.async_wait(asio::use_awaitable));
    if (attempt != attempt_) {
        runner.cancel();
        log->warn("drc session to {}:{} dropped: a later enter or exit replaced it", where.host,
                  where.port);
        co_return result(code::Result::drc_broker_refused);
    }
    if (!session->connected()) {
        runner.cancel();
        log->error("drc broker {}:{} did not accept the session", where.host, where.port);
        co_return result(code::Result::drc_broker_refused);
    }
    session_ = session;
    runner_ = runner;
    link_ = std::move(link);
    last_heartbeat_ = ctx_.clock().monotonic();
    ctx_.dock.drc_state = code::DrcState::connected;
    co_await notify(code::DrcState::connected, code::Result::ok);
    pusher_ = ctx_.spawn(pushes(session), "drc-pushes");
    log->info("drc session up: {}:{} tls={} websockets={}", where.host, where.port, where.tls,
              where.transport == BrokerTransport::websockets);
    co_return result(code::Result::ok);
}

asio::awaitable<Reply> DrcControl::exit() {
    ++attempt_;
    co_await close();
    co_return result(code::Result::ok);
}

asio::awaitable<void> DrcControl::run(std::shared_ptr<transport::Session> session) {
    std::string error;
    try {
        co_await session->run(false);
    } catch (const transport::ConnectionLost& e) {
        error = e.what();
    } catch (const transport::ConnectionRefused& e) {
        error = e.what();
    }
    if (!error.empty() && session_ == session) {
        logger("dock_sim.drc")->warn("drc session lost: {}", error);
        session_.reset();
        runner_ = {};
        link_.reset();
        std::exchange(pusher_, {}).cancel();
        ctx_.dock.drc_state = code::DrcState::disconnected;
        // The real dock sends this result when it loses the DRC connection.
        co_await notify(code::DrcState::disconnected, code::Result::drc_connection_lost);
    }
}

asio::awaitable<void> DrcControl::close() {
    auto session = std::exchange(session_, nullptr);
    auto runner = std::exchange(runner_, {});
    auto pusher = std::exchange(pusher_, {});
    link_.reset();
    if (!session) co_return;
    pusher.cancel();
    runner.cancel();
    ctx_.dock.drc_state = code::DrcState::disconnected;
    logger("dock_sim.drc")->info("drc session closed");
    co_await notify(code::DrcState::disconnected, code::Result::ok);
}

asio::awaitable<void> DrcControl::idle_exit() {
    logger("dock_sim.drc")
        ->warn("no drc heart_beat for {} s: leaving drc mode", drc_heartbeat_timeout_s);
    ++attempt_;
    co_await close();
}

asio::awaitable<void> DrcControl::notify(code::DrcState state, code::Result result) {
    // The real dock puts result and drc_state flat in data.
    co_await ctx_.emit_event("drc_status_notify", {{"result", result}, {"drc_state", state}});
}

asio::awaitable<void> DrcControl::push(std::shared_ptr<transport::Session> session,
                                       std::string method, Json data) {
    co_await session->publish(topic::drc_up(ctx_.dock_sn()),
                              {{"method", std::move(method)},
                               {"data", std::move(data)},
                               {"seq", ++up_seq_},
                               {"timestamp", envelope::now_ms()}},
                              1);
}

asio::awaitable<void> DrcControl::push_states(std::shared_ptr<transport::Session> session) {
    const auto& drone = ctx_.drone;
    co_await push(session, "hsi_info_push", payloads::drc_hsi_info(drone));
    co_await push(session, "delay_info_push", payloads::drc_delay_info(drone));
    co_await push(session, "drc_drone_state_push", payloads::drc_drone_state(drone));
    co_await push(session, "drc_batteries_info_push", payloads::drc_batteries_info(drone));
    co_await push_camera(session);
}

asio::awaitable<void> DrcControl::push_camera(std::shared_ptr<transport::Session> session) {
    camera_.refresh();
    co_await push(session, "drc_camera_state_push", payloads::drc_camera_state(ctx_.drone));
    co_await push(session, "drc_camera_osd_info_push", payloads::drc_camera_osd_info(ctx_.drone));
}

asio::awaitable<void> DrcControl::pushes(std::shared_ptr<transport::Session> session) {
    for (std::int64_t count = 0; session_ == session; ++count) {
        if (ctx_.clock().monotonic() - last_heartbeat_ >= drc_heartbeat_timeout_s) {
            // close() cancels this task. Leave from a task of its own, so that the notify goes out.
            ctx_.spawn(idle_exit(), "drc-idle-exit");
            co_return;
        }
        co_await push(session, "osd_info_push", payloads::drc_osd_info(ctx_.drone));
        const auto every = std::max<std::int64_t>(1, osd_hz_ / hsi_hz_);
        if (count % every == 0) co_await push_states(session);
        co_await ctx_.clock().sleep(1.0 / osd_hz_);
    }
}

asio::awaitable<void> DrcControl::reply(Json request, std::string method, code::Result result) {
    const auto session = session_;
    if (!session) co_return;
    const Json data = request.value("data", Json::object());
    Json body = {{"result", result}};
    // The DJI drone_control reply repeats the command seq in output.
    if (method == "drone_control" || method == "stick_control")
        body["output"] = {{"seq", data.is_object() ? data.value("seq", Json(0)) : Json(0)}};
    Json message = {{"method", std::move(method)}, {"data", std::move(body)}};
    // The backend waits for the seq of its request, a UUID next to data.
    if (request.contains("seq")) message["seq"] = request["seq"];
    co_await session->publish(topic::drc_up(ctx_.dock_sn()), std::move(message), 1);
}

asio::awaitable<void> DrcControl::on_down(Json message) {
    // One topic carries every DRC command. The method field selects the handler.
    const auto method = message.value("method", "");
    const Json data =
        message.contains("data") && message["data"].is_object() ? message["data"] : Json::object();
    if (method.empty()) {
        logger("dock_sim.drc")->info("drc <no method>: {}", message.dump());
        co_return;
    }
    if (!routing::routed(ctx_.cfg().device.dock_model, routing::Channel::drc_down, method)) {
        logger("dock_sim.drc")
            ->warn("drc {}: the {} gateway does not route it, no reply", method,
                   ctx_.cfg().device.dock_model);
        co_return;
    }
    if (method == "heart_beat") {
        last_heartbeat_ = ctx_.clock().monotonic();
        const auto session = session_;
        if (!session) co_return;
        // The real dock returns the received message with no change.
        co_await session->publish(topic::drc_up(ctx_.dock_sn()), std::move(message), 0);
        co_return;
    }
    auto code = code::Result::ok;
    bool camera = false;
    try {
        if (const auto set = camera_.drc_set(method, data)) {
            code = *set;
            camera = true;
        } else if (method == "drone_control" || method == "stick_control") {
            manual_.stick(data);
        } else if (method == "drone_emergency_stop") {
            code = manual_.emergency_stop();
        } else if (method == "drc_emergency_landing" || method == "drc_force_landing") {
            code = manual_.land_here();
        } else if (method == "drc_night_lights_state_set") {
            ctx_.drone.night_lights_state =
                static_cast<int>(integer(data, "night_lights_state", 0));
        } else if (method == "drc_stealth_state_set") {
            ctx_.drone.stealth_state = static_cast<int>(integer(data, "stealth_state", 0));
        }
    } catch (const BadParameter& e) {
        logger("dock_sim.drc")->warn("drc {}: {}", method, e.what());
        code = code::Result::param_error;
    }
    if (method != "drone_control" && method != "stick_control")
        logger("dock_sim.drc")
            ->info("drc {}: {} -> {}", method, data.dump(), std::to_underlying(code));
    co_await reply(message, method, code);
    // The dock reports the camera state when it changes.
    if (camera && code == code::Result::ok && session_) co_await push_camera(session_);
    if (method == "drc_initial_state_subscribe" && session_) {
        const auto session = session_;
        co_await push(session, "osd_info_push", payloads::drc_osd_info(ctx_.drone));
        co_await push_states(session);
    }
}

}  // namespace docksim::services
