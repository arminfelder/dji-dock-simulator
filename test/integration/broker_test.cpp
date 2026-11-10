// The dock against a real EMQX broker and a scripted cloud. Needs docker and
// the emqx/emqx image, or a broker at DOCKSIM_TEST_BROKER=host:port.
// Run with: ctest --preset gcc-debug -L integration

#include "../golden/harness.hpp"
#include "sim/dock_sim.hpp"
#include "transport/mqtt.hpp"
#include "wayline/wpml.hpp"

#include <asio/co_spawn.hpp>
#include <asio/io_context.hpp>
#include <asio/thread_pool.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <random>
#include <thread>
#include <tuple>

using namespace docksim;

namespace {

constexpr const char* image = "emqx/emqx:5.8.8";

std::string shell(const std::string& command) {
    std::string out;
    if (FILE* pipe = popen(command.c_str(), "r"); pipe != nullptr) {
        std::array<char, 256> buf{};
        while (fgets(buf.data(), static_cast<int>(buf.size()), pipe) != nullptr) out += buf.data();
        if (pclose(pipe) != 0) return {};
    }
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
    return out;
}

// One broker for the whole test binary.
class Broker {
public:
    static Broker& get() {
        static Broker broker;
        return broker;
    }
    [[nodiscard]] bool available() const { return port_ != 0; }
    [[nodiscard]] const std::string& host() const { return host_; }
    [[nodiscard]] int port() const { return port_; }
    [[nodiscard]] const std::string& why() const { return why_; }

    Broker(const Broker&) = delete;
    Broker& operator=(const Broker&) = delete;

    Broker(Broker&&) = delete;

    Broker& operator=(Broker&&) = delete;
    ~Broker() {
        if (!container_.empty()) std::ignore = shell("docker stop " + container_ + " 2>/dev/null");
    }

private:
    Broker() {
        if (const char* given = std::getenv("DOCKSIM_TEST_BROKER")) {
            const std::string text = given;
            const auto colon = text.rfind(':');
            host_ = text.substr(0, colon);
            port_ = std::stoi(text.substr(colon + 1));
            return;
        }
        std::random_device rd;
        const int port = 20000 + static_cast<int>(rd() % 20000);
        container_ = std::format("docksim-test-{:08x}", rd());
        const auto id = shell(std::format(
            "docker run -d --rm --name {} -p 127.0.0.1:{}:1883 {} 2>&1", container_, port, image));
        if (id.empty()) {
            why_ = std::format("cannot start {} with docker", image);
            container_.clear();
            return;
        }
        // EMQX takes some seconds before it accepts MQTT.
        for (int i = 0; i < 120; ++i) {
            if (!shell(std::format("docker exec {} emqx ctl status 2>/dev/null | grep -q 'is "
                                   "started' && echo up",
                                   container_))
                     .empty()) {
                host_ = "127.0.0.1";
                port_ = port;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds{500});
        }
        why_ = "broker did not come up";
    }

    std::string host_;
    int port_ = 0;
    std::string container_;
    std::string why_;
};

// Answers requests like the backend and records everything the dock sends.
class Cloud {
public:
    Cloud(const sim::Executor& ex, const BrokerConfig& cfg)
        : ex_(ex), transport_(ex, cfg, "cloud"), changed_(ex) {
        transport_.subscribe("thing/product/+/#", 1);
        transport_.subscribe("sys/product/+/status", 1);
        transport_.on("", [this](std::string topic, Json payload) -> asio::awaitable<void> {
            seen_.emplace_back(topic, payload);
            changed_.set();
            const auto method = payload.value("method", "");
            if (topic.ends_with("/requests") && replies_.replies.contains(method)) {
                Json answer = payload;
                answer["data"] = replies_.replies.at(method);
                co_await transport_.publish(topic + "_reply", answer, 1);
            }
        });
    }

    transport::MqttTransport& transport() { return transport_; }

    // Number of messages seen so far. wait() can skip them.
    [[nodiscard]] std::size_t seen() const { return seen_.size(); }

    template <class Match>
    asio::awaitable<Json> wait(Match match, std::chrono::seconds timeout = std::chrono::seconds{30},
                               std::size_t from = 0) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (true) {
            for (std::size_t i = from; i < seen_.size(); ++i) {
                const auto& [topic, payload] = seen_[i];
                if (match(topic, payload)) co_return payload;
            }
            const auto left = deadline - std::chrono::steady_clock::now();
            if (left <= std::chrono::steady_clock::duration::zero())
                throw std::runtime_error("expected message did not arrive");
            changed_.clear();
            std::ignore = co_await changed_.wait_for(left);
        }
    }

    asio::awaitable<Json> service(const std::string& method, Json data) {
        const auto tid = std::format("t-{}", ++counter_);
        co_await transport_.publish(
            std::format("thing/product/{}/services", golden::dock_sn),
            {{"tid", tid}, {"bid", tid}, {"timestamp", 0}, {"method", method}, {"data", data}}, 1);
        const auto reply = co_await wait([&](const std::string& t, const Json& p) {
            return t.ends_with("/services_reply") && p.value("tid", "") == tid;
        });
        co_return reply["data"];
    }

    [[nodiscard]] std::vector<Json> events(const std::string& method) const {
        std::vector<Json> out;
        for (const auto& [topic, payload] : seen_) {
            if (topic.ends_with("/events") && payload.value("method", "") == method)
                out.push_back(payload["data"]);
        }
        return out;
    }

private:
    sim::Executor ex_;
    transport::MqttTransport transport_;
    sim::Event changed_;
    golden::FakeCloud replies_;
    std::vector<std::pair<std::string, Json>> seen_;
    int counter_ = 0;
};

class NullStorage final : public media::Storage {
public:
    explicit NullStorage(std::string prefix) : prefix_(std::move(prefix)) {}
    [[nodiscard]] std::string object_key(std::string_view name) const override {
        return std::format("{}/{}", prefix_, name);
    }
    void put(const std::string& /*key*/, std::span<const std::byte> /*blob*/,
             std::string_view /*content_type*/) override {}

private:
    std::string prefix_;
};

Config broker_config(const Broker& broker) {
    Config cfg;
    cfg.broker.host = broker.host();
    cfg.broker.port = broker.port();
    cfg.broker.tls = false;
    cfg.broker.username = "dock";
    cfg.media.image_size = {64, 48};
    cfg.media.upload_delay = 0.0;
    cfg.live.enabled = false;
    cfg.flight.tick = 0.05;
    cfg.flight.speed = 60.0;
    cfg.flight.climb_rate = 40.0;
    cfg.flight.prepare_seconds = 0.3;
    cfg.flight.land_seconds = 0.1;
    return cfg;
}

template <class Body>
void with_dock(Body body) {
    auto& broker = Broker::get();
    if (!broker.available()) SKIP(broker.why());
    const Config cfg = broker_config(broker);
    asio::io_context io;
    asio::thread_pool workers(2);
    std::exception_ptr failure;

    const auto scenario = [&]() -> asio::awaitable<void> {
        const auto ex = co_await asio::this_coro::executor;
        std::ifstream in(
            std::filesystem::path(DOCKSIM_SOURCE_DIR) / "test/golden/fixtures/mission.kmz",
            std::ios::binary);
        const std::string blob{std::istreambuf_iterator<char>(in), {}};

        Cloud cloud(ex, cfg.broker);
        transport::MqttTransport transport(ex, cfg.broker, cfg.device.dock_sn);
        sim::RealClock clock;
        sim::DockSim dock(
            ex, cfg, transport, clock, workers,
            {
                .sessions =
                    [ex](const BrokerConfig& b, const std::string& id) {
                        return std::shared_ptr<transport::Session>(
                            std::make_shared<transport::MqttTransport>(ex, b, id));
                    },
                .storage =
                    [](const Json& o) {
                        return std::make_unique<NullStorage>(o.value("object_key_prefix", ""));
                    },
                .photos = [](const wayline::Shot&, std::pair<int, int>, const std::string&,
                             const media::PhotoMeta&) { return std::vector<std::byte>(16); },
                .download = [blob](std::string) -> asio::awaitable<wayline::Wayline> {
                    co_return wayline::parse_kmz(std::as_bytes(std::span(blob)));
                },
                .streamer = nullptr,
                .verify_license = [](std::string_view, std::string_view,
                                     std::string_view) { return true; },
                .weather_seed = std::nullopt,
            });
        sim::TaskGroup sessions(ex, {});
        sessions.spawn(cloud.transport().run(true), "cloud");
        sessions.spawn(transport.run(true), "dock");
        co_await cloud.transport().wait_connected();
        co_await transport.wait_connected();
        co_await dock.start(true);
        try {
            co_await body(cloud, broker);
        } catch (...) {
            failure = std::current_exception();
        }
        co_await dock.stop();
        co_await dock.ctx().cancel_all();
        co_await sessions.cancel_all();
    };
    asio::co_spawn(io, scenario(), [&](const std::exception_ptr& e) {
        if (e && !failure) failure = e;
    });
    io.run();
    workers.join();
    if (failure) std::rethrow_exception(failure);
}

}  // namespace

TEST_CASE("integration: online, mission, media and drc", "[integration]") {
    with_dock([](Cloud& cloud, const Broker& broker) -> asio::awaitable<void> {
        co_await cloud.wait([](const std::string&, const Json& p) {
            return p.value("method", "") == "update_topo";
        });
        co_await cloud.wait([](const std::string& t, const Json& p) {
            return t == std::format("thing/product/{}/osd", golden::dock_sn) &&
                   p["data"].contains("job_number");
        });
        co_await cloud.wait([](const std::string&, const Json& p) {
            return p.value("method", "") == "storage_config_get";
        });

        const Json prepare = {{"flight_id", "f-1"},
                              {"file", {{"url", "http://kmz.invalid/f.kmz"}}},
                              {"rth_altitude", 60}};
        CHECK((co_await cloud.service("flighttask_prepare", prepare))["result"] == 0);
        CHECK((co_await cloud.service("flighttask_execute", {{"flight_id", "f-1"}}))["result"] ==
              0);
        co_await cloud.wait(
            [](const std::string&, const Json& p) {
                return p.value("method", "") == "flighttask_progress" &&
                       p["data"]["output"]["status"] == "ok";
            },
            std::chrono::seconds{60});
        co_await cloud.wait([](const std::string&, const Json& p) {
            return p.value("method", "") == "file_upload_callback" &&
                   p["data"]["flight_task"]["uploaded_file_count"] == 2;
        });
        std::vector<std::string> names;
        for (const auto& e : cloud.events("file_upload_callback"))
            names.push_back(e["file"]["name"]);
        CHECK(names == std::vector<std::string>{"DJI_f-1_0001.jpeg", "DJI_f-1_0002.jpeg"});

        const Json enter = {{"mqtt_broker",
                             {{"address", std::format("tcp://{}:{}", broker.host(), broker.port())},
                              {"username", "drc"},
                              {"password", "x"},
                              {"client_id", std::format("org-{}", golden::dock_sn)},
                              {"enable_tls", false}}}};
        CHECK((co_await cloud.service("drc_mode_enter", enter))["result"] == 0);
        CHECK(std::ranges::contains(cloud.events("drc_status_notify"),
                                    Json{{"result", 0}, {"drc_state", 2}}));
        co_await cloud.transport().publish(
            std::format("thing/product/{}/drc/down", golden::dock_sn),
            {{"method", "heart_beat"}, {"data", {{"seq", 3}, {"timestamp", 5}}}}, 1);
        const auto beat = co_await cloud.wait([](const std::string& t, const Json& p) {
            return t.ends_with("/drc/up") && p.value("method", "") == "heart_beat";
        });
        CHECK(beat["data"]["seq"] == 3);
        CHECK((co_await cloud.service("drc_mode_exit", Json::object()))["result"] == 0);
        CHECK(std::ranges::contains(cloud.events("drc_status_notify"),
                                    Json{{"result", 0}, {"drc_state", 0}}));
    });
}

TEST_CASE("integration: unknown service and bad parameters", "[integration]") {
    with_dock([](Cloud& cloud, const Broker&) -> asio::awaitable<void> {
        CHECK((co_await cloud.service("no_such_method", Json::object()))["result"] == 314001);
        CHECK((co_await cloud.service("camera_look_at", Json::object()))["result"] == 514171);
    });
}

TEST_CASE("integration: takeoff, drc sticks and return to home", "[integration]") {
    with_dock([](Cloud& cloud, const Broker& broker) -> asio::awaitable<void> {
        const auto& device = golden::golden_config().device;
        const auto drc_down = std::format("thing/product/{}/drc/down", golden::dock_sn);
        const auto drc_method = [](std::string method) {
            return [method](const std::string& t, const Json& p) {
                return t.ends_with("/drc/up") && p.value("method", "") == method;
            };
        };
        co_await cloud.wait([](const std::string&, const Json& p) {
            return p.value("method", "") == "update_topo";
        });
        const Json enter = {{"mqtt_broker",
                             {{"address", std::format("tcp://{}:{}", broker.host(), broker.port())},
                              {"username", "drc"},
                              {"password", "x"},
                              {"client_id", std::format("org-{}", golden::dock_sn)},
                              {"enable_tls", false}}},
                            {"osd_frequency", 10},
                            {"hsi_frequency", 1}};
        CHECK((co_await cloud.service("drc_mode_enter", enter))["result"] == 0);
        co_await cloud.wait(drc_method("osd_info_push"));
        CHECK((co_await cloud.service("flight_authority_grab", Json::object()))["result"] == 0);

        const Json takeoff = {{"flight_id", "takeoff-1"},
                              {"target_latitude", device.latitude + 60.0 / 111'195.0},
                              {"target_longitude", device.longitude},
                              {"target_height", device.altitude + 40.0},
                              {"security_takeoff_height", 30},
                              {"rth_altitude", 50},
                              {"rth_mode", 1},
                              {"max_speed", 15},
                              {"commander_mode_lost_action", 1},
                              {"commander_flight_mode", 1},
                              {"commander_flight_height", 40}};
        CHECK((co_await cloud.service("takeoff_to_point", takeoff))["result"] == 0);
        co_await cloud.wait(
            [](const std::string&, const Json& p) {
                return p.value("method", "") == "takeoff_to_point_progress" &&
                       p["data"]["status"] == "task_finish";
            },
            std::chrono::seconds{30});

        // The web UI sends a stick command every 50 ms.
        const auto before = cloud.seen();
        sim::RealClock clock;
        for (int seq = 0; seq < 10; ++seq) {
            co_await cloud.transport().publish(
                drc_down,
                {{"method", "drone_control"},
                 {"data", {{"seq", seq}, {"x", 5.0}, {"y", 0.0}, {"h", 0.0}, {"w", 0.0}}}},
                0);
            co_await clock.sleep(0.05);
        }
        const auto answer =
            co_await cloud.wait(drc_method("drone_control"), std::chrono::seconds{10}, before);
        CHECK(answer["data"]["result"] == 0);
        co_await cloud.wait(
            [](const std::string& t, const Json& p) {
                return t.ends_with("/drc/up") && p.value("method", "") == "osd_info_push" &&
                       p["data"]["horizontal_speed"].get<double>() > 1.0;
            },
            std::chrono::seconds{10}, before);

        const auto camera = co_await cloud.wait(drc_method("drc_camera_osd_info_push"));
        CHECK(camera["data"]["payload_index"] == "80-0-0");
        CHECK((co_await cloud.service("camera_photo_take",
                                      {{"payload_index", "80-0-0"}}))["result"] == 0);
        const auto photo = co_await cloud.wait([](const std::string&, const Json& p) {
            return p.value("method", "") == "camera_photo_take_progress";
        });
        CHECK(photo["data"]["output"]["status"] == "ok");

        const Json poi = {{"latitude", device.latitude + 80.0 / 111'195.0},
                          {"longitude", device.longitude},
                          {"height", device.altitude}};
        CHECK((co_await cloud.service("poi_mode_enter", poi))["result"] == 0);
        co_await cloud.wait([](const std::string&, const Json& p) {
            return p.value("method", "") == "poi_status_notify" &&
                   p["data"]["status"] == "in_progress";
        });
        CHECK((co_await cloud.service("poi_mode_exit", Json::object()))["result"] == 0);
        co_await cloud.wait([](const std::string&, const Json& p) {
            return p.value("method", "") == "poi_status_notify" && p["data"]["status"] == "ok";
        });

        CHECK((co_await cloud.service("return_home", Json::object()))["result"] == 0);
        const auto info = co_await cloud.wait([](const std::string&, const Json& p) {
            return p.value("method", "") == "return_home_info";
        });
        CHECK(info["data"]["home_dock_sn"] == golden::dock_sn);
        const auto landed = cloud.seen();
        co_await cloud.wait(
            [](const std::string& t, const Json& p) {
                return t == std::format("thing/product/{}/osd", golden::dock_sn) &&
                       p["data"].contains("drone_in_dock") && p["data"]["drone_in_dock"] == 1 &&
                       p["data"]["mode_code"] == 0;
            },
            std::chrono::seconds{60}, landed);
        CHECK((co_await cloud.service("drc_mode_exit", Json::object()))["result"] == 0);
    });
}
