#include "harness.hpp"

#include "log/log.hpp"
#include "util/digest.hpp"
#include "wayline/wpml.hpp"

#include <asio/co_spawn.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace docksim::golden {

namespace {

const std::filesystem::path source_dir{DOCKSIM_SOURCE_DIR};

Json storage_output() {
    return {
        {"bucket", "media"},
        {"endpoint", "https://storage.example.invalid"},
        {"object_key_prefix", "org-1"},
        {"provider", "minio"},
        {"region", "us-east-1"},
        {"credentials",
         {{"access_key_id", "AK"},
          {"access_key_secret", "SK"},
          {"security_token", ""},
          {"expire", 3600}}},
    };
}

std::vector<std::byte> read_bytes(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + path.string());
    std::vector<char> chars{std::istreambuf_iterator<char>(in), {}};
    std::vector<std::byte> out(chars.size());
    std::ranges::transform(chars, out.begin(), [](char c) { return static_cast<std::byte>(c); });
    return out;
}

class FakeStorage final : public media::Storage {
public:
    FakeStorage(const Json& output, std::vector<std::pair<std::string, std::size_t>>& puts,
                int& fail_put)
        : puts_(puts), fail_put_(fail_put) {
        prefix_ = output.value("object_key_prefix", "");
        while (!prefix_.empty() && prefix_.front() == '/') prefix_.erase(0, 1);
        while (!prefix_.empty() && prefix_.back() == '/') prefix_.pop_back();
    }
    [[nodiscard]] std::string object_key(std::string_view name) const override {
        return prefix_.empty() ? std::string(name) : std::format("{}/{}", prefix_, name);
    }
    void put(const std::string& key, std::span<const std::byte> blob,
             std::string_view /*content_type*/) override {
        if (fail_put_ == static_cast<int>(puts_.size()) + 1) {
            fail_put_ = 0;
            throw std::runtime_error("storage is down");
        }
        puts_.emplace_back(key, blob.size());
    }

private:
    std::string prefix_;
    std::vector<std::pair<std::string, std::size_t>>& puts_;
    int& fail_put_;
};

constexpr std::array<std::pair<const char*, const char*>, 3> volatile_text{{
    {"tid", "<tid>"},
    {"bid", "<bid>"},
    {"nonce", "<nonce>"},
}};

}  // namespace

Json normalize(const Json& value) {
    if (value.is_array()) {
        Json out = Json::array();
        for (const auto& item : value) out.push_back(normalize(item));
        return out;
    }
    if (!value.is_object()) return value;
    Json out = Json::object();
    // The dock osd draws its weather from a random source. Only the range is fixed.
    const bool weather = value.contains("wind_speed") && value.contains("humidity");
    for (const auto& [key, item] : value.items()) {
        if (const auto* v = std::ranges::find(volatile_text, key,
                                              [](const auto& p) { return std::string(p.first); });
            v != volatile_text.end()) {
            out[key] = v->second;
        } else if (key == "timestamp" || key == "ack_timestamp") {
            out[key] = 0;
        } else if (weather && (key == "wind_speed" || key == "environment_temperature" ||
                               key == "temperature" || key == "humidity")) {
            out[key] = "<weather>";
        } else {
            out[key] = normalize(item);
        }
    }
    return out;
}

std::string areas_geojson() {
    return Json{
        {"type", "FeatureCollection"},
        {"features",
         Json::array(
             {{{"type", "Feature"},
               {"id", "nfz-north"},
               {"geofence_type", "nfz"},
               {"geometry", {{"type", "Point"}, {"coordinates", {11.401, 47.272}}}},
               {"properties", {{"subType", "Circle"}, {"radius", 200}, {"enable", true}}}}})},
    }
        .dump();
}

Json file_list_reply(const std::string& name, const std::string& url, const std::string& content) {
    return {{"result", 0},
            {"output",
             {{"files", Json::array({{{"name", name},
                                      {"url", url},
                                      {"checksum", digest::sha256_hex(digest::bytes(content))},
                                      {"size", content.size()}}})}}}};
}

FakeCloud::FakeCloud() {
    replies = {
        {"config",
         {{"app_id", "126988"},
          {"app_key", "key"},
          {"app_license", "license"},
          {"ntp_server_host", "ntp.example.invalid"}}},
        {"airport_bind_status",
         {{"result", 0},
          {"output",
           {{"bind_status",
             Json::array({{{"sn", dock_sn}, {"is_device_bind_organization", true}},
                          {{"sn", drone_sn}, {"is_device_bind_organization", true}}})}}}}},
        {"storage_config_get", {{"result", 0}, {"output", storage_output()}}},
        {"flight_areas_get", file_list_reply("areas.json", areas_url, areas_geojson())},
    };
}

std::optional<std::pair<std::string, Json>> FakeCloud::reply_for(const std::string& topic,
                                                                 const Json& payload) const {
    if (!topic.ends_with("/requests")) return std::nullopt;
    const auto method = payload.value("method", "");
    const auto it = replies.find(method);
    if (it == replies.end()) return std::nullopt;
    // thing/product/{sn}/requests
    const auto sn = topic.substr(14, topic.size() - 14 - 9);
    return std::pair{std::format("thing/product/{}/requests_reply", sn),
                     Json{{"tid", payload["tid"]},
                          {"bid", payload["bid"]},
                          {"method", method},
                          {"timestamp", 0},
                          {"data", it->second}}};
}

void FakeTransport::on(std::string suffix, transport::Handler handler) {
    handlers_.emplace_back(std::move(suffix), std::move(handler));
}

void FakeTransport::subscribe(std::string topic, std::optional<int> qos) {
    subscriptions.emplace_back(std::move(topic), qos.value_or(qos_));
}

asio::awaitable<void> FakeTransport::publish(std::string topic, Json payload,
                                             std::optional<int> qos) {
    published.push_back({topic, qos.value_or(qos_), payload});
    if (responder != nullptr) {
        if (auto answer = responder->reply_for(topic, payload)) {
            co_await inject(std::move(answer->first), std::move(answer->second));
        }
    }
}

asio::awaitable<void> FakeTransport::inject(std::string topic, Json payload) {
    for (const auto& [suffix, handler] : handlers_) {
        if (!topic.ends_with(suffix)) continue;
        try {
            co_await handler(topic, payload);
        } catch (const std::exception& e) {
            // As the MQTT transport does: a failing handler is logged, the next one still runs.
            log::get("dock_sim.transport")->error("handler failed for {}: {}", topic, e.what());
        }
    }
}

asio::awaitable<void> FakeTransport::run(bool /*reconnect*/) {
    co_await closed_.wait();
}

Config golden_config() {
    Config cfg;
    cfg.media.renderer = PhotoRenderer::synthetic;
    cfg.media.image_size = {64, 48};
    cfg.media.upload_delay = 3600.0;
    cfg.live.source = LiveSource::pattern;
    // The snapshots were recorded with this step.
    cfg.flight.tick = 0.5;
    // The snapshots have no flight_areas_get after the login.
    cfg.maintenance.sync_flight_areas_on_start = false;
    return cfg;
}

Json wire(const std::vector<Published>& published) {
    Json out = Json::array();
    for (const auto& p : published) {
        out.push_back({{"topic", p.topic}, {"qos", p.qos}, {"payload", normalize(p.payload)}});
    }
    return out;
}

Driver::Driver(Config cfg)
    : guard_(io_.get_executor()),
      cfg_(std::move(cfg)),
      virtual_(frozen_now),
      clock_(virtual_, frozen_now),
      transport_(std::make_unique<FakeTransport>(io_.get_executor(), cfg_.broker.qos,
                                                 cfg_.device.dock_sn)) {
    transport_->responder = &cloud_;
    files[areas_url] = areas_geojson();
    const auto kmz = read_bytes(source_dir / "test/golden/fixtures/mission.kmz");
    sim::Dependencies deps{
        .sessions =
            [this](const BrokerConfig& broker, const std::string& client_id) {
                auto session =
                    std::make_shared<FakeTransport>(io_.get_executor(), broker.qos, client_id);
                drc_sessions_.push_back(session);
                return std::shared_ptr<transport::Session>(session);
            },
        .storage =
            [this](const Json& output) {
                return std::make_unique<FakeStorage>(output, puts, fail_put);
            },
        .photos =
            [](const wayline::Shot&, std::pair<int, int>, const std::string&,
               const media::PhotoMeta&) { return std::vector<std::byte>(16, std::byte{0xFF}); },
        .download = [this, kmz](std::string url) -> asio::awaitable<wayline::Wayline> {
            downloads.push_back(url);
            if (url.ends_with("broken.kmz")) throw std::runtime_error("HTTP 403");
            co_return wayline::parse_kmz(kmz);
        },
        .streamer = std::make_unique<OkStreamer>(),
        .verify_license = [](std::string_view, std::string_view, std::string_view) { return true; },
        .weather_seed = 1234,
        .fetch = [this](std::string url) -> asio::awaitable<std::vector<std::byte>> {
            fetched.push_back(url);
            const auto it = files.find(url);
            if (it == files.end()) throw std::runtime_error("HTTP 404");
            const auto body = digest::bytes(it->second);
            co_return std::vector<std::byte>(body.begin(), body.end());
        },
    };
    sim_ = std::make_unique<sim::DockSim>(io_.get_executor(), cfg_, *transport_, clock_, workers_,
                                          std::move(deps));
}

Driver::~Driver() {
    try {
        drive(sim_->manual().shutdown());
        drive(sim_->mission().shutdown());
        drive(sim_->ctx().cancel_all());
    } catch (...) {  // NOLINT(bugprone-empty-catch): teardown in a destructor must not throw.
    }
    sim_.reset();
    workers_.join();
}

void Driver::drive(asio::awaitable<void> step) {
    bool done = false;
    std::exception_ptr failure;
    asio::co_spawn(io_, std::move(step), [&](std::exception_ptr e) {
        failure = std::move(e);
        done = true;
    });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{20};
    while (!done && std::chrono::steady_clock::now() < deadline) {
        io_.poll();
        if (done) break;
        // Jump the clock only when every task waits on it. A task that waits
        // on a worker thread is not idle, so then wait a little real time.
        if (!virtual_.jump()) io_.run_one_for(std::chrono::milliseconds{2});
    }
    if (failure) std::rethrow_exception(failure);
    if (!done) throw std::runtime_error("step did not finish in real time");
}

void Driver::connect() {
    drive([this]() -> asio::awaitable<void> {
        co_await sim_->start(false);
        co_await sim_->ctx().settle();
    }());
}

void Driver::osd_tick() {
    drive(sim_->osd_tick());
}

void Driver::service(const std::string& method, Json data) {
    drive([this, method, data = std::move(data)]() -> asio::awaitable<void> {
        co_await transport_->inject(
            std::format("thing/product/{}/services", cfg_.device.dock_sn),
            {{"tid", "t"}, {"bid", "b"}, {"timestamp", 0}, {"method", method}, {"data", data}});
        if (run_spawned) co_await sim_->ctx().settle();
    }());
}

void Driver::property_set(const std::string& sn, Json data) {
    drive(transport_->inject(
        std::format("thing/product/{}/property/set", sn),
        {{"tid", "t"}, {"bid", "b"}, {"timestamp", 0}, {"data", std::move(data)}}));
}

void Driver::wait_mission() {
    drive(sim_->mission().wait());
}

void Driver::upload_media(const std::string& flight_id) {
    drive(sim_->uploads().upload(flight_id));
}

void Driver::drc_inject(const std::string& topic, Json payload) {
    drive(drc_sessions_.back()->inject(topic, std::move(payload)));
}

void Driver::advance(double seconds) {
    drive([this, seconds]() -> asio::awaitable<void> { co_await clock_.sleep(seconds); }());
}

void Driver::wait_manual() {
    drive(sim_->manual().wait());
}

Json Driver::subscriptions() const {
    Json out = Json::array();
    for (const auto& [topic, qos] : transport_->subscriptions)
        out.push_back(Json::array({topic, qos}));
    return out;
}

Json Driver::drc_wire() const {
    const auto& session = *drc_sessions_.back();
    Json subs = Json::array();
    for (const auto& [topic, qos] : session.subscriptions)
        subs.push_back(Json::array({topic, qos}));
    Json out = Json::array({{{"client_id", session.client_id()}, {"subscriptions", subs}}});
    for (auto& item : wire(session.published)) out.push_back(std::move(item));
    return out;
}

bool same_json(const Json& a, const Json& b) {
    return nlohmann::json::parse(a.dump()) == nlohmann::json::parse(b.dump());
}

Json load_snapshot(const std::string& name) {
    std::ifstream in(source_dir / "test/golden/snapshots" / (name + ".json"));
    if (!in) throw std::runtime_error("no snapshot " + name);
    return Json::parse(in);
}

void dump_actual(const std::string& name, const Json& actual) {
    const auto dir = std::filesystem::temp_directory_path() / "docksim-golden";
    std::filesystem::create_directories(dir);
    std::ofstream(dir / (name + ".actual.json"))
        << nlohmann::json::parse(actual.dump()).dump(2) << "\n";
}

}  // namespace docksim::golden
