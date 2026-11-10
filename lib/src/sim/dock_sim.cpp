#include "sim/dock_sim.hpp"

#include "cloud/license.hpp"
#include "device/catalog.hpp"
#include "device/weather.hpp"
#include "log/log.hpp"
#include "media/photo.hpp"
#include "protocol/envelope.hpp"
#include "protocol/payloads.hpp"
#include "protocol/routing.hpp"
#include "protocol/topics.hpp"
#include "services/peripherals.hpp"

#include <format>
#include <stdexcept>

namespace docksim::sim {

namespace {

const auto& logger() {
    static const auto log = log::get("dock_sim");
    return log;
}

services::Fetch no_fetch() {
    return [](std::string url) -> asio::awaitable<std::vector<std::byte>> {
        throw std::runtime_error(std::format("no file download in this build: {}", url));
        co_return std::vector<std::byte>{};
    };
}

using LicenseCheck = std::function<bool(std::string_view, std::string_view, std::string_view)>;

// Without a key, the simulator accepts every license. The real dock does not.
LicenseCheck key_check(const std::optional<std::string>& key) {
    if (!key) {
        return [](std::string_view, std::string_view, std::string_view) {
            logger()->warn("license not checked: device.license_key is not set");
            return true;
        };
    }
    return [key = *key](std::string_view id, std::string_view app_key, std::string_view lic) {
        return license::verify(key, id, app_key, lic);
    };
}

const Json& output_of(const Json& reply) {
    static const Json empty = Json::object();
    const auto it = reply.find("output");
    return it != reply.end() && it->is_object() ? *it : empty;
}

}  // namespace

DockSim::DockSim(const Executor& ex, const Config& cfg, transport::Transport& transport,
                 Clock& clock, asio::thread_pool& workers, Dependencies deps)
    : ctx_(ex, cfg, clock, transport, workers),
      sampler_(ex, ctx_.drone, &ctx_.dock.weather),
      cameras_(std::make_shared<Cameras>(cfg, sampler_.function())),
      uploads_(
          ctx_, deps.storage,
          deps.photos
              ? std::move(deps.photos)
              : media::PhotoMaker(
                    [cameras = cameras_](const wayline::Shot& shot, std::pair<int, int> size,
                                         const std::string& caption, const media::PhotoMeta& meta) {
                        return media::encode_photo(cameras->photo(shot, size, caption), meta);
                    })),
      mission_(ctx_, deps.download, [this](const wayline::Task& task) { uploads_.queue(task); }),
      manual_(ctx_, mission_, std::move(deps.download), uploads_),
      camera_(ctx_, manual_),
      live_(ctx_,
            deps.streamer ? std::move(deps.streamer) : std::make_unique<live::NullStreamer>()),
      drc_(ctx_, std::move(deps.sessions), manual_, camera_),
      payload_(ctx_),
      maintenance_(ctx_, {.offline = [this] { return reboot_offline(); },
                          .online = [this] { return reboot_online(); }}),
      areas_(ctx_, deps.fetch ? deps.fetch : no_fetch()),
      offline_map_(ctx_, deps.fetch ? deps.fetch : no_fetch()),
      logs_(ctx_, std::move(deps.storage)),
      hms_(ctx_),
      verify_license_(deps.verify_license ? std::move(deps.verify_license)
                                          : key_check(cfg.device.license_key)) {
    if (deps.weather_seed) ctx_.dock.rng.seed(*deps.weather_seed);
    const auto& sn = cfg.device.dock_sn;

    maintenance_.register_services(registry_, cfg.device.dock_model);
    services::register_flighttask(registry_, mission_, cfg.device.dock_model);
    manual_.register_services(registry_, mission_);
    mission_.refuse_while([this] { return manual_.flying(); });
    mission_.preflight([this] { return preflight(); });
    manual_.preflight([this] { return preflight(); });
    manual_.guard(
        [this](double latitude, double longitude) { return areas_.forbidden(latitude, longitude); },
        [this] { return areas_.observe(); });
    mission_.on_step([this] { std::ignore = areas_.observe(); });
    areas_.register_services(registry_);
    offline_map_.register_services(registry_);
    logs_.register_services(registry_);
    services::register_psdk(registry_);
    services::register_remote_unlocking(registry_);
    payload_.register_services(registry_);
    camera_.register_services(registry_);
    live_.register_services(registry_);
    drc_.register_services(registry_);
    uploads_.register_services(registry_);

    const auto ignore = [](std::string, Json) -> asio::awaitable<void> { co_return; };
    transport.on("/services", [this](std::string, Json m) -> asio::awaitable<void> {
        co_await on_services(std::move(m));
    });
    transport.on("/status_reply", [](std::string, Json m) -> asio::awaitable<void> {
        logger()->info("status_reply result={}",
                       m.value("data", Json::object()).value("result", Json()).dump());
        co_return;
    });
    transport.on("/events_reply", ignore);
    transport.on("/property/set", [this](std::string topic, Json m) -> asio::awaitable<void> {
        co_await on_property_set(std::move(topic), std::move(m));
    });
    transport.on("/state_reply", ignore);
    // The real dock subscribes here. Its answer is not known yet, so only log.
    transport.on("/network/probe", [](std::string, Json m) -> asio::awaitable<void> {
        logger()->info("network/probe: {}", m.dump());
        co_return;
    });
    for (auto t : {topic::services(sn), topic::requests_reply(sn), topic::status_reply(sn),
                   topic::events_reply(sn), topic::property_set(sn),
                   topic::property_set(cfg.device.drone_sn), topic::state_reply(sn),
                   topic::network_probe(sn)}) {
        transport.subscribe(std::move(t));
    }
}

asio::awaitable<void> DockSim::start(bool telemetry) {
    co_await activate();
    co_await update_topo(true);
    co_await publish_state();
    const auto& cfg = ctx_.cfg();
    if (cfg.organization.bind_on_start) ctx_.spawn(bind_to_organization(), "bind");
    ctx_.spawn(announce_storage(), "storage-announce");
    if (cfg.maintenance.sync_flight_areas_on_start) areas_.start_sync();
    // Without telemetry the hms list goes out once, so that start() can settle.
    if (!cfg.maintenance.hms.empty() && (telemetry || cfg.maintenance.hms_interval <= 0.0))
        ctx_.spawn(hms_.run(), "hms");
    if (telemetry) {
        ctx_.spawn(dock_osd_loop(), "dock-osd");
        ctx_.spawn(drone_osd_loop(), "drone-osd");
        ctx_.spawn(state_loop(), "state");
        if (ctx_.cfg().telemetry.topo_interval > 0) ctx_.spawn(topo_loop(), "topo");
    }
}

asio::awaitable<void> DockSim::stop() {
    co_await live_.close();
    co_await drc_.exit();
    co_await manual_.shutdown();
    co_await mission_.shutdown();
    co_await ctx_.cancel_all();
    co_await update_topo(false);
}

std::optional<code::Result> DockSim::preflight() const {
    const auto& dock = ctx_.dock;
    if (dock.emergency_stop_state != 0) return code::Result::emergency_stop;
    if (const auto weather = device::weather_block(dock.weather, ctx_.cfg().weather))
        return weather;
    if (ctx_.drone.battery_percent() < device::return_home_power_percent)
        return code::Result::battery_low;
    if (dock.cover_fault) return code::Result::cover_open_failed;
    return std::nullopt;
}

// The dock sends the config request every 10 s until a reply comes. A missing
// key or a bad signature makes it disconnect from the cloud.
asio::awaitable<void> DockSim::activate() {
    while (true) {
        Json data;
        try {
            data = co_await ctx_.request(
                "config", {{"config_type", "json"}, {"config_scope", "product"}}, config_retry);
        } catch (const transport::RequestTimeout&) {
            logger()->warn("config: no requests_reply, sending again");
            continue;
        }
        const auto text = [&](const char* key) -> std::optional<std::string> {
            const auto it = data.find(key);
            if (it == data.end() || !it->is_string()) return std::nullopt;
            return it->get<std::string>();
        };
        const auto app_id = text("app_id");
        const auto app_key = text("app_key");
        const auto app_license = text("app_license");
        if (!app_id || !app_key || !app_license) {
            std::string keys;
            for (const auto& [key, _] : data.items()) keys += (keys.empty() ? "" : ", ") + key;
            throw LicenseError(std::format("can't get app info from request: [{}]", keys));
        }
        if (!verify_license_(*app_id, *app_key, *app_license)) {
            throw LicenseError(std::format("license check failed for app id {}", *app_id));
        }
        logger()->info("license check success, app id {}, ntp server {}", *app_id,
                       data.value("ntp_server_host", Json()).dump());
        co_return;
    }
}

asio::awaitable<void> DockSim::update_topo(bool online) {
    const auto& device = ctx_.cfg().device;
    const auto& dock = catalog::device(device.dock_model);
    const auto& drone = catalog::device(device.drone_model);
    Json sub_devices = Json::array();
    if (online) {
        sub_devices.push_back({
            {"sn", device.drone_sn},
            {"domain", drone.domain},
            {"type", drone.type},
            {"sub_type", drone.sub_type},
            {"index", device.channel_index},
            {"device_secret", device.device_secret},
            {"nonce", envelope::new_id()},
            {"thing_version", device.thing_version},
        });
    }
    logger()->info("update_topo sub_devices={}", sub_devices.size());
    Json data = {
        {"domain", dock.domain},
        {"type", dock.type},
        {"sub_type", dock.sub_type},
        {"device_secret", device.device_secret},
        {"nonce", envelope::new_id()},
        {"thing_version", device.thing_version},
        {"sub_devices", std::move(sub_devices)},
    };
    co_await ctx_.transport().publish(topic::status(device.dock_sn),
                                      envelope::make(std::move(data), "update_topo"));
}

// Every state message needs the gateway field. The cloud looks the device SDK
// up by it and drops the message if it is absent.
asio::awaitable<void> DockSim::publish_state() {
    const auto& sn = ctx_.dock_sn();
    for (auto& group : payloads::dock_state(ctx_.dock)) {
        co_await ctx_.transport().publish(topic::state(sn),
                                          envelope::make(std::move(group), {}, {{"gateway", sn}}));
    }
    for (auto& group : payloads::drone_state(ctx_.drone)) {
        co_await ctx_.transport().publish(topic::state(ctx_.drone_sn()),
                                          envelope::make(std::move(group), {}, {{"gateway", sn}}));
    }
}

// The binding handshake: bind_status, then organization get, then bind.
asio::awaitable<void> DockSim::bind_to_organization() {
    const auto& org = ctx_.cfg().organization;
    const auto& device = ctx_.cfg().device;
    Json status;
    try {
        status = co_await ctx_.request(
            "airport_bind_status",
            {{"devices", Json::array({{{"sn", device.drone_sn}}, {{"sn", device.dock_sn}}})}});
    } catch (const transport::RequestTimeout& e) {
        logger()->error("airport_bind_status: {}", e.what());
        co_return;
    }
    std::vector<std::string> unbound;
    for (const auto& sn : {device.dock_sn, device.drone_sn}) {
        bool bound = false;
        const auto& list = output_of(status).value("bind_status", Json::array());
        for (const auto& entry : list.is_array() ? list : Json::array()) {
            if (entry.value("sn", "") == sn && entry.value("is_device_bind_organization", false))
                bound = true;
        }
        if (!bound) unbound.push_back(sn);
    }
    if (unbound.empty()) {
        logger()->info("both devices are already bound to the organization");
        co_return;
    }
    if (!org.organization_id || !org.binding_code || org.organization_id->empty() ||
        org.binding_code->empty()) {
        logger()->warn("{} device(s) not bound, and no organization_id or binding_code is set",
                       unbound.size());
        co_return;
    }
    Json found;
    try {
        found = co_await ctx_.request("airport_organization_get",
                                      {{"device_binding_code", *org.binding_code},
                                       {"organization_id", *org.organization_id}});
    } catch (const transport::RequestTimeout& e) {
        logger()->error("airport_organization_get: {}", e.what());
        co_return;
    }
    logger()->info("organization: {}", output_of(found).value("organization_name", Json()).dump());
    Json bind_devices = Json::array();
    const std::array<std::tuple<std::string, std::string, std::string>, 2> devices{{
        {device.dock_sn, device.dock_model, org.dock_callsign},
        {device.drone_sn, device.drone_model, org.drone_callsign},
    }};
    for (const auto& [sn, model, callsign] : devices) {
        if (!std::ranges::contains(unbound, sn)) continue;
        bind_devices.push_back({
            {"sn", sn},
            {"device_binding_code", *org.binding_code},
            {"organization_id", *org.organization_id},
            {"device_callsign", callsign},
            {"device_model_key", catalog::model_key(model)},
        });
    }
    Json outcome;
    try {
        outcome = co_await ctx_.request("airport_organization_bind",
                                        {{"bind_devices", std::move(bind_devices)}});
    } catch (const transport::RequestTimeout& e) {
        logger()->error("airport_organization_bind: {}", e.what());
        co_return;
    }
    logger()->info("bind result: {}", outcome.dump());
    co_await update_topo(true);
}

// Ask for the media storage location once after the dock goes online. The
// web UI warns at launch if the cloud never saw this request after the login
// of this session.
asio::awaitable<void> DockSim::announce_storage() {
    co_await ctx_.clock().sleep(storage_announce_delay_s);
    Json data;
    try {
        data = co_await ctx_.request("storage_config_get", {{"module", 0}});
    } catch (const transport::RequestTimeout& e) {
        logger()->error("storage_config_get: {}", e.what());
        co_return;
    }
    logger()->info("storage announced: result={} bucket={}", data.value("result", Json()).dump(),
                   output_of(data).value("bucket", Json()).dump());
}

asio::awaitable<void> DockSim::osd_tick() {
    co_await dock_osd();
    co_await drone_osd();
}

asio::awaitable<void> DockSim::dock_osd() {
    device::charge(ctx_.dock, ctx_.drone, ctx_.cfg().telemetry.dock_osd_interval);
    const auto& sn = ctx_.dock_sn();
    for (auto& message : payloads::dock_osd(ctx_.dock)) {
        co_await ctx_.transport().publish(
            topic::osd(sn), envelope::make(std::move(message), {}, {{"gateway", sn}}), 0);
    }
}

asio::awaitable<void> DockSim::drone_osd() {
    camera_.refresh();
    co_await ctx_.transport().publish(
        topic::osd(ctx_.drone_sn()),
        envelope::make(payloads::drone_osd(ctx_.drone), {}, {{"gateway", ctx_.dock_sn()}}), 0);
}

asio::awaitable<void> DockSim::dock_osd_loop() {
    while (true) {
        if (ctx_.dock.online) co_await dock_osd();
        co_await ctx_.clock().sleep(ctx_.cfg().telemetry.dock_osd_interval);
    }
}

asio::awaitable<void> DockSim::drone_osd_loop() {
    while (true) {
        if (ctx_.dock.online) co_await drone_osd();
        co_await ctx_.clock().sleep(ctx_.cfg().telemetry.drone_osd_interval);
    }
}

asio::awaitable<void> DockSim::state_loop() {
    while (true) {
        co_await ctx_.clock().sleep(ctx_.cfg().telemetry.state_interval);
        if (ctx_.dock.online) co_await publish_state();
    }
}

// Republish the topology. It lets the cloud find the dock after a restart.
asio::awaitable<void> DockSim::topo_loop() {
    while (true) {
        co_await ctx_.clock().sleep(ctx_.cfg().telemetry.topo_interval);
        if (ctx_.dock.online) co_await update_topo(true);
    }
}

asio::awaitable<void> DockSim::on_services(Json message) {
    const auto method =
        message.value("method", Json("")).is_string() ? message.value("method", "") : std::string{};
    const Json data =
        message.contains("data") && !message["data"].is_null() ? message["data"] : Json::object();
    // A rebooting dock has no link. The cloud gets no reply and times out.
    if (!ctx_.dock.online) {
        logger()->warn("service {} while offline: dropped", method);
        co_return;
    }
    if (!routing::routed(ctx_.cfg().device.dock_model, routing::Channel::services, method)) {
        logger()->warn("service {}: the {} gateway does not route it, no reply", method,
                       ctx_.cfg().device.dock_model);
        co_return;
    }
    const auto bid = message.find("bid");
    ctx_.request_bid = bid != message.end() && bid->is_string() ? bid->get<std::string>() : "";
    auto answer = co_await registry_.dispatch(method, data);
    co_await ctx_.transport().publish(topic::services_reply(ctx_.dock_sn()),
                                      envelope::reply(message, std::move(answer), ctx_.dock_sn()));
}

asio::awaitable<void> DockSim::on_property_set(std::string topic, Json message) {
    // thing/product/{sn}/property/set
    const auto first = topic.find('/', topic.find('/') + 1);
    const auto sn = topic.substr(first + 1, topic.find('/', first + 1) - first - 1);
    const Json data = message.value("data", Json::object());
    logger()->info("property/set on {}: {}", sn, data.dump());
    if (!data.is_object()) co_return;
    for (const auto& [key, value] : data.items()) {
        co_await ctx_.transport().publish(topic::property_set_reply(sn),
                                          {
                                              {"tid", message.value("tid", Json())},
                                              {"bid", message.value("bid", Json())},
                                              {"timestamp", envelope::now_ms()},
                                              {"data", {{"result", 0}, {"output", {{key, value}}}}},
                                          });
    }
}

asio::awaitable<void> DockSim::reboot_offline() {
    co_await drc_.exit();
    co_await live_.close();
    co_await update_topo(false);
}

asio::awaitable<void> DockSim::reboot_online() {
    co_await update_topo(true);
    co_await publish_state();
    if (ctx_.cfg().maintenance.sync_flight_areas_on_start) areas_.start_sync();
}

}  // namespace docksim::sim
