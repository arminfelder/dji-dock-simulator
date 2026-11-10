// Maintenance services: remote debug, device reboot, firmware update, device
// logs, custom flight areas, offline map, remote unlocking, HMS and the
// conditional and fallback paths of the wayline task. Driven on the virtual
// clock of the golden harness.

#include "device/catalog.hpp"
#include "flight_support.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <format>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace docksim;
using namespace docksim::golden;
using namespace docksim::flight_test;
using Catch::Approx;
using code::DockMode;

namespace {

Config maintenance_config() {
    Config cfg = manual_config();
    cfg.maintenance.job_duration = 2.0;
    cfg.maintenance.reboot_duration = 10.0;
    cfg.maintenance.ota_duration = 10.0;
    cfg.maintenance.log_step_duration = 0.5;
    return cfg;
}

// The full payload of every main-topic message with this method, in order.
std::vector<Json> payloads(Driver& d, std::string_view suffix, std::string_view method) {
    std::vector<Json> out;
    for (const auto& p : d.main_transport().published) {
        if (p.topic.ends_with(suffix) && p.payload.value("method", "") == method)
            out.push_back(p.payload);
    }
    return out;
}

Json last_reply(Driver& d, std::string_view method) {
    const auto replies = messages(d, "/services_reply", method);
    REQUIRE_FALSE(replies.empty());
    return replies.back();
}

// Index in the wire of the first message that matches.
std::ptrdiff_t position(Driver& d, std::string_view suffix, std::string_view method) {
    const auto& wire = d.main_transport().published;
    const auto it = std::ranges::find_if(wire, [&](const Published& p) {
        return p.topic.ends_with(suffix) && p.payload.value("method", "") == method;
    });
    return it == wire.end() ? -1 : it - wire.begin();
}

Json takeoff_to(Driver& d, double latitude, bool safety_check) {
    Json data = takeoff_data(d);
    data["target_latitude"] = latitude;
    data["flight_safety_advance_check"] = safety_check;
    return data;
}

// Inside the no-fly circle of areas_geojson().
constexpr double nfz_latitude = 47.272;

void sync_areas(Driver& d) {
    d.service("flight_areas_update", Json::object());
    d.advance(1.0);
    REQUIRE(d.sim().flight_areas().areas().size() == 1);
}

// The put runs on a worker thread, and the virtual clock does not wait for it.
void finish_logs(Driver& d) {
    for (int i = 0; i < 2000 && d.sim().logs().uploading(); ++i) {
        d.advance(0.25);
        std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    REQUIRE_FALSE(d.sim().logs().uploading());
}

Json log_start(std::vector<std::pair<std::string, std::string>> files) {
    Json list = Json::array();
    for (const auto& [module, key] : files)
        list.push_back({{"module", module},
                        {"object_key", key},
                        {"list", Json::array({{{"boot_index", 3}}, {{"boot_index", 2}}})}});
    return {
        {"bucket", "logs"},
        {"region", "us-east-1"},
        {"provider", "minio"},
        {"endpoint", "https://storage.example.invalid"},
        {"credentials",
         {{"access_key_id", "AK"},
          {"access_key_secret", "SK"},
          {"security_token", ""},
          {"expire", 3600}}},
        {"params", {{"files", std::move(list)}}},
    };
}

}  // namespace

TEST_CASE("remote debug: jobs need debug mode and report progress under the request bid",
          "[maintenance]") {
    Driver d(maintenance_config());
    d.connect();
    auto& dock = d.sim().ctx().dock;

    d.service("cover_open", Json::object());
    CHECK(reply_result(d, "cover_open") == 514153);
    d.service("air_conditioner_mode_switch", {{"action", 1}});
    CHECK(reply_result(d, "air_conditioner_mode_switch") == 514153);
    // Commands of the remote-debug panel work without debug mode.
    d.service("battery_store_mode_switch", {{"action", 2}});
    CHECK(last_reply(d, "battery_store_mode_switch")["output"]["status"] == "ok");
    CHECK(dock.battery_store_mode == 2);
    d.service("alarm_state_switch", {{"action", 5}});
    CHECK(reply_result(d, "alarm_state_switch") == 514171);

    d.service("debug_mode_open", Json::object());
    CHECK(reply_result(d, "debug_mode_open") == 0);
    CHECK(dock.mode_code == DockMode::remote_debug);
    d.service("air_conditioner_mode_switch", {{"action", 1}});
    CHECK(dock.air_conditioner_state == 1);

    d.clear();
    d.service("cover_open", Json::object());
    CHECK(last_reply(d, "cover_open") ==
          Json{{"result", 0}, {"output", {{"status", "in_progress"}}}});
    d.service("cover_close", Json::object());
    CHECK(reply_result(d, "cover_close") == 514116);
    d.advance(0.5);
    CHECK(dock.cover_state == 0);
    d.advance(2.0);
    CHECK(dock.cover_state == 1);

    // The steps of the cover_open job of the real Dock 2, then ok on the last one.
    const std::vector<std::string> keys{"get_bid",           "check_work_mode", "check_task_state",
                                        "check_scram_state", "open_cover",      "open_cover"};
    const auto progress = payloads(d, "/events", "cover_open");
    REQUIRE(progress.size() == keys.size());
    for (std::size_t i = 0; i < keys.size(); ++i) {
        const auto& e = progress[i];
        INFO(i);
        CHECK(e["bid"] == "b");
        CHECK(e["need_reply"] == 1);
        const auto& step = e["data"]["output"]["progress"];
        CHECK(step["step_key"] == keys[i]);
        CHECK(step["current_step"] == std::min<int>(static_cast<int>(i) + 1, 5));
        CHECK(step["total_steps"] == 5);
        CHECK(step["step_result"] == 0);
        CHECK(step["percent"] == static_cast<int>(i) * 20);
    }
    CHECK(progress[0]["data"]["output"]["status"] == "in_progress");
    CHECK(progress.back()["data"]["output"]["status"] == "ok");
    CHECK(position(d, "/services_reply", "cover_open") < position(d, "/events", "cover_open"));

    d.service("charge_close", Json::object());
    d.advance(2.5);
    CHECK_FALSE(dock.charging_enabled);
    d.service("drone_close", Json::object());
    d.advance(2.5);
    CHECK_FALSE(dock.drone_powered);

    // A flight task and a takeoff are refused in debug mode.
    d.service("takeoff_to_point", takeoff_data(d));
    CHECK(reply_result(d, "takeoff_to_point") == 514146);
    d.service("debug_mode_close", Json::object());
    CHECK(dock.mode_code == DockMode::idle);
}

TEST_CASE("remote debug: debug mode is refused while the aircraft works", "[maintenance]") {
    Driver d(maintenance_config());
    d.connect();
    d.service("takeoff_to_point", takeoff_data(d));
    d.advance(1.0);
    d.service("debug_mode_open", Json::object());
    CHECK(reply_result(d, "debug_mode_open") == 514148);
    d.service("ota_create",
              {{"devices", Json::array({{{"sn", dock_sn}, {"product_version", "15.01.0001"}}})}});
    CHECK(reply_result(d, "ota_create") == 312015);
}

TEST_CASE("device_reboot: progress, a silent offline period, then online again", "[maintenance]") {
    Driver d(maintenance_config());
    d.connect();
    auto& dock = d.sim().ctx().dock;
    d.service("device_reboot", Json::object());
    CHECK(reply_result(d, "device_reboot") == 514153);
    d.service("debug_mode_open", Json::object());
    d.service("drc_mode_enter", drc_enter());
    REQUIRE(dock.drc_state == code::DrcState::connected);

    d.clear();
    d.service("device_reboot", Json::object());
    CHECK(last_reply(d, "device_reboot")["output"]["status"] == "in_progress");
    d.advance(2.5);
    CHECK_FALSE(dock.online);
    CHECK(d.sim().maintenance().rebooting());
    CHECK(dock.drc_state == code::DrcState::disconnected);
    const auto progress = events(d, "device_reboot");
    REQUIRE(progress.size() == 6);
    CHECK(progress[1]["output"]["progress"]["step_key"] == "upgrading_prevent_reboot");
    CHECK(progress.back()["output"]["status"] == "ok");
    CHECK(progress.back()["output"]["progress"]["step_key"] == "write_reboot_param_file");
    CHECK(progress.back()["output"]["progress"]["total_steps"] == 5);
    // live_.close() runs on a worker thread, and the virtual clock does not wait for it.
    for (int i = 0; i < 2000 && messages(d, "/status", "update_topo").empty(); ++i) {
        d.advance(0.001);
        std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    const auto topo = messages(d, "/status", "update_topo");
    REQUIRE(topo.size() == 1);
    CHECK(topo[0]["sub_devices"].empty());
    CHECK(position(d, "/events", "device_reboot") < position(d, "/status", "update_topo"));

    d.clear();
    d.service("cover_open", Json::object());
    d.osd_tick();
    CHECK(messages(d, "/services_reply", "cover_open").empty());

    d.advance(10.0);
    CHECK(dock.online);
    CHECK(dock.mode_code == DockMode::idle);
    const auto back = messages(d, "/status", "update_topo");
    REQUIRE(back.size() == 1);
    CHECK(back[0]["sub_devices"].size() == 1);
    bool state = false;
    for (const auto& p : d.main_transport().published) state = state || p.topic.ends_with("/state");
    CHECK(state);
    d.service("cover_open", Json::object());
    CHECK(reply_result(d, "cover_open") == 514153);
}

TEST_CASE("ota_create: progress events, then the new firmware version", "[maintenance]") {
    Driver d(maintenance_config());
    d.connect();
    auto& ctx = d.sim().ctx();
    const Json request = {{"devices", Json::array({{
                                          {"sn", drone_sn},
                                          {"product_version", "15.01.0001"},
                                          {"file_url", "https://example.invalid/fw.zip"},
                                          {"md5", "0123"},
                                          {"file_size", 10},
                                          {"file_name", "fw.zip"},
                                          {"firmware_upgrade_type", 2},
                                      }})}};
    d.service("ota_create",
              {{"devices", Json::array({{{"sn", "OTHER"}, {"product_version", "1"}}})}});
    CHECK(reply_result(d, "ota_create") == 514171);

    d.clear();
    d.service("ota_create", request);
    CHECK(last_reply(d, "ota_create")["output"]["status"] == "in_progress");
    CHECK(ctx.dock.mode_code == DockMode::upgrading);
    CHECK(ctx.drone.firmware_upgrading);
    d.service("ota_create", request);
    CHECK(reply_result(d, "ota_create") == 312014);
    d.service("debug_mode_open", Json::object());
    CHECK(reply_result(d, "debug_mode_open") == 514147);

    d.advance(11.0);
    CHECK(ctx.drone.firmware_version == "15.01.0001");
    CHECK(ctx.dock.firmware_version == default_firmware);
    CHECK_FALSE(ctx.drone.firmware_upgrading);
    CHECK(ctx.dock.mode_code == DockMode::idle);

    const auto progress = payloads(d, "/events", "ota_progress");
    REQUIRE(progress.size() == static_cast<std::size_t>(services::ota_progress_steps + 1));
    const auto& first = progress.front()["data"]["output"];
    CHECK(first["status"] == "in_progress");
    // The real dock sends no step_key.
    CHECK_FALSE(first["progress"].contains("step_key"));
    CHECK(first["progress"]["current_step"] == 1);
    const auto& last = progress.back()["data"]["output"];
    CHECK(last["status"] == "ok");
    CHECK(last["progress"]["percent"] == 100);
    CHECK(last["progress"]["current_step"] == 2);
    CHECK(progress.back()["bid"] == "b");

    // The new version is in the state before the end event.
    const auto& wire = d.main_transport().published;
    std::ptrdiff_t state_at = -1;
    for (std::size_t i = 0; i < wire.size(); ++i) {
        const auto& p = wire[i];
        if (p.topic == std::format("thing/product/{}/state", drone_sn) &&
            p.payload["data"].value("firmware_version", "") == "15.01.0001" &&
            p.payload["data"]["firmware_upgrade_status"] == false)
            state_at = static_cast<std::ptrdiff_t>(i);
    }
    REQUIRE(state_at >= 0);
    std::ptrdiff_t ok_at = -1;
    for (std::size_t i = 0; i < wire.size(); ++i) {
        if (wire[i].payload.value("method", "") == "ota_progress" &&
            wire[i].payload["data"]["output"]["status"] == "ok")
            ok_at = static_cast<std::ptrdiff_t>(i);
    }
    CHECK(state_at < ok_at);
}

TEST_CASE("fileupload: list, upload with progress, cancel", "[maintenance]") {
    Driver d(maintenance_config());
    d.connect();

    d.service("fileupload_list", {{"module_list", Json::array({"0", "3"})}});
    const auto files = last_reply(d, "fileupload_list")["output"]["files"];
    REQUIRE(files.size() == 2);
    CHECK(files[0]["module"] == "0");
    CHECK(files[0]["device_sn"] == drone_sn);
    CHECK(files[1]["module"] == "3");
    CHECK(files[1]["device_sn"] == dock_sn);
    REQUIRE(files[1]["list"].size() == static_cast<std::size_t>(services::log_boot_count));
    for (const auto& entry : files[1]["list"]) {
        CHECK(entry["start_time"].get<std::int64_t>() < entry["end_time"].get<std::int64_t>());
        CHECK(entry["size"].get<std::int64_t>() > 0);
    }

    SECTION("upload") {
        d.clear();
        d.service("fileupload_start",
                  log_start({{"3", "org-1/logs_dock.zip"}, {"0", "org-1/logs_drone.zip"}}));
        CHECK(reply_result(d, "fileupload_start") == 0);
        d.service("fileupload_start", log_start({{"3", "org-1/again.zip"}}));
        CHECK(reply_result(d, "fileupload_start") == 325009);
        finish_logs(d);
        REQUIRE(d.puts.size() == 2);
        CHECK(d.puts[0].first == "org-1/logs_dock.zip");
        CHECK(d.puts[1].first == "org-1/logs_drone.zip");

        const auto progress = payloads(d, "/events", "fileupload_progress");
        REQUIRE(progress.size() >= 2);
        for (const auto& e : progress) {
            CHECK(e["bid"] == "b");
            CHECK(e["need_reply"] == 0);
        }
        CHECK(progress.front()["data"]["output"]["status"] == "in_progress");
        const auto& end = progress.back()["data"]["output"];
        CHECK(end["status"] == "ok");
        REQUIRE(end["ext"]["files"].size() == 2);
        for (const auto& f : end["ext"]["files"]) {
            CHECK(f["progress"]["status"] == "ok");
            CHECK(f["progress"]["progress"] == 100);
            CHECK(f["progress"]["result"] == 0);
            CHECK(f["fingerprint"].get<std::string>().size() == 32);
            CHECK(f["size"].get<std::size_t>() > 0);
        }
        CHECK(end["ext"]["files"][0]["key"] == "org-1/logs_dock.zip");
        CHECK(end["ext"]["files"][0]["device_sn"] == dock_sn);
        CHECK(d.puts[0].second == end["ext"]["files"][0]["size"].get<std::size_t>());
    }

    SECTION("cancel") {
        d.clear();
        d.service("fileupload_start", log_start({{"3", "org-1/logs_dock.zip"}}));
        d.service("fileupload_update", {{"status", "cancel"}, {"module_list", Json::array({"3"})}});
        CHECK(reply_result(d, "fileupload_update") == 0);
        finish_logs(d);
        CHECK(d.puts.empty());
        const auto progress = events(d, "fileupload_progress");
        REQUIRE_FALSE(progress.empty());
        CHECK(progress.back()["output"]["status"] == "canceled");
        CHECK(progress.back()["output"]["ext"]["files"][0]["progress"]["status"] == "canceled");
    }
}

TEST_CASE("flight areas: sync, checksum failure, no-fly refusal, drone location", "[maintenance]") {
    Driver d(maintenance_config());
    d.connect();
    d.clear();

    SECTION("sync stores the areas") {
        sync_areas(d);
        CHECK(statuses(events(d, "flight_areas_sync_progress")) ==
              std::vector<std::string>{"synchronizing", "synchronized"});
        CHECK(d.fetched == std::vector<std::string>{areas_url});
        const auto& area = d.sim().flight_areas().areas()[0];
        CHECK(area.id == "nfz-north");
        CHECK(area.kind == services::areas::Kind::nfz);
        CHECK(area.radius == Approx(200.0));
        CHECK(d.sim().flight_areas().forbidden(nfz_latitude, home(d).longitude));
        CHECK_FALSE(d.sim().flight_areas().forbidden(home(d).latitude, home(d).longitude));
    }

    SECTION("a checksum mismatch fails the sync") {
        d.cloud().replies["flight_areas_get"]["output"]["files"][0]["checksum"] =
            std::string(64, '0');
        d.service("flight_areas_update", Json::object());
        d.advance(1.0);
        const auto progress = events(d, "flight_areas_sync_progress");
        REQUIRE(progress.size() == 2);
        CHECK(progress.back()["status"] == "fail");
        CHECK(progress.back()["reason"] == 12);
        CHECK(d.sim().flight_areas().areas().empty());
    }

    SECTION("a missing file fails the sync with download failed") {
        d.files.clear();
        d.service("flight_areas_update", Json::object());
        d.advance(1.0);
        CHECK(events(d, "flight_areas_sync_progress").back()["reason"] == 3);
    }

    SECTION("takeoff_to_point into a no-fly zone with the safety check") {
        sync_areas(d);
        d.service("takeoff_to_point", takeoff_to(d, nfz_latitude, true));
        CHECK(reply_result(d, "takeoff_to_point") == 336513);
        CHECK_FALSE(d.sim().manual().flying());
        d.service("takeoff_to_point", takeoff_to(d, nfz_latitude, false));
        CHECK(reply_result(d, "takeoff_to_point") == 0);
    }

    SECTION("fly_to_point into a no-fly zone after a checked takeoff") {
        sync_areas(d);
        d.service("takeoff_to_point",
                  takeoff_to(d, home(d).latitude + 60.0 / metres_per_degree, true));
        d.advance(30.0);
        REQUIRE(d.sim().manual().phase() == flight::Phase::hover);
        Json target = point_north(d, 0, 40);
        target["latitude"] = nfz_latitude;
        d.service("fly_to_point", {{"fly_to_id", "f"}, {"points", Json::array({target})}});
        CHECK(reply_result(d, "fly_to_point") == 336513);
    }

    SECTION("flight_areas_drone_location while airborne, joystick_invalid_notify on entry") {
        sync_areas(d);
        d.service("takeoff_to_point", takeoff_data(d));
        d.advance(30.0);
        REQUIRE(d.sim().manual().phase() == flight::Phase::hover);
        const auto locations = payloads(d, "/events", "flight_areas_drone_location");
        REQUIRE(locations.size() >= 4);
        CHECK(locations.size() <= 8);
        const auto& last = locations.back();
        CHECK(last["need_reply"] == 0);
        const auto& entry = last["data"]["drone_locations"][0];
        CHECK(entry["area_id"] == "nfz-north");
        CHECK(entry["is_in_area"] == false);
        CHECK(entry["area_distance"].get<double>() == Approx(1000.8 - 60.0 - 200.0).margin(15.0));

        d.service("drc_mode_enter", drc_enter());
        d.clear();
        d.sim().ctx().drone.latitude = nfz_latitude;
        d.advance(0.5);
        const auto notes = events(d, "joystick_invalid_notify");
        REQUIRE(notes.size() == 1);
        CHECK(notes[0]["reason"] == 3);
    }
}

TEST_CASE("offline map: get, download, sync progress", "[maintenance]") {
    Driver d(maintenance_config());
    d.connect();
    const std::string url = "https://files.example.invalid/map.bin";
    Json reply = file_list_reply("map.bin", url, "MAPDATA");
    reply["output"]["offline_map_enable"] = true;
    d.cloud().replies["offline_map_get"] = reply;
    d.clear();

    SECTION("success") {
        d.files[url] = "MAPDATA";
        d.service("offline_map_update", Json::object());
        CHECK(reply_result(d, "offline_map_update") == 0);
        d.advance(1.0);
        const auto progress = events(d, "offline_map_sync_progress");
        CHECK(statuses(progress) == std::vector<std::string>{"synchronizing", "synchronized"});
        CHECK(progress.back()["file"]["name"] == "map.bin");
        CHECK(d.sim().offline_map().enabled());
        CHECK(d.sim().offline_map().files() == std::vector<std::string>{"map.bin"});
        CHECK(messages(d, "/requests", "offline_map_get").size() == 1);
    }

    SECTION("download failure") {
        d.service("offline_map_update", Json::object());
        d.advance(1.0);
        const auto progress = events(d, "offline_map_sync_progress");
        CHECK(progress.back()["status"] == "fail");
        CHECK(progress.back()["reason"] == 3);
        CHECK(d.sim().offline_map().files().empty());
    }
}

TEST_CASE("remote unlocking: an empty, consistent license list", "[maintenance]") {
    Driver d(maintenance_config());
    d.connect();
    d.service("unlock_license_list", {{"device_model_domain", 0}});
    CHECK(last_reply(d, "unlock_license_list") ==
          Json{{"result", 0},
               {"output",
                {{"consistence", true}, {"device_model_domain", 0}, {"licenses", Json::array()}}}});
    d.service("unlock_license_switch", {{"license_id", 7}, {"enable", true}});
    CHECK(last_reply(d, "unlock_license_switch")["output"]["license_id"] == 7);
    d.service("unlock_license_update", {{"file", {{"url", "https://example.invalid/l"}}}});
    CHECK(reply_result(d, "unlock_license_update") == 0);
}

TEST_CASE("hms: the configured alarms, once or at the interval", "[maintenance]") {
    Config cfg = maintenance_config();
    cfg.maintenance.hms = {HmsAlarm{.code = "0x16100083",
                                    .level = 2,
                                    .module = 3,
                                    .device = "drone",
                                    .in_the_sky = false,
                                    .imminent = true,
                                    .component_index = 1,
                                    .sensor_index = 0}};

    SECTION("once") {
        cfg.maintenance.hms_interval = 0.0;
        Driver d(cfg);
        d.connect();
        const auto hms = payloads(d, "/events", "hms");
        REQUIRE(hms.size() == 1);
        CHECK(hms[0]["need_reply"] == 0);
        CHECK(hms[0]["data"] ==
              Json{{"list", Json::array({{
                                {"code", "0x16100083"},
                                {"level", 2},
                                {"module", 3},
                                {"in_the_sky", 0},
                                {"imminent", 1},
                                {"device_type", catalog::model_key("m3d")},
                                {"args", {{"component_index", 1}, {"sensor_index", 0}}},
                            }})}});
    }

    SECTION("interval") {
        cfg.maintenance.hms_interval = 30.0;
        Driver d(cfg);
        d.connect();
        CHECK(events(d, "hms").empty());
        d.sim().ctx().spawn(d.sim().hms().run(), "hms");
        d.advance(65.0);
        CHECK(events(d, "hms").size() == 3);
    }

    SECTION("empty list sends nothing") {
        cfg.maintenance.hms.clear();
        cfg.maintenance.hms_interval = 0.0;
        Driver d(cfg);
        d.connect();
        CHECK(events(d, "hms").empty());
    }
}

TEST_CASE("flighttask: a conditional task reports flighttask_ready after the reply",
          "[maintenance]") {
    Driver d(maintenance_config());
    d.connect();
    d.clear();
    d.service("flighttask_prepare", {{"flight_id", "cond-1"},
                                     {"task_type", wayline::conditional_task_type},
                                     {"file", {{"url", "https://example.invalid/m.kmz"}}}});
    CHECK(reply_result(d, "flighttask_prepare") == 0);
    d.advance(0.1);
    const auto ready = payloads(d, "/events", "flighttask_ready");
    REQUIRE(ready.size() == 1);
    CHECK(ready[0]["data"] == Json{{"flight_ids", Json::array({"cond-1"})}});
    CHECK(position(d, "/services_reply", "flighttask_prepare") <
          position(d, "/events", "flighttask_ready"));

    // Dock 2 firmware does not route flighttask_create: no reply.
    d.clear();
    const Json now = {{"flight_id", "now-1"},
                      {"task_type", 0},
                      {"file", {{"url", "https://example.invalid/m.kmz"}}}};
    d.service("flighttask_create", now);
    d.advance(0.1);
    CHECK(messages(d, "/services_reply", "flighttask_create").empty());
    CHECK(d.sim().mission().task()->flight_id == "cond-1");

    auto cfg = maintenance_config();
    cfg.device.dock_model = "dock1";
    Driver d1(cfg);
    d1.connect();
    d1.clear();
    d1.service("flighttask_create", now);
    CHECK(reply_result(d1, "flighttask_create") == 0);
    d1.advance(0.1);
    CHECK(events(d1, "flighttask_ready").empty());
    CHECK(d1.sim().mission().task()->flight_id == "now-1");
}

TEST_CASE("flighttask: a failed KMZ download asks flighttask_resource_get once", "[maintenance]") {
    Driver d(maintenance_config());
    d.connect();
    d.clear();
    const Json prepare = {{"flight_id", "f-1"},
                          {"task_type", 0},
                          {"file", {{"url", "https://example.invalid/broken.kmz"}}}};

    SECTION("a fresh URL") {
        d.cloud().replies["flighttask_resource_get"] = {
            {"result", 0},
            {"output",
             {{"file", {{"url", "https://example.invalid/fresh.kmz"}, {"fingerprint", "f"}}}}}};
        d.service("flighttask_prepare", prepare);
        CHECK(reply_result(d, "flighttask_prepare") == 0);
        CHECK(d.downloads == std::vector<std::string>{"https://example.invalid/broken.kmz",
                                                      "https://example.invalid/fresh.kmz"});
        const auto asked = messages(d, "/requests", "flighttask_resource_get");
        REQUIRE(asked.size() == 1);
        CHECK(asked[0] == Json{{"flight_id", "f-1"}});
        CHECK(d.sim().mission().task()->file_url == "https://example.invalid/fresh.kmz");
    }

    SECTION("no new URL") {
        d.cloud().replies["flighttask_resource_get"] = {{"result", 0},
                                                        {"output", {{"file", {{"url", ""}}}}}};
        d.service("flighttask_prepare", prepare);
        CHECK(reply_result(d, "flighttask_prepare") == 514002);
        CHECK(d.downloads.size() == 1);
    }

    SECTION("the second URL fails too") {
        d.cloud().replies["flighttask_resource_get"] = {
            {"result", 0},
            {"output", {{"file", {{"url", "https://example.invalid/2/broken.kmz"}}}}}};
        d.service("flighttask_prepare", prepare);
        CHECK(reply_result(d, "flighttask_prepare") == 514002);
        CHECK(d.downloads.size() == 2);
    }
}

namespace {

// Every services method of the cloud-sdk *MethodEnum classes. The
// drc/down methods (RemoteControlMethodEnum, drone_control, heart_beat) are not
// services and are left out.
const std::vector<std::string> cloud_services_methods{
    // ControlMethodEnum
    "flight_authority_grab",
    "payload_authority_grab",
    "drc_mode_enter",
    "drc_mode_exit",
    "fly_to_point",
    "fly_to_point_stop",
    "fly_to_point_update",
    "takeoff_to_point",
    "camera_mode_switch",
    "camera_photo_take",
    "camera_screen_drag",
    "camera_photo_stop",
    "camera_recording_start",
    "camera_recording_stop",
    "camera_aim",
    "camera_focal_length_set",
    "gimbal_reset",
    "camera_look_at",
    "camera_screen_split",
    "photo_storage_set",
    "video_storage_set",
    "camera_exposure_set",
    "camera_exposure_mode_set",
    "camera_focus_mode_set",
    "camera_focus_value_set",
    "ir_metering_mode_set",
    "ir_metering_point_set",
    "ir_metering_area_set",
    "camera_point_focus_action",
    "poi_mode_enter",
    "poi_mode_exit",
    "poi_circle_speed_set",
    // DebugMethodEnum
    "debug_mode_open",
    "debug_mode_close",
    "supplement_light_open",
    "supplement_light_close",
    "device_reboot",
    "drone_open",
    "drone_close",
    "drone_format",
    "device_format",
    "cover_open",
    "cover_close",
    "cover_force_close",
    "putter_open",
    "putter_close",
    "charge_open",
    "charge_close",
    "battery_maintenance_switch",
    "alarm_state_switch",
    "battery_store_mode_switch",
    "sdr_workmode_switch",
    "air_conditioner_mode_switch",
    "esim_activate",
    "sim_slot_switch",
    "esim_operator_switch",
    "rtk_calibration",
    // FirmwareMethodEnum
    "ota_create",
    // FlightAreaMethodEnum
    "flight_areas_update",
    "flight_areas_delete",
    // InterconnectionMethodEnum
    "custom_data_transmission_to_esdk",
    "custom_data_transmission_to_psdk",
    // LiveStreamMethodEnum
    "live_start_push",
    "live_stop_push",
    "live_set_quality",
    "live_lens_change",
    // LogMethodEnum
    "fileupload_list",
    "fileupload_start",
    "fileupload_update",
    // MapMethodEnum
    "offline_map_update",
    // MediaMethodEnum
    "upload_flighttask_media_prioritize",
    // PsdkMethodEnum
    "speaker_play_volume_set",
    "speaker_play_mode_set",
    "speaker_play_stop",
    "speaker_replay",
    "speaker_tts_play_start",
    "speaker_audio_play_start",
    "psdk_input_box_text_set",
    "psdk_widget_value_set",
    // RemoteUnlockingMethodEnum
    "unlock_license_switch",
    "unlock_license_update",
    "unlock_license_list",
    // WaylineMethodEnum
    "flighttask_create",
    "flighttask_prepare",
    "flighttask_execute",
    "flighttask_undo",
    "flighttask_pause",
    "flighttask_recovery",
    "in_flight_wayline_deliver",
    "in_flight_wayline_recover",
    "in_flight_wayline_stop",
    "in_flight_wayline_cancel",
    "return_home",
    "return_home_cancel",
};

}  // namespace

TEST_CASE("registry: every cloud-sdk services method has a handler", "[maintenance]") {
    SECTION("dock1 has all of them") {
        Config cfg = golden_config();
        cfg.device.dock_model = "dock1";
        Driver d(cfg);
        for (const auto& method : cloud_services_methods) {
            INFO(method);
            CHECK(d.sim().registry().contains(method));
        }
    }
    SECTION("dock2 has no putter") {
        Driver d;
        for (const auto& method : cloud_services_methods) {
            INFO(method);
            CHECK(d.sim().registry().contains(method) ==
                  (method != "putter_open" && method != "putter_close" &&
                   method != "flighttask_create"));
        }
    }
    SECTION("only the live quality and camera services are stubs") {
        Driver d;
        std::set<std::string> stubs;
        for (const auto* service : d.sim().registry().services())
            if (service->stub) stubs.insert(service->name);
        CHECK(stubs == std::set<std::string>{"live_camera_change", "live_set_quality"});
    }
}
