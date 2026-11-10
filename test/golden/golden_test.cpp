// Golden wire tests. The snapshots in test/golden/snapshots are the contract
// with the backend; they were written from the simulator that ran on a test deployment.
// A diff means the dock now sends something different. Read
// docs/protocol-quirks.md before changing a snapshot.

#include "harness.hpp"

#include <catch2/catch_test_macros.hpp>

#include <format>

using namespace docksim;
using namespace docksim::golden;

namespace {

void check(const std::string& name, const Json& actual) {
    const Json expected = normalize(load_snapshot(name));
    const bool same = same_json(expected, actual);
    if (!same) dump_actual(name, actual);
    INFO("snapshot " << name << " differs; actual written to $TMPDIR/docksim-golden/" << name
                     << ".actual.json");
    CHECK(same);
}

Driver& online(Driver& d) {
    d.connect();
    d.clear();
    return d;
}

const std::string drone_video = std::format("{}/80-0-0/normal-0", drone_sn);

}  // namespace

TEST_CASE("golden: connect") {
    Driver d;
    d.connect();
    check("connect", {{"subscriptions", d.subscriptions()}, {"wire", d.main_wire()}});
}

TEST_CASE("golden: osd_tick") {
    Driver d;
    online(d).osd_tick();
    check("osd_tick", d.main_wire());
}

TEST_CASE("golden: services") {
    const std::vector<std::pair<std::string, Json>> cases{
        // The jobs of the remote-debug panel need remote debug mode.
        {"air_conditioner_mode_switch", {{"action", 1}}},
        {"debug_mode_open", Json::object()},
        {"cover_open", Json::object()},
        {"cover_close", Json::object()},
        {"cover_force_close", Json::object()},
        {"drone_open", Json::object()},
        {"drone_close", Json::object()},
        {"charge_open", Json::object()},
        {"charge_close", Json::object()},
        {"supplement_light_open", Json::object()},
        {"supplement_light_close", Json::object()},
        {"alarm_state_switch", {{"action", 1}}},
        {"battery_store_mode_switch", {{"action", 2}}},
        {"battery_maintenance_switch", {{"action", 1}}},
        {"sdr_workmode_switch", {{"link_workmode", 1}}},
        {"device_reboot", Json::object()},
        {"debug_mode_close", Json::object()},
        {"return_home", Json::object()},
        {"return_home_cancel", Json::object()},
        {"flighttask_pause", Json::object()},
        {"flighttask_recovery", Json::object()},
        {"flighttask_stop", {{"flight_id", "f-none"}}},
        {"flighttask_undo", {{"flight_ids", Json::array({"f-none"})}}},
        {"flighttask_execute", {{"flight_id", "f-none"}}},
        {"live_set_quality", {{"video_id", drone_video}, {"video_quality", 2}}},
        {"live_camera_change", {{"camera_position", 0}}},
        {"live_lens_change", {{"video_id", drone_video}, {"video_type", "zoom"}}},
        {"live_stop_push", {{"video_id", drone_video}}},
        {"camera_screen_drag",
         {{"payload_index", "80-0-0"},
          {"locked", false},
          {"pitch_speed", 0.2},
          {"yaw_speed", -0.3}}},
        {"camera_aim",
         {{"payload_index", "80-0-0"},
          {"camera_type", "wide"},
          {"locked", false},
          {"x", 0.75},
          {"y", 0.25}}},
        {"camera_look_at",
         {{"payload_index", "80-0-0"},
          {"locked", true},
          {"latitude", 47.2640},
          {"longitude", 11.4020},
          {"height", 600.0}}},
        {"camera_focal_length_set",
         {{"payload_index", "80-0-0"}, {"camera_type", "zoom"}, {"zoom_factor", 12}}},
        {"gimbal_reset", {{"payload_index", "80-0-0"}, {"reset_mode", 1}}},
        {"upload_flighttask_media_prioritize", {{"flight_id", "f-none"}}},
        {"flight_areas_update", Json::object()},
        {"flight_authority_grab", Json::object()},
        {"payload_authority_grab", {{"payload_index", "80-0-0"}}},
        {"fly_to_point",
         {{"fly_to_id", "fly-none"},
          {"max_speed", 5},
          {"points",
           Json::array({{{"latitude", 47.264}, {"longitude", 11.401}, {"height", 620}}})}}},
        {"fly_to_point_update",
         {{"max_speed", 5},
          {"points",
           Json::array({{{"latitude", 47.264}, {"longitude", 11.401}, {"height", 620}}})}}},
        {"fly_to_point_stop", Json::object()},
        {"camera_recording_start", {{"payload_index", "80-0-0"}}},
        {"camera_recording_stop", {{"payload_index", "80-0-0"}}},
        {"camera_mode_switch", {{"payload_index", "80-0-0"}, {"camera_mode", 0}}},
        {"camera_photo_take", {{"payload_index", "80-0-0"}}},
        {"camera_photo_stop", {{"payload_index", "80-0-0"}}},
        {"camera_screen_split", {{"payload_index", "80-0-0"}, {"enable", true}}},
        {"photo_storage_set",
         {{"payload_index", "80-0-0"}, {"photo_storage_settings", Json::array({"current", "ir"})}}},
        {"video_storage_set",
         {{"payload_index", "80-0-0"}, {"video_storage_settings", Json::array({"wide", "zoom"})}}},
        {"camera_exposure_mode_set",
         {{"payload_index", "80-0-0"}, {"camera_type", "zoom"}, {"exposure_mode", 4}}},
        {"camera_exposure_set",
         {{"payload_index", "80-0-0"}, {"camera_type", "zoom"}, {"exposure_value", 18}}},
        {"camera_focus_mode_set",
         {{"payload_index", "80-0-0"}, {"camera_type", "wide"}, {"focus_mode", 0}}},
        {"camera_focus_value_set",
         {{"payload_index", "80-0-0"}, {"camera_type", "wide"}, {"focus_value", 50}}},
        {"camera_point_focus_action",
         {{"payload_index", "80-0-0"}, {"camera_type", "zoom"}, {"x", 0.4}, {"y", 0.6}}},
        {"ir_metering_mode_set", {{"payload_index", "80-0-0"}, {"mode", 1}}},
        {"ir_metering_point_set", {{"payload_index", "80-0-0"}, {"x", 0.3}, {"y", 0.7}}},
        {"ir_metering_area_set",
         {{"payload_index", "80-0-0"}, {"x", 0.1}, {"y", 0.2}, {"width", 0.3}, {"height", 0.4}}},
        {"poi_mode_enter", {{"latitude", 47.264}, {"longitude", 11.401}, {"height", 600}}},
        {"poi_circle_speed_set", {{"circle_speed", 4.5}}},
        {"poi_mode_exit", Json::object()},
        {"in_flight_wayline_deliver",
         {{"in_flight_wayline_id", "ifw-none"},
          {"file", {{"url", "https://example.invalid/w.kmz"}, {"fingerprint", "x"}}},
          {"rth_altitude", 60},
          {"rth_mode", 1},
          {"out_of_control_action", 0},
          {"exit_wayline_when_rc_lost", 0},
          {"wayline_precision_type", 1}}},
        {"in_flight_wayline_stop", {{"in_flight_wayline_id", "ifw-none"}}},
        {"in_flight_wayline_recover", {{"in_flight_wayline_id", "ifw-none"}}},
        {"in_flight_wayline_cancel", Json::object()},
        {"flighttask_create",
         {{"flight_id", "f-create"}, {"file", {{"url", "https://example.invalid/m.kmz"}}}}},
        {"unlock_license_list", {{"device_model_domain", 3}}},
        {"fileupload_list", {{"module_list", Json::array({"0", "3"})}}},
        {"speaker_tts_play_start", {{"psdk_index", 1}}},
        {"no_such_method", Json::object()},
    };
    Driver d;
    online(d);
    d.run_spawned = true;
    Json results = Json::object();
    for (const auto& [method, data] : cases) {
        d.clear();
        d.service(method, data);
        results[method] = d.main_wire();
    }
    // The gimbal and dock state after the sequence above, as the OSD shows it.
    d.clear();
    d.osd_tick();
    results["osd_after"] = d.main_wire();
    check("services", results);
}

TEST_CASE("golden: live_start_push") {
    Driver d;
    online(d).service(
        "live_start_push",
        {{"url",
          "https://example.invalid/mtx/org-SIMDRONE0000001-80-0-0-normal-0/whip?streamToken=x"},
         {"url_type", 4},
         {"video_id", drone_video},
         {"video_quality", 0}});
    check("live_start_push", d.main_wire());
}

TEST_CASE("golden: property_set") {
    Driver d;
    online(d).property_set(dock_sn, {{"silent_mode", 1}});
    check("property_set", d.main_wire());
}

TEST_CASE("golden: mission") {
    const Json prepare = {
        {"flight_id", "f-1"},
        {"task_type", 0},
        {"execute_time", 0},
        {"rth_altitude", 60},
        {"file", {{"url", "https://example.invalid/w.kmz"}, {"fingerprint", "x"}}},
    };
    Driver d;
    online(d);
    d.service("flighttask_prepare", prepare);
    d.service("flighttask_prepare", prepare);
    d.service("flighttask_execute", {{"flight_id", "f-1"}});
    d.wait_mission();
    d.upload_media("f-1");
    check("mission", d.main_wire());
    CHECK(d.puts.size() == 2);
}

TEST_CASE("media: a retry after a failed upload sends only the files still missing") {
    Driver d;
    online(d);
    d.service("flighttask_prepare",
              {
                  {"flight_id", "f-1"},
                  {"task_type", 0},
                  {"execute_time", 0},
                  {"rth_altitude", 60},
                  {"file", {{"url", "https://example.invalid/w.kmz"}, {"fingerprint", "x"}}},
              });
    d.service("flighttask_execute", {{"flight_id", "f-1"}});
    d.wait_mission();
    d.fail_put = 2;
    d.upload_media("f-1");
    REQUIRE(d.puts.size() == 1);
    REQUIRE(d.sim().uploads().pending().contains("f-1"));
    d.upload_media("f-1");
    REQUIRE(d.puts.size() == 2);
    CHECK(d.puts[0].first != d.puts[1].first);
    CHECK_FALSE(d.sim().uploads().pending().contains("f-1"));
}

TEST_CASE("golden: drc") {
    const Json enter = {
        {"mqtt_broker",
         {
             {"address", "wss://drc.example.invalid:443/mqtt"},
             {"username", "drc-user"},
             {"password", "jwt"},
             {"client_id", std::format("org-{}", dock_sn)},
             {"expire_time", 1'760'003'600},
             {"enable_tls", false},
         }},
        {"osd_frequency", 10},
        {"hsi_frequency", 1},
    };
    Driver d;
    online(d).service("drc_mode_enter", enter);
    const auto down = std::format("thing/product/{}/drc/down", dock_sn);
    d.drc_inject(down,
                 {{"method", "heart_beat"}, {"data", {{"seq", 7}, {"timestamp", 1}}}, {"seq", 7}});
    // The backend sends a UUID seq next to data and waits for it on drc/up.
    d.drc_inject(down, {{"method", "drone_emergency_stop"},
                        {"data", ""},
                        {"seq", "5f1c2b4e-0000-4000-8000-000000000001"}});
    d.drc_inject(down, {{"method", "drone_control"},
                        {"data", {{"seq", 3}, {"x", 1.0}, {"y", 0.0}, {"h", 0.0}, {"w", 0.0}}}});
    const Json drc = d.drc_wire();
    d.service("drc_mode_exit", Json::object());
    check("drc", {{"main", d.main_wire()}, {"drc", drc}});
}

TEST_CASE("golden: manual_flight") {
    // One-key takeoff 60 m north to 40 m, a fly-to 50 m east, then return to home.
    Driver d;
    online(d);
    const auto& home = d.sim().ctx().cfg().device;
    const double north = home.latitude + 60.0 / 111'195.0;
    d.service("flight_authority_grab", Json::object());
    d.service("takeoff_to_point", {
                                      {"flight_id", "takeoff-1"},
                                      {"target_latitude", north},
                                      {"target_longitude", home.longitude},
                                      {"target_height", home.altitude + 40.0},
                                      {"security_takeoff_height", 30},
                                      {"rth_altitude", 50},
                                      {"rth_mode", 1},
                                      {"max_speed", 10},
                                      {"commander_mode_lost_action", 1},
                                      {"commander_flight_mode", 1},
                                      {"commander_flight_height", 40},
                                  });
    d.advance(30.0);
    d.service("fly_to_point",
              {{"fly_to_id", "flyto-1"},
               {"max_speed", 10},
               {"points", Json::array({{{"latitude", north},
                                        {"longitude", home.longitude + 50.0 / 75'500.0},
                                        {"height", home.altitude + 40.0}}})}});
    d.advance(15.0);
    d.service("return_home", Json::object());
    d.wait_manual();
    check("manual_flight", d.main_wire());
}

TEST_CASE("golden: poi_wayline") {
    // Takeoff 60 m north to 40 m, a POI orbit around a point 20 m further
    // north, an in-flight wayline from the hover, then return to home. The
    // Dock 2 gateway does not route POI, so the dock is a Dock 3.
    Config cfg = golden_config();
    cfg.device.dock_model = "dock3";
    Driver d(cfg);
    online(d);
    const auto& home = d.sim().ctx().cfg().device;
    const double north = home.latitude + 60.0 / 111'195.0;
    d.service("takeoff_to_point", {
                                      {"flight_id", "takeoff-1"},
                                      {"target_latitude", north},
                                      {"target_longitude", home.longitude},
                                      {"target_height", home.altitude + 40.0},
                                      {"security_takeoff_height", 30},
                                      {"rth_altitude", 50},
                                      {"max_speed", 10},
                                  });
    d.advance(30.0);
    d.clear();
    d.service("poi_mode_enter", {{"latitude", north + 20.0 / 111'195.0},
                                 {"longitude", home.longitude},
                                 {"height", home.altitude}});
    d.advance(6.0);
    d.service("poi_circle_speed_set", {{"circle_speed", 5}});
    d.advance(6.0);
    d.service("poi_mode_exit", Json::object());
    d.advance(1.0);
    d.service("in_flight_wayline_deliver",
              {{"in_flight_wayline_id", "ifw-1"},
               {"file", {{"url", "https://example.invalid/w.kmz"}, {"fingerprint", "x"}}},
               {"rth_altitude", 60},
               {"rth_mode", 1},
               {"out_of_control_action", 0},
               {"exit_wayline_when_rc_lost", 0},
               {"wayline_precision_type", 1}});
    d.advance(30.0);
    d.service("return_home", Json::object());
    d.wait_manual();
    check("poi_wayline", d.main_wire());
}
