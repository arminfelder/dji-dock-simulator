// Manual flight and DRC: takeoff_to_point, fly_to_point, return to home and
// the drc/down commands, driven on the virtual clock of the golden harness.

#include "flight_support.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <format>
#include <string>
#include <vector>

using namespace docksim;
using namespace docksim::golden;
using namespace docksim::flight_test;
using Catch::Approx;
using code::DroneMode;
using flight::Phase;

TEST_CASE("manual: takeoff_to_point runs the takeoff sequence", "[manual]") {
    Driver d(manual_config());
    d.connect();
    auto& ctx = d.sim().ctx();
    d.clear();
    d.service("takeoff_to_point", takeoff_data(d));
    CHECK(reply_result(d, "takeoff_to_point") == 0);
    d.advance(0.5);
    CHECK(ctx.dock.mode_code == code::DockMode::working);
    CHECK(ctx.dock.drone_in_dock == 1);
    REQUIRE(events(d, "takeoff_to_point_progress").size() == 1);
    CHECK(events(d, "takeoff_to_point_progress")[0]["status"] == "task_ready");

    d.advance(2.0);
    CHECK(ctx.dock.cover_state == 1);
    CHECK(ctx.dock.drone_in_dock == 0);
    CHECK(ctx.drone.mode_code == DroneMode::auto_takeoff);
    CHECK(ctx.drone.height > 0.0);

    d.advance(7.5);
    CHECK(ctx.drone.mode_code == DroneMode::live_flight_control);

    d.advance(20.0);
    CHECK(ctx.drone.mode_code == DroneMode::manual);
    CHECK(ctx.drone.height == Approx(40.0).margin(0.3));
    CHECK(metres_from_home(d) == Approx(60.0).margin(0.6));
    CHECK(d.sim().manual().airborne());

    const auto progress = events(d, "takeoff_to_point_progress");
    const auto names = statuses(progress);
    REQUIRE(names.size() >= 5);
    CHECK(names.front() == "task_ready");
    CHECK(names[names.size() - 2] == "wayline_ok");
    CHECK(names.back() == "task_finish");
    CHECK(std::count(names.begin(), names.end(), "wayline_progress") >= 10);
    for (const auto& e : progress) {
        CHECK(e["flight_id"] == "takeoff-1");
        CHECK(e.contains("track_id"));
        CHECK(e["result"] == 0);
        CHECK(e["remaining_time"].is_number());
        REQUIRE(e["planned_path_points"].size() == 1);
        CHECK(e["planned_path_points"][0]["height"].get<double>() ==
              Approx(home(d).altitude + 40.0));
    }
    CHECK(progress[1]["remaining_distance"].get<double>() >
          progress[names.size() - 3]["remaining_distance"].get<double>());
}

TEST_CASE("manual: refusals", "[manual]") {
    Driver d(manual_config());
    d.connect();
    d.clear();
    d.service("fly_to_point",
              {{"fly_to_id", "f"}, {"points", Json::array({point_north(d, 10, 30)})}});
    CHECK(reply_result(d, "fly_to_point") == 319022);
    d.service("fly_to_point_update", {{"points", Json::array({point_north(d, 10, 30)})}});
    CHECK(reply_result(d, "fly_to_point_update") == 319022);

    d.service("takeoff_to_point", takeoff_data(d));
    CHECK(reply_result(d, "takeoff_to_point") == 0);
    d.service("takeoff_to_point", takeoff_data(d));
    CHECK(reply_result(d, "takeoff_to_point") == 514116);
    // Still in the dock: no return to home.
    d.service("return_home", Json::object());
    CHECK(reply_result(d, "return_home") == 316024);
    // The mission engine refuses while the manual flight owns the aircraft.
    CHECK(d.sim().mission().execute("anything") == code::Result::task_not_prepared);
    d.service("flighttask_prepare",
              {{"flight_id", "f-1"},
               {"file", {{"url", "https://example.invalid/w.kmz"}, {"fingerprint", "x"}}}});
    d.service("flighttask_execute", {{"flight_id", "f-1"}});
    CHECK(reply_result(d, "flighttask_execute") == 514001);

    d.advance(30.0);
    REQUIRE(d.sim().manual().phase() == Phase::hover);
    d.service("return_home_cancel", Json::object());
    CHECK(reply_result(d, "return_home_cancel") == 319043);
    d.service("takeoff_to_point", takeoff_data(d));
    CHECK(reply_result(d, "takeoff_to_point") == 514116);
    d.service("takeoff_to_point", {{"flight_id", "x"}});
    CHECK(reply_result(d, "takeoff_to_point") == 514116);
}

TEST_CASE("manual: takeoff_to_point without a target is a parameter error", "[manual]") {
    Driver d(manual_config());
    d.connect();
    d.clear();
    d.service("takeoff_to_point", {{"flight_id", "x"}, {"target_latitude", 47.0}});
    CHECK(reply_result(d, "takeoff_to_point") == 514171);
    CHECK_FALSE(d.sim().manual().flying());
}

TEST_CASE("manual: authority grab publishes the control source on the aircraft state", "[manual]") {
    Driver d(manual_config());
    d.connect();
    d.run_spawned = true;
    for (const auto* method : {"flight_authority_grab", "payload_authority_grab"}) {
        d.clear();
        d.service(method, {{"payload_index", "80-0-0"}});
        const auto& wire = d.main_transport().published;
        REQUIRE(wire.size() == 2);
        CHECK(wire[0].topic.ends_with("/services_reply"));
        CHECK(wire[0].payload["data"]["result"] == 0);
        CHECK(wire[1].topic == std::format("thing/product/{}/state", drone_sn));
        CHECK(wire[1].payload["gateway"] == dock_sn);
        CHECK(wire[1].payload["data"]["control_source"] == "A");
        CHECK(wire[1].payload["data"]["payloads"][0]["control_source"] == "A");
    }
}

TEST_CASE("manual: sticks move the aircraft only with DRC up and in manual mode", "[manual]") {
    Driver d(manual_config());
    hovering(d);
    auto& drone = d.sim().ctx().drone;
    drone.attitude_head = 0.0;
    const double lat0 = drone.latitude;

    SECTION("no DRC session: sticks change nothing") {
        d.service("drc_mode_enter", drc_enter());
        d.service("drc_mode_exit", Json::object());
        stick(d, 5, 0, 0, 0);
        d.advance(1.0);
        CHECK(drone.latitude == lat0);
    }

    SECTION("forward is north at heading 0, and a command holds for 0.5 s") {
        d.service("drc_mode_enter", drc_enter());
        stick(d, 5, 0, 0, 0);
        d.advance(1.0);
        const double moved = (drone.latitude - lat0) * metres_per_degree;
        CHECK(moved == Approx(5.0 * flight::stick_hold_s).margin(0.6));
        CHECK(drone.longitude == Approx(home(d).longitude).margin(1e-9));
        const double lat1 = drone.latitude;
        d.advance(1.0);
        CHECK(drone.latitude == lat1);
        CHECK(drone.horizontal_speed == 0.0);
        CHECK(drone.mode_code == DroneMode::manual);
    }

    SECTION("motion is relative to the heading") {
        d.service("drc_mode_enter", drc_enter());
        drone.attitude_head = 90.0;
        const double lon0 = drone.longitude;
        stick(d, 4, 0, 0, 0);
        d.advance(1.0);
        CHECK(drone.longitude > lon0);
        CHECK(drone.latitude == Approx(lat0).margin(1e-8));
        // Right at heading 90 is south.
        stick(d, 0, 4, 0, 0);
        d.advance(1.0);
        CHECK(drone.latitude < lat0);
    }

    SECTION("yaw, climb, and the height floor") {
        d.service("drc_mode_enter", drc_enter());
        stick(d, 0, 0, 0, 90);
        d.advance(1.0);
        CHECK(drone.attitude_head == Approx(45.0).margin(9.1));
        const double h0 = drone.height;
        stick(d, 0, 0, 5, 0);
        d.advance(1.0);
        CHECK(drone.height == Approx(h0 + 2.5).margin(0.6));
        for (int i = 0; i < 40; ++i) {
            stick(d, 0, 0, -4, 0, i);
            d.advance(0.4);
        }
        CHECK(drone.height == Approx(flight::min_height_m));
        CHECK(drone.total_flight_distance > 0.0);
    }

    SECTION("sticks during an automatic flight give joystick_invalid_notify 4, at most every 2 s") {
        d.service("drc_mode_enter", drc_enter());
        d.service("fly_to_point", {{"fly_to_id", "f-1"},
                                   {"max_speed", 2},
                                   {"points", Json::array({point_north(d, 200, 40)})}});
        d.advance(0.5);
        REQUIRE(drone.mode_code == DroneMode::live_flight_control);
        d.clear();
        for (int i = 0; i < 20; ++i) {
            stick(d, 3, 0, 0, 0, i);
            d.advance(0.1);
        }
        const auto notes = events(d, "joystick_invalid_notify");
        REQUIRE(notes.size() == 1);
        CHECK(notes[0]["reason"] == 4);
        // Zero sticks, which the web UI sends all the time, give no notify.
        d.advance(2.0);
        d.clear();
        stick(d, 0, 0, 0, 0);
        CHECK(events(d, "joystick_invalid_notify").empty());
    }
}

TEST_CASE("manual: drone_control replies on drc/up with the seq", "[manual]") {
    Driver d(manual_config());
    d.connect();
    d.service("drc_mode_enter", drc_enter());
    stick(d, 0, 0, 0, 0, 42);
    const auto replies = drc_up(d, "drone_control");
    REQUIRE(replies.size() == 1);
    CHECK(replies[0]["data"]["result"] == 0);
    CHECK(replies[0]["data"]["output"]["seq"] == 42);
}

TEST_CASE("manual: fly_to_point, update and stop", "[manual]") {
    Driver d(manual_config());
    hovering(d);
    auto& drone = d.sim().ctx().drone;
    d.clear();
    d.service("fly_to_point", {{"fly_to_id", "f-1"},
                               {"max_speed", 5},
                               {"points", Json::array({point_north(d, 160, 40)})}});
    CHECK(reply_result(d, "fly_to_point") == 0);
    d.advance(3.0);
    CHECK(drone.mode_code == DroneMode::live_flight_control);
    CHECK(drone.horizontal_speed == Approx(5.0));

    SECTION("arrive") {
        d.advance(30.0);
        CHECK(drone.mode_code == DroneMode::manual);
        CHECK(metres_from_home(d) == Approx(160.0).margin(0.6));
        const auto progress = events(d, "fly_to_point_progress");
        const auto names = statuses(progress);
        CHECK(names.back() == "wayline_ok");
        CHECK(std::count(names.begin(), names.end(), "wayline_progress") >= 10);
        CHECK(progress.front()["fly_to_id"] == "f-1");
        CHECK(progress.back()["remaining_distance"].get<double>() < 1.0);
    }

    SECTION("update moves the target") {
        d.service("fly_to_point_update",
                  {{"max_speed", 8}, {"points", Json::array({point_north(d, 70, 50)})}});
        CHECK(reply_result(d, "fly_to_point_update") == 0);
        d.advance(30.0);
        CHECK(drone.mode_code == DroneMode::manual);
        CHECK(metres_from_home(d) == Approx(70.0).margin(0.6));
        CHECK(drone.height == Approx(50.0).margin(0.3));
        CHECK(statuses(events(d, "fly_to_point_progress")).back() == "wayline_ok");
    }

    SECTION("stop hovers in manual mode") {
        d.service("fly_to_point_stop", Json::object());
        CHECK(reply_result(d, "fly_to_point_stop") == 0);
        d.advance(1.0);
        CHECK(drone.mode_code == DroneMode::manual);
        CHECK(d.sim().manual().phase() == Phase::hover);
        CHECK(statuses(events(d, "fly_to_point_progress")).back() == "wayline_cancel");
        const double lat = drone.latitude;
        d.advance(2.0);
        CHECK(drone.latitude == lat);
    }
}

TEST_CASE("manual: return_home flies home, lands and closes the dock", "[manual]") {
    Driver d(manual_config());
    hovering(d);
    auto& ctx = d.sim().ctx();
    const int jobs = ctx.dock.job_number;
    d.clear();
    d.service("return_home", Json::object());
    CHECK(reply_result(d, "return_home") == 0);
    d.advance(0.2);
    CHECK(ctx.drone.mode_code == DroneMode::auto_rth);
    const auto info = events(d, "return_home_info");
    REQUIRE(info.size() == 1);
    // The firmware return_home_info has no flight_id.
    CHECK_FALSE(info[0].contains("flight_id"));
    CHECK(info[0]["home_dock_sn"] == dock_sn);
    CHECK(info[0]["last_point_type"] == 0);
    REQUIRE(info[0]["planned_path_points"].size() == 2);
    // Home at the rth altitude of the takeoff, which is above the 40 m of the hover.
    CHECK(info[0]["planned_path_points"][1]["height"].get<double>() ==
          Approx(home(d).altitude + 50.0));
    CHECK(info[0]["multi_dock_home_info"][0]["sn"] == dock_sn);
    CHECK(info[0]["multi_dock_home_info"][0]["plan_status"] == 3);
    CHECK(info[0]["multi_dock_home_info"][0]["home_distance"].get<double>() ==
          Approx(60.0).margin(0.6));

    d.advance(4.0);
    CHECK(ctx.drone.height == Approx(50.0).margin(0.3));
    bool landing_seen = false;
    for (int i = 0; i < 400 && d.sim().manual().flying(); ++i) {
        d.advance(0.1);
        landing_seen = landing_seen || ctx.drone.mode_code == DroneMode::auto_landing;
        if (ctx.drone.mode_code == DroneMode::auto_landing) CHECK(metres_from_home(d) < 0.6);
    }
    CHECK(landing_seen);
    CHECK_FALSE(d.sim().manual().flying());
    CHECK(ctx.dock.drone_in_dock == 1);
    CHECK(ctx.dock.cover_state == 0);
    CHECK(ctx.dock.putter_state == 0);
    CHECK(ctx.dock.job_number == jobs + 1);
    CHECK(ctx.dock.mode_code == code::DockMode::idle);
    CHECK(ctx.drone.mode_code == DroneMode::standby);
    CHECK(ctx.drone.height == 0.0);
    // The dock can take off again.
    d.service("takeoff_to_point", takeoff_data(d));
    CHECK(reply_result(d, "takeoff_to_point") == 0);
}

TEST_CASE("manual: return_home_cancel hovers in manual mode", "[manual]") {
    Driver d(manual_config());
    hovering(d);
    auto& drone = d.sim().ctx().drone;
    d.service("return_home", Json::object());
    d.advance(5.0);
    REQUIRE(drone.mode_code == DroneMode::auto_rth);
    d.service("return_home_cancel", Json::object());
    CHECK(reply_result(d, "return_home_cancel") == 0);
    d.advance(0.5);
    CHECK(drone.mode_code == DroneMode::manual);
    const double lat = drone.latitude;
    d.advance(3.0);
    CHECK(drone.latitude == lat);
    CHECK(d.sim().manual().airborne());

    SECTION("a landing can be cancelled too") {
        d.service("return_home", Json::object());
        for (int i = 0; i < 400 && drone.mode_code != DroneMode::auto_landing; ++i) d.advance(0.1);
        REQUIRE(drone.mode_code == DroneMode::auto_landing);
        d.advance(1.0);
        d.service("return_home_cancel", Json::object());
        CHECK(reply_result(d, "return_home_cancel") == 0);
        d.advance(0.5);
        CHECK(drone.mode_code == DroneMode::manual);
        CHECK(drone.height > 0.0);
    }
}

TEST_CASE("manual: low battery starts return to home", "[manual]") {
    Driver d(manual_config());
    hovering(d);
    auto& drone = d.sim().ctx().drone;
    d.service("drc_mode_enter", drc_enter());
    d.clear();
    drone.battery_level = 30.4;
    d.advance(1.0);
    CHECK(drone.mode_code == DroneMode::auto_rth);
    const auto notes = events(d, "joystick_invalid_notify");
    REQUIRE(notes.size() == 1);
    CHECK(notes[0]["reason"] == 1);
    CHECK(events(d, "return_home_info").size() == 1);
    // A cancel keeps the aircraft in the air. The low-battery return comes only once.
    d.service("return_home_cancel", Json::object());
    d.advance(1.0);
    CHECK(drone.mode_code == DroneMode::manual);
}

TEST_CASE("manual: DRC emergency stop and landing reply with the request seq", "[manual]") {
    Driver d(manual_config());
    d.connect();
    d.service("drc_mode_enter", drc_enter());
    auto& drone = d.sim().ctx().drone;

    d.drc_inject(drc_down, {{"method", "drc_emergency_landing"}, {"data", ""}, {"seq", "uuid-0"}});
    auto replies = drc_up(d, "drc_emergency_landing");
    REQUIRE(replies.size() == 1);
    CHECK(replies[0]["seq"] == "uuid-0");
    CHECK(replies[0]["data"]["result"] == 314000);

    d.service("takeoff_to_point", takeoff_data(d));
    d.advance(30.0);
    heart_beat(d);
    d.service("fly_to_point", {{"fly_to_id", "f-1"},
                               {"max_speed", 5},
                               {"points", Json::array({point_north(d, 200, 40)})}});
    d.advance(2.0);
    REQUIRE(drone.mode_code == DroneMode::live_flight_control);

    d.drc_inject(drc_down, {{"method", "drone_emergency_stop"}, {"data", ""}, {"seq", "uuid-1"}});
    replies = drc_up(d, "drone_emergency_stop");
    REQUIRE(replies.size() == 1);
    CHECK(replies[0]["seq"] == "uuid-1");
    CHECK(replies[0]["data"]["result"] == 0);
    d.advance(0.5);
    CHECK(drone.mode_code == DroneMode::manual);
    CHECK(statuses(events(d, "fly_to_point_progress")).back() == "wayline_cancel");

    const double lat = drone.latitude;
    d.drc_inject(drc_down, {{"method", "drc_force_landing"}, {"data", ""}, {"seq", "uuid-2"}});
    replies = drc_up(d, "drc_force_landing");
    REQUIRE(replies.size() == 1);
    CHECK(replies[0]["seq"] == "uuid-2");
    CHECK(replies[0]["data"]["result"] == 0);
    d.advance(1.0);
    CHECK(drone.mode_code == DroneMode::force_landing);
    CHECK(drone.latitude == lat);
    heart_beat(d);
    d.advance(15.0);
    CHECK_FALSE(d.sim().manual().flying());
    CHECK(d.sim().ctx().dock.drone_in_dock == 1);
}

TEST_CASE("manual: other drc/down commands answer 0, and the state follows", "[manual]") {
    Driver d(manual_config());
    d.connect();
    d.service("drc_mode_enter", drc_enter());
    for (const auto* method : {"drc_camera_iso_set", "drc_stealth_state_set"}) {
        d.drc_inject(drc_down, {{"method", method}, {"data", Json::object()}, {"seq", method}});
        const auto replies = drc_up(d, method);
        REQUIRE(replies.size() == 1);
        CHECK(replies[0]["seq"] == method);
        CHECK(replies[0]["data"]["result"] == 0);
        CHECK_FALSE(replies[0].contains("timestamp"));
    }
    // The Dock 2 gateway does not route these methods. The dock gives no reply.
    for (const auto* method :
         {"drc_light_mode_set", "drc_speaker_tts_set", "drc_ai_model_select"}) {
        d.drc_inject(drc_down, {{"method", method}, {"data", Json::object()}, {"seq", method}});
        CHECK(drc_up(d, method).empty());
    }
    d.drc_inject(drc_down, {{"method", "drc_night_lights_state_set"},
                            {"data", {{"night_lights_state", 1}}},
                            {"seq", "n"}});
    CHECK(d.sim().ctx().drone.night_lights_state == 1);
    d.drc_session().published.clear();
    d.drc_inject(drc_down, {{"method", "drc_initial_state_subscribe"}, {"data", ""}, {"seq", "i"}});
    const auto& wire = d.drc_session().published;
    REQUIRE(wire.size() >= 6);
    CHECK(wire[0].payload["method"] == "drc_initial_state_subscribe");
    CHECK(wire[0].payload["data"]["result"] == 0);
    const auto state = drc_up(d, "drc_drone_state_push");
    REQUIRE(state.size() == 1);
    CHECK(state[0]["data"]["night_lights_state"] == 1);
    CHECK(state[0]["data"]["rth_altitude"] == 100);
    CHECK(drc_up(d, "osd_info_push").size() == 1);
    CHECK(drc_up(d, "hsi_info_push").size() == 1);
}

TEST_CASE("manual: DRC pushes follow osd_frequency and hsi_frequency", "[manual]") {
    Driver d(manual_config());
    d.connect();
    d.service("drc_mode_enter", drc_enter());
    d.drc_session().published.clear();
    d.advance(2.05);
    const auto osd = drc_up(d, "osd_info_push");
    CHECK(osd.size() >= 20);
    CHECK(osd.size() <= 21);
    const auto hsi = drc_up(d, "hsi_info_push");
    CHECK(hsi.size() == 2);
    CHECK(drc_up(d, "delay_info_push").size() == hsi.size());
    CHECK(drc_up(d, "drc_drone_state_push").size() == hsi.size());
    CHECK(drc_up(d, "drc_batteries_info_push").size() == hsi.size());
    // The envelope carries an increasing seq and a timestamp.
    CHECK(osd[1]["seq"].get<std::int64_t>() > osd[0]["seq"].get<std::int64_t>());
    CHECK(osd[0].contains("timestamp"));
    for (const auto* key : {"latitude", "longitude", "height", "elevation", "attitude_head",
                            "horizontal_speed", "vertical_speed", "home_distance", "wind_speed",
                            "wind_direction", "gimbal_pitch", "speed_x"}) {
        CHECK(osd[0]["data"].contains(key));
    }
    CHECK(osd[0]["data"]["height"].get<double>() == Approx(home(d).altitude));
    CHECK(hsi[0]["data"]["around_distances"].size() == 360);

    d.service("drc_mode_enter", [] {
        auto data = drc_enter();
        data["osd_frequency"] = 5;
        return data;
    }());
    d.drc_session().published.clear();
    d.advance(2.0);
    CHECK(drc_up(d, "osd_info_push").size() <= 11);
}

TEST_CASE("manual: heart_beat echo, push QoS and the Dock 1 drc/down routes", "[manual]") {
    Driver d(manual_config());
    d.connect();
    d.service("drc_mode_enter", drc_enter());
    d.drc_session().published.clear();
    const Json beat = {{"method", "heart_beat"}, {"data", {{"timestamp", 1234}}}, {"seq", 9}};
    d.drc_inject(drc_down, beat);
    d.advance(0.15);
    const auto& wire = d.drc_session().published;
    bool echoed = false;
    for (const auto& p : wire) {
        if (p.payload.value("method", "") == "heart_beat") {
            // The real dock returns the same message, QoS 0.
            CHECK(p.payload == beat);
            CHECK(p.qos == 0);
            echoed = true;
        } else if (p.payload.value("method", "").ends_with("_push")) {
            CHECK(p.qos == 1);
        }
    }
    CHECK(echoed);

    auto cfg = manual_config();
    cfg.device.dock_model = "dock1";
    Driver d1(cfg);
    d1.connect();
    d1.service("drc_mode_enter", drc_enter());
    d1.drc_session().published.clear();
    d1.drc_inject(drc_down, {{"method", "drc_camera_iso_set"},
                             {"data", {{"camera_type", "zoom"}, {"iso_value", 5}}},
                             {"seq", "iso"}});
    CHECK(drc_up(d1, "drc_camera_iso_set").empty());
    d1.drc_inject(drc_down,
                  {{"method", "drone_emergency_stop"}, {"data", Json::object()}, {"seq", "e"}});
    CHECK(drc_up(d1, "drone_emergency_stop").size() == 1);
}

TEST_CASE("manual: DRC ends after 60 s without heart_beat", "[manual]") {
    Driver d(manual_config());
    d.connect();
    d.service("drc_mode_enter", drc_enter());
    auto& dock = d.sim().ctx().dock;
    CHECK(dock.drc_state == code::DrcState::connected);
    d.advance(40.0);
    heart_beat(d);
    d.advance(50.0);
    CHECK(dock.drc_state == code::DrcState::connected);
    d.clear();
    d.advance(15.0);
    CHECK(dock.drc_state == code::DrcState::disconnected);
    const auto notes = events(d, "drc_status_notify");
    REQUIRE(notes.size() == 1);
    CHECK(notes[0]["drc_state"] == 0);
    // The pushes stop with the session.
    d.drc_session().published.clear();
    d.advance(1.0);
    CHECK(d.drc_session().published.empty());
}

TEST_CASE("manual: drc_mode_enter is idempotent for the session that is up", "[manual]") {
    Driver d(manual_config());
    d.connect();
    d.clear();
    for (int i = 0; i < 3; ++i) {
        d.service("drc_mode_enter", drc_enter());
        CHECK(reply_result(d, "drc_mode_enter") == 0);
    }
    CHECK(d.drc_session_count() == 1);
    CHECK(events(d, "drc_status_notify").size() == 1);
    // The cloud sends a new JWT with every enter. It keeps the session.
    d.service("drc_mode_enter", drc_enter("jwt-2"));
    CHECK(reply_result(d, "drc_mode_enter") == 0);
    CHECK(d.drc_session_count() == 1);
    CHECK(events(d, "drc_status_notify").size() == 1);
    // Another client id opens a new session.
    auto other = drc_enter("jwt-3");
    other["mqtt_broker"]["client_id"] = "org-other";
    d.service("drc_mode_enter", other);
    CHECK(reply_result(d, "drc_mode_enter") == 0);
    CHECK(d.drc_session_count() == 2);
}

TEST_CASE("manual: the dock osd carries drc_state and sub_device", "[manual]") {
    Driver d(manual_config());
    d.connect();
    d.service("drc_mode_enter", drc_enter());
    d.clear();
    d.osd_tick();
    const auto& osd = d.main_transport().published.at(0).payload["data"];
    CHECK(osd["mode_code"] == 0);
    // The backend takes drc_state only from a dock osd without mode_code and job_number.
    CHECK_FALSE(osd.contains("drc_state"));
    const auto& mission = d.main_transport().published.at(1).payload["data"];
    CHECK(mission["drc_state"] == 2);
    CHECK_FALSE(mission.contains("mode_code"));
    CHECK_FALSE(mission.contains("job_number"));
    CHECK(osd["sub_device"]["device_sn"] == drone_sn);
    CHECK(osd["sub_device"]["device_model_key"] == "0-91-0");
    CHECK(osd["sub_device"]["device_online_status"] == 1);
    CHECK(osd["sub_device"]["device_paired"] == 1);
}
