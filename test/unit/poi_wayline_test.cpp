// POI orbit and in-flight wayline of the manual flight, driven on the virtual
// clock of the golden harness.

#include "device/geo.hpp"
#include "flight_support.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

using namespace docksim;
using namespace docksim::golden;
using namespace docksim::flight_test;
using Catch::Approx;
using code::DroneMode;
using flight::Phase;

namespace {

// A point this many metres north of the takeoff target, on the ground.
Json poi_north_of_target(Driver& d, double metres) {
    auto point = point_north(d, 60.0 + metres, 0.0);
    return {{"latitude", point["latitude"]},
            {"longitude", point["longitude"]},
            {"height", home(d).altitude}};
}

double metres_to(Driver& d, const Json& point) {
    const auto& drone = d.sim().ctx().drone;
    return units::in_metres(geo::haversine(
        geo::point(drone.latitude, drone.longitude),
        geo::point(point["latitude"].get<double>(), point["longitude"].get<double>())));
}

Json deliver(const std::string& id = "ifw-1") {
    return {
        {"in_flight_wayline_id", id},
        {"file", {{"url", "https://example.invalid/inflight.kmz"}, {"fingerprint", "x"}}},
        {"rth_altitude", 80},
        {"rth_mode", 1},
        {"out_of_control_action", 0},
        {"exit_wayline_when_rc_lost", 0},
        {"wayline_precision_type", 1},
    };
}

std::vector<int> wayline_statuses(Driver& d) {
    std::vector<int> out;
    for (const auto& e : events(d, "in_flight_wayline_progress"))
        out.push_back(e["status"].get<int>());
    return out;
}

// The Dock 2 gateway does not route POI. Dock 3 keeps it, unverified on firmware.
Config poi_config() {
    Config cfg = manual_config();
    cfg.device.dock_model = "dock3";
    return cfg;
}

void advance_until_hover(Driver& d, double limit_s) {
    for (double t = 0; t < limit_s && d.sim().manual().phase() != Phase::hover; t += 0.5)
        d.advance(0.5);
}

}  // namespace

TEST_CASE("poi: the Dock 2 gateway drops the POI services", "[poi]") {
    Driver d(manual_config());
    d.connect();
    d.service("takeoff_to_point", takeoff_data(d));
    d.advance(30.0);
    d.clear();
    for (const auto* method : {"poi_mode_enter", "poi_circle_speed_set", "poi_mode_exit"}) {
        d.service(method, poi_north_of_target(d, 20));
        d.advance(0.5);
        INFO(method);
        CHECK(messages(d, "/services_reply", method).empty());
    }
    CHECK(events(d, "poi_status_notify").empty());
    CHECK(d.sim().manual().phase() != Phase::poi);
}

TEST_CASE("poi: refused on the ground and outside manual mode", "[poi]") {
    Driver d(poi_config());
    d.connect();
    d.clear();
    d.service("poi_mode_enter", poi_north_of_target(d, 20));
    CHECK(reply_result(d, "poi_mode_enter") == 314000);
    auto notes = events(d, "poi_status_notify");
    REQUIRE(notes.size() == 1);
    CHECK(notes[0]["status"] == "failed");
    CHECK(notes[0]["reason"] == 5);
    d.service("poi_mode_enter", {{"latitude", 47.0}});
    CHECK(reply_result(d, "poi_mode_enter") == 514171);

    d.service("takeoff_to_point", takeoff_data(d));
    d.advance(30.0);
    d.service("fly_to_point", {{"fly_to_id", "f-1"},
                               {"max_speed", 2},
                               {"points", Json::array({point_north(d, 200, 40)})}});
    d.advance(1.0);
    d.clear();
    d.service("poi_mode_enter", poi_north_of_target(d, 20));
    CHECK(reply_result(d, "poi_mode_enter") == 314000);
    notes = events(d, "poi_status_notify");
    REQUIRE(notes.size() == 1);
    CHECK(notes[0]["reason"] == 6);
    CHECK(d.sim().manual().phase() == Phase::fly_to);
}

TEST_CASE("poi: the aircraft orbits the point and faces it", "[poi]") {
    Driver d(poi_config());
    hovering(d);
    auto& drone = d.sim().ctx().drone;
    const auto centre = poi_north_of_target(d, 20);
    d.clear();
    d.service("poi_mode_enter", centre);
    CHECK(reply_result(d, "poi_mode_enter") == 0);
    d.advance(0.5);
    CHECK(drone.mode_code == DroneMode::live_flight_control);
    CHECK(d.sim().manual().poi_radius() == Approx(20.0).margin(0.1));
    auto notes = events(d, "poi_status_notify");
    REQUIRE(notes.size() == 1);
    CHECK(notes[0]["status"] == "in_progress");
    CHECK(notes[0]["reason"] == 0);
    CHECK(notes[0]["circle_radius"].get<double>() == Approx(20.0).margin(0.1));
    CHECK(notes[0]["circle_speed"].get<double>() == Approx(3.0));
    CHECK(notes[0]["max_circle_speed"].get<double>() == Approx(10.0));

    const double lon0 = drone.longitude;
    d.advance(10.0);
    // 3 m/s for about 10 s on a 20 m circle: about 86 degrees clockwise from south.
    CHECK(metres_to(d, centre) == Approx(20.0).margin(0.3));
    CHECK(drone.longitude < lon0);
    CHECK(drone.horizontal_speed == Approx(3.0).margin(0.05));
    CHECK(drone.height == Approx(40.0).margin(0.3));
    const double bearing = units::in_degrees(geo::bearing(
        geo::point(drone.latitude, drone.longitude),
        geo::point(centre["latitude"].get<double>(), centre["longitude"].get<double>())));
    CHECK(geo::signed_angle(drone.camera_heading() - bearing) == Approx(0.0).margin(1.0));
    CHECK(drone.gimbal_pitch == Approx(-63.4).margin(1.0));
    CHECK(events(d, "poi_status_notify").size() == 3);

    d.service("poi_circle_speed_set", {{"circle_speed", 5}});
    CHECK(reply_result(d, "poi_circle_speed_set") == 0);
    d.advance(0.5);
    notes = events(d, "poi_status_notify");
    CHECK(notes.back()["circle_speed"].get<double>() == Approx(5.0));
    CHECK(drone.horizontal_speed == Approx(5.0).margin(0.05));
    d.service("poi_circle_speed_set", {{"circle_speed", -25}});
    CHECK(d.sim().manual().poi_circle_speed() == Approx(-10.0));

    d.clear();
    d.service("poi_mode_exit", Json::object());
    CHECK(reply_result(d, "poi_mode_exit") == 0);
    d.advance(0.5);
    notes = events(d, "poi_status_notify");
    REQUIRE(notes.size() == 1);
    CHECK(notes[0]["status"] == "ok");
    CHECK(notes[0]["reason"] == 0);
    CHECK(drone.mode_code == DroneMode::manual);
    CHECK(d.sim().manual().phase() == Phase::hover);
    const double lat = drone.latitude;
    d.advance(2.0);
    CHECK(drone.latitude == lat);
}

TEST_CASE("poi: a centre under the aircraft gives the minimum radius", "[poi]") {
    Driver d(poi_config());
    hovering(d);
    const auto centre = poi_north_of_target(d, 0);
    d.service("poi_mode_enter", centre);
    CHECK(d.sim().manual().poi_radius() == Approx(flight::poi_min_radius_m));
    d.advance(5.0);
    CHECK(metres_to(d, centre) == Approx(flight::poi_min_radius_m).margin(0.3));
}

TEST_CASE("poi: other commands end the orbit", "[poi]") {
    Driver d(poi_config());
    hovering(d);
    auto& drone = d.sim().ctx().drone;
    d.service("poi_mode_enter", poi_north_of_target(d, 20));
    d.advance(2.0);
    d.clear();

    SECTION("return to home") {
        d.service("return_home", Json::object());
        CHECK(reply_result(d, "return_home") == 0);
        d.advance(0.5);
        CHECK(drone.mode_code == DroneMode::auto_rth);
        CHECK(d.sim().manual().phase() == Phase::rth);
    }
    SECTION("fly to") {
        d.service("fly_to_point", {{"fly_to_id", "f-1"},
                                   {"max_speed", 5},
                                   {"points", Json::array({point_north(d, 100, 40)})}});
        CHECK(reply_result(d, "fly_to_point") == 0);
        d.advance(0.5);
        CHECK(d.sim().manual().phase() == Phase::fly_to);
    }
    SECTION("emergency stop") {
        d.service("drc_mode_enter", drc_enter());
        d.drc_inject(drc_down, {{"method", "drone_emergency_stop"}, {"data", ""}, {"seq", "s"}});
        d.advance(0.5);
        CHECK(drone.mode_code == DroneMode::manual);
        CHECK(d.sim().manual().phase() == Phase::hover);
    }
    SECTION("low battery") {
        drone.battery_level = 30.4;
        d.advance(1.0);
        CHECK(drone.mode_code == DroneMode::auto_rth);
    }
    const auto notes = events(d, "poi_status_notify");
    REQUIRE(notes.size() == 1);
    CHECK(notes[0]["status"] == "failed");
    CHECK(notes[0]["reason"] == 6);
    d.clear();
    d.service("poi_mode_exit", Json::object());
    CHECK(reply_result(d, "poi_mode_exit") == 0);
    CHECK(events(d, "poi_status_notify").empty());
}

TEST_CASE("in-flight wayline: refused on the ground and without an id", "[inflight]") {
    Driver d(manual_config());
    d.connect();
    d.clear();
    d.service("in_flight_wayline_deliver", deliver());
    CHECK(reply_result(d, "in_flight_wayline_deliver") == 314000);
    d.service("in_flight_wayline_deliver", {{"file", {{"url", "https://example.invalid/w.kmz"}}}});
    CHECK(reply_result(d, "in_flight_wayline_deliver") == 514171);
    d.service("in_flight_wayline_stop", {{"in_flight_wayline_id", "ifw-1"}});
    CHECK(reply_result(d, "in_flight_wayline_stop") == 319020);
    d.service("in_flight_wayline_cancel", Json::object());
    CHECK(reply_result(d, "in_flight_wayline_cancel") == 321259);
    CHECK(events(d, "in_flight_wayline_progress").empty());
}

TEST_CASE("in-flight wayline: the aircraft flies the route and hovers at its end", "[inflight]") {
    Driver d(manual_config());
    hovering(d);
    auto& drone = d.sim().ctx().drone;
    d.clear();
    d.service("in_flight_wayline_deliver", deliver());
    CHECK(reply_result(d, "in_flight_wayline_deliver") == 0);
    d.service("in_flight_wayline_deliver", deliver("ifw-2"));
    CHECK(reply_result(d, "in_flight_wayline_deliver") == 321257);
    d.advance(1.0);
    CHECK(drone.mode_code == DroneMode::wayline);
    CHECK(d.sim().manual().phase() == Phase::wayline);
    // fly_to and POI need manual mode.
    d.service("fly_to_point",
              {{"fly_to_id", "f"}, {"points", Json::array({point_north(d, 10, 30)})}});
    CHECK(reply_result(d, "fly_to_point") == 319022);

    advance_until_hover(d, 120.0);
    CHECK(d.sim().manual().phase() == Phase::hover);
    CHECK(drone.mode_code == DroneMode::manual);
    CHECK(d.sim().manual().airborne());
    CHECK(drone.rth_altitude == Approx(80.0));
    CHECK(drone.height == Approx(45.0).margin(0.3));
    CHECK(drone.latitude == Approx(47.264).margin(1e-5));
    CHECK(drone.longitude == Approx(11.402).margin(1e-5));

    CHECK(wayline_statuses(d) == std::vector<int>{1, 2, 3, 3, 3, 6});
    const auto progress = events(d, "in_flight_wayline_progress");
    for (const auto& e : progress) {
        CHECK(e["in_flight_wayline_id"] == "ifw-1");
        CHECK(e["result"] == 0);
    }
    CHECK(progress[2]["progress"]["percent"] == 33);
    CHECK(progress[3]["progress"]["percent"] == 66);
    CHECK(progress[4]["progress"]["percent"] == 100);
    CHECK(progress[4]["way_point_index"] == 2);
    CHECK(progress[5]["progress"]["percent"] == 100);

    // The two photo actions wait for the landing with the media of the flight.
    REQUIRE(d.sim().uploads().held().contains("takeoff-1"));
    const auto& shots = d.sim().uploads().held().at("takeoff-1").shots;
    REQUIRE(shots.size() == 2);
    CHECK(shots[0].waypoint_index == 0);
    CHECK(shots[1].waypoint_index == 2);
    CHECK(drone.camera_photos == 2);

    // The aircraft takes a second route after the first.
    d.service("in_flight_wayline_deliver", deliver("ifw-2"));
    CHECK(reply_result(d, "in_flight_wayline_deliver") == 0);
}

TEST_CASE("in-flight wayline: pause and recover", "[inflight]") {
    Driver d(manual_config());
    hovering(d);
    auto& drone = d.sim().ctx().drone;
    d.service("in_flight_wayline_deliver", deliver());
    d.advance(1.5);
    d.clear();
    d.service("in_flight_wayline_recover", {{"in_flight_wayline_id", "ifw-1"}});
    CHECK(reply_result(d, "in_flight_wayline_recover") == 319042);
    d.service("in_flight_wayline_stop", {{"in_flight_wayline_id", "other"}});
    CHECK(reply_result(d, "in_flight_wayline_stop") == 514171);
    d.service("in_flight_wayline_stop", {{"in_flight_wayline_id", "ifw-1"}});
    CHECK(reply_result(d, "in_flight_wayline_stop") == 0);
    d.advance(0.5);
    CHECK(wayline_statuses(d) == std::vector<int>{4});
    CHECK(drone.mode_code == DroneMode::wayline);
    const double lat = drone.latitude;
    const double lon = drone.longitude;
    d.advance(3.0);
    CHECK(drone.latitude == lat);
    CHECK(drone.longitude == lon);
    CHECK(drone.horizontal_speed == 0.0);

    d.service("in_flight_wayline_recover", {{"in_flight_wayline_id", "ifw-1"}});
    CHECK(reply_result(d, "in_flight_wayline_recover") == 0);
    d.advance(0.5);
    CHECK(wayline_statuses(d) == std::vector<int>{4, 3});
    CHECK(drone.latitude != lat);
    advance_until_hover(d, 120.0);
    CHECK(wayline_statuses(d).back() == 6);
}

TEST_CASE("in-flight wayline: cancel and return to home end the route", "[inflight]") {
    Driver d(manual_config());
    hovering(d);
    auto& drone = d.sim().ctx().drone;
    d.service("in_flight_wayline_deliver", deliver());
    d.advance(1.5);
    d.clear();

    SECTION("cancel hovers in manual mode") {
        d.service("in_flight_wayline_cancel", Json::object());
        CHECK(reply_result(d, "in_flight_wayline_cancel") == 0);
        d.advance(0.5);
        CHECK(drone.mode_code == DroneMode::manual);
        CHECK(d.sim().manual().phase() == Phase::hover);
        d.service("in_flight_wayline_cancel", Json::object());
        CHECK(reply_result(d, "in_flight_wayline_cancel") == 321259);
    }
    SECTION("a paused route can be cancelled") {
        d.service("in_flight_wayline_stop", {{"in_flight_wayline_id", "ifw-1"}});
        d.advance(0.5);
        d.service("in_flight_wayline_cancel", Json::object());
        d.advance(0.5);
        CHECK(d.sim().manual().phase() == Phase::hover);
    }
    SECTION("return to home") {
        d.service("return_home", Json::object());
        d.advance(0.5);
        CHECK(drone.mode_code == DroneMode::auto_rth);
    }
    SECTION("a return to home before the route starts") {
        d.service("in_flight_wayline_cancel", Json::object());
        d.advance(0.5);
        d.clear();
        d.service("in_flight_wayline_deliver", deliver("ifw-2"));
        CHECK(reply_result(d, "in_flight_wayline_deliver") == 0);
        d.service("return_home", Json::object());
        d.advance(0.5);
        const auto progress = events(d, "in_flight_wayline_progress");
        REQUIRE(progress.size() == 1);
        CHECK(progress[0]["in_flight_wayline_id"] == "ifw-2");
    }
    const auto statuses = wayline_statuses(d);
    REQUIRE_FALSE(statuses.empty());
    CHECK(statuses.back() == 5);
}
