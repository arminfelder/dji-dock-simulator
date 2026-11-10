#include "control/sim_control.hpp"
#include "device/weather.hpp"
#include "flight_support.hpp"
#include "transport/tap.hpp"

#include <asio/co_spawn.hpp>
#include <asio/detached.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <format>

using namespace docksim;
using namespace docksim::flight_test;
using Catch::Approx;

namespace {

const Json mission_prepare = {
    {"flight_id", "f-1"},
    {"task_type", 0},
    {"execute_time", 0},
    {"rth_altitude", 60},
    {"file", {{"url", "https://example.invalid/w.kmz"}, {"fingerprint", "x"}}},
};

struct Console {
    Driver d{manual_config()};
    control::WireLog wire;
    control::SimControl control{d.sim(), nullptr, wire};
};

Json dock_osd_weather(Driver& d) {
    d.osd_tick();
    const auto& out = d.main_transport().published;
    for (auto it = out.rbegin(); it != out.rend(); ++it) {
        if (it->topic == std::format("thing/product/{}/osd", dock_sn) &&
            it->payload["data"].contains("wind_speed"))
            return it->payload["data"];
    }
    FAIL("no dock osd with weather");
    return {};
}

int execute_result(Driver& d) {
    d.service("flighttask_prepare", mission_prepare);
    d.service("flighttask_execute", {{"flight_id", "f-1"}});
    return reply_result(d, "flighttask_execute");
}

}  // namespace

TEST_CASE("weather: calm air and noise mode change nothing") {
    device::Weather w;
    CHECK(device::drain_factor(w) == 1.0);
    CHECK(device::ground_speed(w, 90.0, 10.0) == 10.0);
    CHECK_FALSE(device::weather_block(w, {}));
    w.noise = false;
    CHECK(device::drain_factor(w) == 1.0);
    CHECK(device::ground_speed(w, 90.0, 10.0) == 10.0);
}

TEST_CASE("weather: headwind slows the aircraft, tailwind does not speed it up") {
    const device::Weather w{.noise = false, .wind_speed = 5.0, .wind_direction = 0.0};
    CHECK(device::ground_speed(w, 0.0, 10.0) == Approx(5.0));
    CHECK(device::ground_speed(w, 180.0, 10.0) == Approx(10.0));
    CHECK(device::ground_speed({.noise = false, .wind_speed = 30.0}, 0.0, 10.0) == Approx(3.0));
    CHECK(device::drain_factor({.noise = false, .wind_speed = 12.0}) == Approx(1.5));
}

TEST_CASE("weather: launch limits give the DJI codes") {
    const WeatherConfig limits;
    CHECK(device::weather_block({.noise = false, .wind_speed = 12.0}, limits) ==
          code::Result::wind_too_high);
    CHECK_FALSE(device::weather_block({.noise = false, .wind_speed = 11.9}, limits));
    CHECK(device::weather_block({.noise = false, .rainfall = 3}, limits) ==
          code::Result::heavy_rain);
    CHECK_FALSE(device::weather_block({.noise = false, .rainfall = 2}, limits));
}

TEST_CASE("weather: drift only without satellite fix") {
    const device::Weather w{.noise = false, .wind_speed = 4.0, .wind_direction = 270.0};
    CHECK_FALSE(device::drift(w, {.gps_number = 18}, 1.0));
    const auto d = device::drift(w, {.gps_number = 3}, 2.0);
    REQUIRE(d);
    // Wind from the west pushes east.
    CHECK(d->east == Approx(8.0));
    CHECK(d->north == Approx(0.0).margin(1e-9));
}

TEST_CASE("control: set weather shows in the dock osd") {
    Console c;
    c.d.connect();
    c.control.set_weather({{"wind_speed", 7.26}, {"rainfall", 2}, {"humidity", 91}});
    const auto osd = dock_osd_weather(c.d);
    CHECK(osd["wind_speed"] == 7.3);
    CHECK(osd["rainfall"] == 2);
    CHECK(osd["humidity"] == 91);
}

TEST_CASE("control: strong wind refuses the mission, calm weather lets it go") {
    Console c;
    c.d.connect();
    c.control.set_weather({{"wind_speed", 15.0}});
    CHECK(execute_result(c.d) == 514135);
    CHECK(c.control.snapshot()["dock"]["launch_refusal"] == 514135);
    c.control.reset_weather();
    CHECK(execute_result(c.d) == 0);
    c.d.wait_mission();
}

TEST_CASE("control: heavy rain, emergency stop, low battery and cover fault refuse a launch") {
    Console c;
    c.d.connect();
    c.control.set_weather({{"rainfall", 3}});
    CHECK(execute_result(c.d) == 514134);
    c.control.reset_weather();
    c.control.set_faults({{"emergency_stop", true}});
    CHECK(execute_result(c.d) == 514121);
    c.control.set_faults({{"emergency_stop", false}, {"cover_fault", true}});
    CHECK(execute_result(c.d) == 514107);
    c.control.set_faults({{"cover_fault", false}});
    c.control.set_battery({{"percent", 20}});
    CHECK(execute_result(c.d) == 514103);
}

TEST_CASE("control: strong wind refuses takeoff_to_point") {
    Console c;
    c.d.connect();
    c.control.set_weather({{"wind_speed", 13.0}});
    c.d.service("takeoff_to_point", takeoff_data(c.d));
    CHECK(reply_result(c.d, "takeoff_to_point") == 514135);
}

TEST_CASE("control: without satellite fix the hovering aircraft drifts with the wind") {
    Console c;
    hovering(c.d);
    const auto& drone = c.d.sim().ctx().drone;
    const double before = drone.longitude;
    c.control.set_weather({{"wind_speed", 5.0}, {"wind_direction", 270.0}});
    c.d.advance(4.0);
    CHECK(drone.longitude == before);
    c.control.set_gnss("drone", {{"gps_number", 2}});
    c.d.advance(4.0);
    const double metres_east = (drone.longitude - before) * metres_per_degree *
                               std::cos(drone.latitude * 3.141592653589793 / 180.0);
    CHECK(metres_east == Approx(20.0).margin(1.0));
}

TEST_CASE("control: move the dock only while the aircraft is in it") {
    Console c;
    c.d.connect();
    c.control.move_dock({{"latitude", 48.2}, {"longitude", 16.37}, {"altitude", 170.0}});
    const auto osd = dock_osd_weather(c.d);
    CHECK(osd["latitude"] == 48.2);
    CHECK(osd["longitude"] == 16.37);
    CHECK(osd["height"] == 170.0);
    CHECK(c.d.sim().ctx().drone.home_distance() == Approx(0.0).margin(1e-6));
    CHECK(drone_osd(c.d)["home_latitude"] == 48.2);

    Console flying;
    hovering(flying.d);
    CHECK_THROWS_AS(flying.control.move_dock({{"latitude", 48.0}, {"longitude", 16.0}}),
                    control::Conflict);
}

TEST_CASE("control: teleport the aircraft only in flight") {
    Console c;
    c.d.connect();
    CHECK_THROWS_AS(c.control.teleport_drone({{"latitude", 47.3}, {"longitude", 11.4}}),
                    control::Conflict);
    hovering(c.d);
    c.control.teleport_drone({{"latitude", 47.27}, {"longitude", 11.41}, {"height", 80.0}});
    const auto& drone = c.d.sim().ctx().drone;
    CHECK(drone.latitude == 47.27);
    CHECK(drone.height == 80.0);
}

TEST_CASE("control: battery level in the dock sets the charge percent") {
    Console c;
    c.d.connect();
    c.control.set_battery({{"percent", 42.0}});
    CHECK(c.d.sim().ctx().dock.drone_charge_percent == 42);
    CHECK(c.control.snapshot()["drone"]["battery"] == 42.0);
}

TEST_CASE("control: low battery in flight starts return to home") {
    Console c;
    hovering(c.d);
    c.control.set_battery({{"percent", 25.0}});
    c.d.advance(1.0);
    CHECK(c.d.sim().manual().phase() == flight::Phase::rth);
}

TEST_CASE("control: a raised hms alarm goes out in the hms event, a clear removes it") {
    Console c;
    c.d.connect();
    c.control.raise_hms({{"code", "0x16100083"}, {"level", 2}, {"device", "drone"}});
    c.d.advance(0.1);
    auto hms = events(c.d, "hms");
    REQUIRE_FALSE(hms.empty());
    CHECK(hms.back()["list"][0]["code"] == "0x16100083");
    c.control.clear_hms("0x16100083");
    c.d.advance(0.1);
    hms = events(c.d, "hms");
    CHECK(hms.back()["list"].empty());
    CHECK_THROWS_AS(c.control.clear_hms("0x1"), control::BadRequest);
}

TEST_CASE("control: bad values are refused and change nothing") {
    Console c;
    CHECK_THROWS_AS(c.control.set_weather({{"wind_speed", -1}}), control::BadRequest);
    CHECK_THROWS_AS(c.control.set_weather({{"rainfall", 1.5}}), control::BadRequest);
    CHECK_THROWS_AS(c.control.set_battery({{"percent", "x"}}), control::BadRequest);
    CHECK_THROWS_AS(c.control.set_battery(Json::array()), control::BadRequest);
    CHECK_THROWS_AS(c.control.move_dock({{"latitude", 91}, {"longitude", 0}}),
                    control::BadRequest);
    CHECK_THROWS_AS(c.control.set_gnss("moon", Json::object()), control::BadRequest);
    CHECK_THROWS_AS(c.control.set_time_scale({{"time_scale", 2}}), control::Conflict);
    CHECK(c.d.sim().ctx().dock.weather.noise);
}

TEST_CASE("control: the wire log keeps the last messages in order") {
    control::WireLog log(3);
    for (int i = 0; i < 5; ++i)
        log.add(transport::Direction::out, "t", {{"method", std::format("m{}", i)}});
    const auto all = log.since(0);
    REQUIRE(all.size() == 3);
    CHECK(all.front().method == "m2");
    CHECK(all.back().seq == 5);
    CHECK(log.since(4).size() == 1);
}

TEST_CASE("control: the tapped transport logs both directions") {
    asio::io_context io;
    golden::FakeTransport inner(io.get_executor(), 1, "c");
    control::WireLog log;
    transport::TappedTransport tap(inner, log.sink());
    bool handled = false;
    tap.on("/services", [&](std::string, Json) -> asio::awaitable<void> {
        handled = true;
        co_return;
    });
    asio::co_spawn(io, inner.inject("thing/x/services", {{"method", "ping"}}), asio::detached);
    asio::co_spawn(io, tap.publish("thing/x/services_reply", {{"method", "ping"}}, std::nullopt),
                   asio::detached);
    io.run();
    CHECK(handled);
    const auto entries = log.since(0);
    REQUIRE(entries.size() == 2);
    CHECK(entries[0].direction == transport::Direction::in);
    CHECK(entries[1].topic == "thing/x/services_reply");
}
