#pragma once

// Helpers of the manual-flight tests: a short-tick config, takeoff and DRC
// requests, and filters over the published messages.

#include "flight/manual.hpp"
#include "golden/harness.hpp"

#include <catch2/catch_test_macros.hpp>

#include <format>
#include <string>
#include <string_view>
#include <vector>

namespace docksim::flight_test {

using namespace docksim::golden;

// Metres per degree of latitude, near enough for test targets.
inline constexpr double metres_per_degree = 111'195.0;

inline Config manual_config() {
    Config cfg = golden_config();
    cfg.flight.tick = 0.1;
    cfg.flight.prepare_seconds = 2.0;
    return cfg;
}

inline const DeviceConfig& home(Driver& d) {
    return d.sim().ctx().cfg().device;
}

inline Json point_north(Driver& d, double metres, double height_above_dock) {
    return {{"latitude", home(d).latitude + metres / metres_per_degree},
            {"longitude", home(d).longitude},
            {"height", home(d).altitude + height_above_dock}};
}

inline Json takeoff_data(Driver& d) {
    const auto target = point_north(d, 60.0, 40.0);
    return {
        {"flight_id", "takeoff-1"},
        {"target_latitude", target["latitude"]},
        {"target_longitude", target["longitude"]},
        {"target_height", target["height"]},
        {"security_takeoff_height", 30},
        {"rth_altitude", 50},
        {"rth_mode", 1},
        {"max_speed", 10},
        {"commander_mode_lost_action", 1},
        {"commander_flight_mode", 1},
        {"commander_flight_height", 40},
        {"flight_safety_advance_check", false},
    };
}

inline Json drc_enter(const std::string& password = "jwt") {
    return {
        {"mqtt_broker",
         {
             {"address", "wss://drc.example.invalid:443/mqtt"},
             {"username", "drc-user"},
             {"password", password},
             {"client_id", std::format("org-{}", dock_sn)},
             {"enable_tls", false},
         }},
        {"osd_frequency", 10},
        {"hsi_frequency", 1},
    };
}

inline const std::string drc_down = std::format("thing/product/{}/drc/down", dock_sn);

// data of every main-topic message with this method, in order.
inline std::vector<Json> messages(Driver& d, std::string_view suffix, std::string_view method) {
    std::vector<Json> out;
    for (const auto& p : d.main_transport().published) {
        if (p.topic.ends_with(suffix) && p.payload.value("method", "") == method)
            out.push_back(p.payload["data"]);
    }
    return out;
}

inline std::vector<Json> events(Driver& d, std::string_view method) {
    return messages(d, "/events", method);
}

inline int reply_result(Driver& d, std::string_view method) {
    const auto replies = messages(d, "/services_reply", method);
    REQUIRE_FALSE(replies.empty());
    return replies.back()["result"].get<int>();
}

inline std::vector<Json> drc_up(Driver& d, std::string_view method) {
    std::vector<Json> out;
    for (const auto& p : d.drc_session().published) {
        if (p.payload.value("method", "") == method) out.push_back(p.payload);
    }
    return out;
}

inline std::vector<std::string> statuses(const std::vector<Json>& list) {
    std::vector<std::string> out;
    out.reserve(list.size());
    for (const auto& e : list) out.push_back(e["status"].get<std::string>());
    return out;
}

inline void stick(Driver& d, double x, double y, double h, double w, int seq = 0) {
    d.drc_inject(drc_down, {{"method", "drone_control"},
                            {"data", {{"seq", seq}, {"x", x}, {"y", y}, {"h", h}, {"w", w}}}});
}

inline void heart_beat(Driver& d) {
    d.drc_inject(drc_down, {{"method", "heart_beat"}, {"data", {{"timestamp", 1}}}, {"seq", 1}});
}

// Online, airborne and hovering at the takeoff target.
inline Driver& hovering(Driver& d) {
    d.connect();
    d.service("takeoff_to_point", takeoff_data(d));
    d.advance(30.0);
    REQUIRE(d.sim().manual().phase() == flight::Phase::hover);
    return d;
}

inline double metres_from_home(Driver& d) {
    return d.sim().ctx().drone.home_distance();
}

// data of the last aircraft osd. Sends one osd round first.
inline Json drone_osd(Driver& d) {
    d.osd_tick();
    const auto& wire = d.main_transport().published;
    for (auto it = wire.rbegin(); it != wire.rend(); ++it) {
        if (it->topic == std::format("thing/product/{}/osd", drone_sn)) return it->payload["data"];
    }
    FAIL("no aircraft osd");
    return {};
}

// data of every drc/up message with this method, in order.
inline std::vector<Json> drc_data(Driver& d, std::string_view method) {
    std::vector<Json> out;
    for (const auto& p : d.drc_session().published) {
        if (p.payload.value("method", "") == method) out.push_back(p.payload["data"]);
    }
    return out;
}

}  // namespace docksim::flight_test
