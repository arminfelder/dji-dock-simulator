#include "control/sim_control.hpp"

#include "log/log.hpp"
#include "protocol/envelope.hpp"
#include "sim/clock.hpp"
#include "sim/dock_sim.hpp"

#include <cmath>
#include <format>
#include <optional>

namespace docksim::control {

namespace {

const auto& logger() {
    static const auto log = log::get("dock_sim.control");
    return log;
}

const Json& object_of(const Json& body) {
    if (!body.is_object()) throw BadRequest("body must be a JSON object");
    return body;
}

std::optional<double> number(const Json& body, const char* key, double lo, double hi) {
    const auto it = body.find(key);
    if (it == body.end() || it->is_null()) return std::nullopt;
    if (!it->is_number()) throw BadRequest(std::format("{} must be a number", key));
    const double v = it->get<double>();
    if (!std::isfinite(v) || v < lo || v > hi)
        throw BadRequest(std::format("{} must be in {} .. {}", key, lo, hi));
    return v;
}

std::optional<int> integer(const Json& body, const char* key, int lo, int hi) {
    const auto it = body.find(key);
    if (it == body.end() || it->is_null()) return std::nullopt;
    if (!it->is_number_integer()) throw BadRequest(std::format("{} must be an integer", key));
    const auto v = it->get<std::int64_t>();
    if (v < lo || v > hi) throw BadRequest(std::format("{} must be in {} .. {}", key, lo, hi));
    return static_cast<int>(v);
}

std::optional<bool> boolean(const Json& body, const char* key) {
    const auto it = body.find(key);
    if (it == body.end() || it->is_null()) return std::nullopt;
    if (!it->is_boolean()) throw BadRequest(std::format("{} must be true or false", key));
    return it->get<bool>();
}

double required(const std::optional<double>& v, const char* key) {
    if (!v) throw BadRequest(std::format("{} is required", key));
    return *v;
}

Json gnss_json(const device::Gnss& g) {
    return {{"fix", g.fix},
            {"quality", g.quality},
            {"gps_number", g.gps_number},
            {"rtk_number", g.rtk_number}};
}

Json alarm_json(const HmsAlarm& a) {
    return {{"code", a.code},          {"level", a.level},          {"module", a.module},
            {"device", a.device},      {"in_the_sky", a.in_the_sky}, {"imminent", a.imminent}};
}

}  // namespace

SimControl::SimControl(sim::DockSim& dock, sim::RealClock* clock, WireLog& wire)
    : dock_(&dock), clock_(clock), wire_(&wire) {}

bool SimControl::aircraft_busy() const {
    return dock_->mission().flying() || dock_->manual().flying();
}

Json SimControl::snapshot() const {
    auto& ctx = dock_->ctx();
    const auto& cfg = ctx.cfg();
    const auto& d = ctx.dock;
    const auto& a = ctx.drone;
    const auto& w = d.weather;
    const auto refusal = dock_->preflight();

    Json mission = {{"flying", dock_->mission().flying()}};
    if (const auto& task = dock_->mission().task()) {
        const auto& p = dock_->mission().progress();
        mission["flight_id"] = task->flight_id;
        mission["status"] = p.status;
        mission["percent"] = p.percent;
        mission["waypoint"] = p.waypoint;
        Json points = Json::array();
        if (task->wayline) {
            for (const auto& wp : task->wayline->waypoints)
                points.push_back({{"latitude", wp.latitude},
                                  {"longitude", wp.longitude},
                                  {"height", wp.height}});
        }
        mission["waypoints"] = std::move(points);
    }

    Json hms = Json::array();
    for (const auto& alarm : cfg.maintenance.hms) {
        auto j = alarm_json(alarm);
        j["source"] = "config";
        hms.push_back(std::move(j));
    }
    for (const auto& alarm : dock_->hms().raised()) {
        auto j = alarm_json(alarm);
        j["source"] = "runtime";
        hms.push_back(std::move(j));
    }

    return {
        {"time", envelope::now_ms()},
        {"time_scale", clock_ != nullptr ? clock_->time_scale() : 1.0},
        {"time_scale_adjustable", clock_ != nullptr},
        {"dock",
         {
             {"sn", cfg.device.dock_sn},
             {"model", cfg.device.dock_model},
             {"online", d.online},
             {"mode_code", d.mode_code},
             {"cover_state", d.cover_state},
             {"putter_state", d.putter_state},
             {"drone_in_dock", d.drone_in_dock != 0},
             {"charge_state", d.drone_charge_state},
             {"charge_percent", d.drone_charge_percent},
             {"flighttask_step_code", d.flighttask_step_code},
             {"drc_state", d.drc_state},
             {"latitude", d.site.latitude},
             {"longitude", d.site.longitude},
             {"altitude", d.site.altitude},
             {"gnss", gnss_json(d.gnss)},
             {"weather",
              {{"noise", w.noise},
               {"wind_speed", w.wind_speed},
               {"wind_direction", w.wind_direction},
               {"rainfall", w.rainfall},
               {"temperature", w.temperature},
               {"humidity", w.humidity}}},
             {"weather_limits",
              {{"max_wind_takeoff", cfg.weather.max_wind_takeoff},
               {"max_rainfall_takeoff", cfg.weather.max_rainfall_takeoff}}},
             {"launch_refusal", refusal ? Json(static_cast<int>(*refusal)) : Json()},
         }},
        {"drone",
         {
             {"sn", cfg.device.drone_sn},
             {"model", cfg.device.drone_model},
             {"mode_code", a.mode_code},
             {"latitude", a.latitude},
             {"longitude", a.longitude},
             {"height", a.height},
             {"attitude_head", a.attitude_head},
             {"course", a.course},
             {"horizontal_speed", a.horizontal_speed},
             {"vertical_speed", a.vertical_speed},
             {"battery", a.battery_level},
             {"home_distance", a.home_distance()},
             {"gimbal_pitch", a.gimbal_pitch},
             {"camera_heading", a.camera_heading()},
             {"zoom_factor", a.zoom_factor},
             {"rth_altitude", a.rth_altitude},
             {"total_flight_time", a.total_flight_time},
             {"total_flight_distance", a.total_flight_distance},
             {"camera_mode", a.camera.camera_mode},
             {"recording", a.camera.recording_state != 0},
             {"gnss", gnss_json(a.gnss)},
         }},
        {"mission", std::move(mission)},
        {"manual",
         {{"flying", dock_->manual().flying()},
          {"phase", static_cast<int>(dock_->manual().phase())}}},
        {"faults",
         {{"emergency_stop", d.emergency_stop_state != 0},
          {"cover_fault", d.cover_fault},
          {"hms", std::move(hms)}}},
    };
}

Json SimControl::move_dock(const Json& body) {
    const auto& b = object_of(body);
    const double lat = required(number(b, "latitude", -90.0, 90.0), "latitude");
    const double lon = required(number(b, "longitude", -180.0, 180.0), "longitude");
    const auto alt = number(b, "altitude", -500.0, 9000.0);
    auto& ctx = dock_->ctx();
    if (ctx.dock.drone_in_dock == 0 || aircraft_busy())
        throw Conflict("the dock moves only while the aircraft is in the dock");
    auto& site = ctx.dock.site;
    site.latitude = lat;
    site.longitude = lon;
    if (alt) site.altitude = *alt;
    ctx.drone.home = site;
    ctx.drone.latitude = lat;
    ctx.drone.longitude = lon;
    logger()->info("dock moved to {:.7f}, {:.7f}, {:.1f} m", lat, lon, site.altitude);
    ctx.spawn(dock_->publish_state(), "control-state");
    return snapshot();
}

Json SimControl::teleport_drone(const Json& body) {
    const auto& b = object_of(body);
    const double lat = required(number(b, "latitude", -90.0, 90.0), "latitude");
    const double lon = required(number(b, "longitude", -180.0, 180.0), "longitude");
    const auto height = number(b, "height", 0.0, 1500.0);
    auto& ctx = dock_->ctx();
    if (ctx.dock.drone_in_dock != 0 || !aircraft_busy())
        throw Conflict("the aircraft moves only while it flies");
    ctx.drone.latitude = lat;
    ctx.drone.longitude = lon;
    if (height) ctx.drone.height = *height;
    logger()->info("aircraft moved to {:.7f}, {:.7f}", lat, lon);
    return snapshot();
}

Json SimControl::set_battery(const Json& body) {
    const double percent =
        required(number(object_of(body), "percent", 0.0, 100.0), "percent");
    auto& ctx = dock_->ctx();
    ctx.drone.battery_level = percent;
    if (ctx.dock.drone_in_dock != 0) ctx.dock.drone_charge_percent = ctx.drone.battery_percent();
    logger()->info("battery set to {:.1f} %", percent);
    return snapshot();
}

Json SimControl::set_weather(const Json& body) {
    const auto& b = object_of(body);
    const auto wind = number(b, "wind_speed", 0.0, 40.0);
    const auto direction = number(b, "wind_direction", 0.0, 360.0);
    const auto rain = integer(b, "rainfall", 0, 3);
    const auto temperature = number(b, "temperature", -40.0, 60.0);
    const auto humidity = integer(b, "humidity", 0, 100);
    auto& ctx = dock_->ctx();
    auto& w = ctx.dock.weather;
    w.noise = false;
    if (wind) w.wind_speed = *wind;
    if (direction) w.wind_direction = *direction;
    if (rain) w.rainfall = *rain;
    if (temperature) w.temperature = *temperature;
    if (humidity) w.humidity = *humidity;
    ctx.drone.wind_speed = w.wind_speed;
    ctx.drone.wind_direction = w.wind_direction;
    logger()->info("weather: wind {:.1f} m/s from {:.0f} deg, rainfall {}", w.wind_speed,
                   w.wind_direction, w.rainfall);
    return snapshot();
}

Json SimControl::reset_weather() {
    auto& ctx = dock_->ctx();
    ctx.dock.weather = {};
    ctx.drone.wind_speed = 0.0;
    ctx.drone.wind_direction = 0.0;
    return snapshot();
}

Json SimControl::set_gnss(const std::string& target, const Json& body) {
    auto& ctx = dock_->ctx();
    device::Gnss* g = nullptr;
    if (target == "dock") g = &ctx.dock.gnss;
    if (target == "drone") g = &ctx.drone.gnss;
    if (g == nullptr) throw BadRequest("target must be dock or drone");
    const auto& b = object_of(body);
    const auto gps = integer(b, "gps_number", 0, 60);
    const auto rtk = integer(b, "rtk_number", 0, 60);
    const auto quality = integer(b, "quality", 0, 5);
    const auto fix = integer(b, "fix", 0, 2);
    if (gps) g->gps_number = *gps;
    if (rtk) g->rtk_number = *rtk;
    if (quality) g->quality = *quality;
    if (fix) g->fix = *fix;
    return snapshot();
}

Json SimControl::set_time_scale(const Json& body) {
    const double scale =
        required(number(object_of(body), "time_scale", 0.1, 100.0), "time_scale");
    if (clock_ == nullptr) throw Conflict("this clock has no time scale");
    clock_->set_time_scale(scale);
    logger()->info("time scale {}", scale);
    return snapshot();
}

Json SimControl::set_faults(const Json& body) {
    const auto& b = object_of(body);
    const auto stop = boolean(b, "emergency_stop");
    const auto cover = boolean(b, "cover_fault");
    auto& d = dock_->ctx().dock;
    if (stop) d.emergency_stop_state = *stop ? 1 : 0;
    if (cover) d.cover_fault = *cover;
    return snapshot();
}

Json SimControl::raise_hms(const Json& body) {
    const auto& b = object_of(body);
    const auto it = b.find("code");
    if (it == b.end() || !it->is_string() || it->get<std::string>().empty())
        throw BadRequest("code must be a string, for example 0x16100083");
    HmsAlarm alarm{.code = it->get<std::string>()};
    if (const auto level = integer(b, "level", 0, 2)) alarm.level = *level;
    if (const auto module = integer(b, "module", 0, 3)) alarm.module = *module;
    if (const auto sky = boolean(b, "in_the_sky")) alarm.in_the_sky = *sky;
    if (const auto imminent = boolean(b, "imminent")) alarm.imminent = *imminent;
    if (const auto dev = b.find("device"); dev != b.end()) {
        if (!dev->is_string() || (*dev != "dock" && *dev != "drone"))
            throw BadRequest("device must be dock or drone");
        alarm.device = dev->get<std::string>();
    }
    dock_->hms().raise(std::move(alarm));
    dock_->ctx().spawn(dock_->hms().send(), "control-hms");
    return snapshot();
}

Json SimControl::clear_hms(const std::string& code) {
    if (!dock_->hms().clear(code)) throw BadRequest(std::format("no runtime alarm {}", code));
    dock_->ctx().spawn(dock_->hms().send(), "control-hms");
    return snapshot();
}

Json SimControl::wire(std::uint64_t since) const {
    Json out = Json::array();
    for (const auto& e : wire_->since(since)) out.push_back(to_json(e));
    return {{"last_seq", wire_->last_seq()}, {"entries", std::move(out)}};
}

}  // namespace docksim::control
