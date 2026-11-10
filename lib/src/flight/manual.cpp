#include "flight/manual.hpp"

#include "device/geo.hpp"
#include "device/weather.hpp"
#include "device/gimbal.hpp"
#include "log/log.hpp"
#include "media/uploads.hpp"
#include "protocol/envelope.hpp"
#include "protocol/payloads.hpp"
#include "protocol/topics.hpp"
#include "services/registry.hpp"
#include "sim/context.hpp"
#include "util/numeric.hpp"
#include "wayline/engine.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <utility>

namespace docksim::flight {

namespace {

using code::DockMode;
using code::DroneMode;
using code::FlighttaskStatus;
using code::FlyStatus;
using code::InFlightStatus;
using code::PoiReason;
using code::Result;

const auto& logger() {
    static const auto log = log::get("dock_sim.manual");
    return log;
}

// Estimated battery percent for a flight of this time. Rounded up.
int battery_for(double seconds) {
    return std::clamp(static_cast<int>(std::ceil(seconds / device::drain_seconds_per_percent)), 0,
                      100);
}

}  // namespace

ManualFlight::ManualFlight(sim::DockContext& ctx, const wayline::MissionEngine& mission,
                           wayline::Download download, media::MediaUploads& uploads)
    : ctx_(ctx),
      mission_(mission),
      download_(std::move(download)),
      uploads_(uploads),
      flights_(ctx.executor(), [](const std::string& name, std::exception_ptr e) {
          try {
              std::rethrow_exception(std::move(e));
          } catch (const std::exception& error) {
              logger()->error("{} failed: {}", name, error.what());
          }
      }) {}

void ManualFlight::register_services(services::Registry& registry,
                                     wayline::MissionEngine& mission) {
    using services::Reply;
    const auto grab = [this](const Json&) -> asio::awaitable<Reply> {
        // The cloud skips the grab while control_source is "A", so the group goes out once more.
        ctx_.spawn(publish_control_source(), "control-source");
        co_return services::ok();
    };
    registry.add("flight_authority_grab", grab);
    registry.add("payload_authority_grab", grab);
    registry.add("takeoff_to_point", [this](const Json& data) -> asio::awaitable<Reply> {
        co_return services::result(takeoff(data));
    });
    registry.add("fly_to_point", [this](const Json& data) -> asio::awaitable<Reply> {
        co_return services::result(fly_to(data));
    });
    registry.add("fly_to_point_update", [this](const Json& data) -> asio::awaitable<Reply> {
        co_return services::result(fly_to_update(data));
    });
    registry.add("fly_to_point_stop", [this](const Json&) -> asio::awaitable<Reply> {
        co_return services::result(fly_to_stop());
    });
    registry.add("return_home", [this, &mission](const Json&) -> asio::awaitable<Reply> {
        if (flying()) co_return services::result(return_home());
        std::ignore = mission.cancel();
        co_return services::ok();
    });
    registry.add("return_home_cancel", [this](const Json&) -> asio::awaitable<Reply> {
        if (flying()) co_return services::result(return_home_cancel());
        co_return services::ok();
    });
    registry.add("poi_mode_enter", [this](const Json& data) -> asio::awaitable<Reply> {
        co_return services::result(poi_enter(data));
    });
    registry.add("poi_mode_exit", [this](const Json&) -> asio::awaitable<Reply> {
        co_return services::result(poi_exit());
    });
    registry.add("poi_circle_speed_set", [this](const Json& data) -> asio::awaitable<Reply> {
        co_return services::result(poi_speed(data));
    });
    registry.add("in_flight_wayline_deliver", [this](const Json& data) -> asio::awaitable<Reply> {
        co_return services::result(wayline_deliver(data));
    });
    registry.add("in_flight_wayline_stop", [this](const Json& data) -> asio::awaitable<Reply> {
        co_return services::result(wayline_pause(data));
    });
    registry.add("in_flight_wayline_recover", [this](const Json& data) -> asio::awaitable<Reply> {
        co_return services::result(wayline_resume(data));
    });
    registry.add("in_flight_wayline_cancel", [this](const Json&) -> asio::awaitable<Reply> {
        co_return services::result(wayline_cancel());
    });
}

Result ManualFlight::takeoff(const Json& data) {
    const auto& cfg = ctx_.cfg();
    if (ctx_.dock.mode_code == DockMode::remote_debug) return Result::remote_debug_active;
    if (ctx_.dock.mode_code != DockMode::idle || mission_.flying() || flying() ||
        ctx_.dock.drone_in_dock == 0) {
        return Result::dock_busy;
    }
    if (preflight_) {
        if (const auto refused = preflight_()) return *refused;
    }
    const double latitude = services::number(data, "target_latitude");
    const double longitude = services::number(data, "target_longitude");
    const double height = services::number(data, "target_height");
    const double speed = services::number(data, "max_speed", cfg.flight.speed);
    const bool safety_check = services::flag(data, "flight_safety_advance_check", false);
    if (safety_check && forbidden_ && forbidden_(latitude, longitude))
        return Result::target_restricted;
    safety_check_ = safety_check;
    security_height_ = std::max(min_height_m, services::number(data, "security_takeoff_height",
                                                               cfg.flight.takeoff_altitude));
    ctx_.drone.rth_altitude = services::number(data, "rth_altitude", ctx_.drone.rth_altitude);
    target_ = {
        .latitude = latitude,
        .longitude = longitude,
        .height = std::max(min_height_m, height - ctx_.drone.home.altitude),
        .speed = speed > 0 ? speed : cfg.flight.speed,
    };
    flight_id_ = services::text(data, "flight_id", "");
    const auto now = std::chrono::floor<std::chrono::seconds>(ctx_.clock().now());
    track_id_ = std::format("track-{}", now.time_since_epoch().count());
    fly_to_id_.clear();
    poi_ = {};
    inflight_ = {};
    low_battery_done_ = false;
    airborne_ = false;
    sticks_until_ = 0.0;
    last_progress_ = -std::numeric_limits<double>::infinity();
    phase_ = Phase::prepare;
    flight_ = flights_.spawn(run(), std::format("manual-flight-{}", flight_id_));
    return Result::ok;
}

ManualFlight::Target ManualFlight::parse_point(const Json& data, double speed) const {
    const auto it = data.find("points");
    if (it == data.end() || !it->is_array() || it->empty() || !(*it)[0].is_object())
        throw services::BadParameter("points must hold one target point");
    const Json& point = (*it)[0];
    return {
        .latitude = services::number(point, "latitude"),
        .longitude = services::number(point, "longitude"),
        .height =
            std::max(min_height_m, services::number(point, "height") - ctx_.drone.home.altitude),
        .speed = speed,
    };
}

Result ManualFlight::fly_to(const Json& data) {
    const bool from_hover = phase_ == Phase::hover && manual_mode();
    if (!airborne() || (!from_hover && phase_ != Phase::poi)) return Result::fly_to_failed;
    const double speed = services::number(data, "max_speed", fly_to_default_speed);
    const auto target = parse_point(data, speed > 0 ? speed : fly_to_default_speed);
    if (restricted(target.latitude, target.longitude)) return Result::target_restricted;
    target_ = target;
    fly_to_id_ = services::text(data, "fly_to_id", "");
    last_progress_ = -std::numeric_limits<double>::infinity();
    phase_ = Phase::fly_to;
    return Result::ok;
}

Result ManualFlight::fly_to_update(const Json& data) {
    if (!airborne() || (phase_ != Phase::fly_to && phase_ != Phase::takeoff))
        return Result::fly_to_failed;
    const double speed = services::number(data, "max_speed", target_.speed);
    const auto target = parse_point(data, speed > 0 ? speed : target_.speed);
    if (restricted(target.latitude, target.longitude)) return Result::target_restricted;
    target_ = target;
    return Result::ok;
}

Result ManualFlight::fly_to_stop() {
    if (airborne() && (phase_ == Phase::fly_to || phase_ == Phase::takeoff)) phase_ = Phase::hover;
    return Result::ok;
}

Result ManualFlight::return_home() {
    if (!airborne()) return Result::rth_failed;
    const bool landing_home = phase_ == Phase::landing && landing_mode_ == DroneMode::auto_landing;
    if (phase_ != Phase::rth && !landing_home) phase_ = Phase::rth;
    return Result::ok;
}

Result ManualFlight::return_home_cancel() {
    const bool landing_home = phase_ == Phase::landing && landing_mode_ == DroneMode::auto_landing;
    if (!airborne() || (phase_ != Phase::rth && !landing_home)) return Result::rth_cancel_failed;
    phase_ = Phase::hover;
    return Result::ok;
}

void ManualFlight::stick(const Json& data) {
    const Sticks sticks{
        .forward = std::clamp(services::number(data, "x", 0.0), -stick_horizontal_max,
                              stick_horizontal_max),
        .right = std::clamp(services::number(data, "y", 0.0), -stick_horizontal_max,
                            stick_horizontal_max),
        .up = std::clamp(services::number(data, "h", 0.0), -stick_down_max, stick_up_max),
        .yaw = std::clamp(services::number(data, "w", 0.0), -stick_yaw_max, stick_yaw_max),
    };
    if (!airborne() || !drc_up()) return;
    const double now = ctx_.clock().monotonic();
    const auto mode = ctx_.drone.mode_code;
    if (phase_ == Phase::hover && (mode == DroneMode::manual || mode == DroneMode::virtual_stick)) {
        sticks_ = sticks;
        sticks_until_ = now + stick_hold_s;
        return;
    }
    // The web UI sends zero sticks all the time while its control panel is open.
    const bool moved =
        sticks.forward != 0.0 || sticks.right != 0.0 || sticks.up != 0.0 || sticks.yaw != 0.0;
    if (moved && now - last_joystick_notify_ >= joystick_notify_interval_s) {
        last_joystick_notify_ = now;
        notify_joystick(code::JoystickInvalidReason::rc_takeover);
    }
}

Result ManualFlight::emergency_stop() {
    sticks_until_ = 0.0;
    if (airborne() && phase_ != Phase::hover) phase_ = Phase::hover;
    return Result::ok;
}

Result ManualFlight::land_here() {
    if (!airborne()) return Result::not_supported;
    landing_mode_ = DroneMode::force_landing;
    phase_ = Phase::landing;
    return Result::ok;
}

Result ManualFlight::poi_enter(const Json& data) {
    const double latitude = services::number(data, "latitude");
    const double longitude = services::number(data, "longitude");
    const double height = services::number(data, "height");
    const bool ready = phase_ == Phase::poi || (phase_ == Phase::hover && manual_mode());
    if (!airborne() || !ready) {
        const auto reason = airborne() ? PoiReason::drone_mode_error : PoiReason::on_the_ground;
        ctx_.spawn(poi_notify(FlighttaskStatus::failed, reason), "poi-status");
        return Result::not_supported;
    }
    const auto& drone = ctx_.drone;
    poi_ = {.latitude = latitude, .longitude = longitude, .height = height, .active = true};
    poi_radius_ =
        std::max(poi_min_radius_m,
                 units::in_metres(geo::haversine(geo::point(drone.latitude, drone.longitude),
                                                 geo::point(latitude, longitude))));
    last_poi_notify_ = -std::numeric_limits<double>::infinity();
    phase_ = Phase::poi;
    return Result::ok;
}

Result ManualFlight::poi_exit() {
    if (phase_ == Phase::poi) {
        poi_.exit = true;
        phase_ = Phase::hover;
    }
    return Result::ok;
}

Result ManualFlight::poi_speed(const Json& data) {
    poi_speed_ = std::clamp(services::number(data, "circle_speed"), -poi_max_speed, poi_max_speed);
    // The next notify carries the new speed at once.
    last_poi_notify_ = -std::numeric_limits<double>::infinity();
    return Result::ok;
}

Result ManualFlight::wayline_deliver(const Json& data) {
    if (inflight_.open) return Result::task_started;
    const auto id = services::text(data, "in_flight_wayline_id", "");
    const Json file =
        data.contains("file") && data["file"].is_object() ? data["file"] : Json::object();
    const auto url = services::text(file, "url", "");
    if (id.empty() || url.empty())
        throw services::BadParameter("in_flight_wayline_id and file.url are required");
    if (!airborne() || phase_ != Phase::hover || !manual_mode()) return Result::not_supported;
    ctx_.drone.rth_altitude = services::number(data, "rth_altitude", ctx_.drone.rth_altitude);
    inflight_ = {.id = id, .url = url, .open = true};
    phase_ = Phase::wayline;
    return Result::ok;
}

Result ManualFlight::wayline_pause(const Json& data) {
    if (phase_ != Phase::wayline) return Result::pause_failed;
    if (services::text(data, "in_flight_wayline_id", inflight_.id) != inflight_.id)
        return Result::param_error;
    inflight_.paused = true;
    return Result::ok;
}

Result ManualFlight::wayline_resume(const Json& data) {
    if (phase_ != Phase::wayline || !inflight_.paused) return Result::resume_failed;
    if (services::text(data, "in_flight_wayline_id", inflight_.id) != inflight_.id)
        return Result::param_error;
    inflight_.paused = false;
    return Result::ok;
}

Result ManualFlight::wayline_cancel() {
    if (phase_ != Phase::wayline) return Result::task_not_started;
    phase_ = Phase::hover;
    return Result::ok;
}

std::string ManualFlight::media_id() const {
    return flight_id_.empty() ? track_id_ : flight_id_;
}

void ManualFlight::store_photo(const wayline::Shot& shot) {
    ++ctx_.drone.camera_photos;
    if (airborne()) {
        uploads_.hold(media_id(), shot);
        return;
    }
    // The upload names each file after the first 8 characters of the id, so they must differ.
    const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(ctx_.clock().now().time_since_epoch())
            .count();
    const auto seq = ++photo_seq_;
    const auto id =
        std::format("{:08x}-0000-4000-8000-{:012x}", static_cast<std::uint32_t>(ms) + seq, seq);
    uploads_.queue({.flight_id = id, .media_count = 1, .shots = {shot}});
}

bool ManualFlight::manual_mode() const {
    const auto mode = ctx_.drone.mode_code;
    return mode == DroneMode::manual || mode == DroneMode::virtual_stick;
}

bool ManualFlight::restricted(double latitude, double longitude) const {
    return safety_check_ && forbidden_ && forbidden_(latitude, longitude);
}

bool ManualFlight::drc_up() const {
    return ctx_.dock.drc_state == code::DrcState::connected;
}

void ManualFlight::notify_joystick(code::JoystickInvalidReason reason) {
    ctx_.spawn(ctx_.emit_event("joystick_invalid_notify", {{"reason", reason}}),
               "joystick-invalid-notify");
}

asio::awaitable<void> ManualFlight::publish_control_source() {
    co_await ctx_.transport().publish(topic::state(ctx_.drone_sn()),
                                      envelope::make(payloads::control_source_group(ctx_.drone), {},
                                                     {{"gateway", ctx_.dock_sn()}}));
}

asio::awaitable<void> ManualFlight::run() {
    try {
        co_await prepare();
        co_await takeoff_leg();
        while (phase_ != Phase::idle) {
            co_await close_interrupted();
            switch (phase_) {
                case Phase::hover:
                    co_await hover_tick();
                    break;
                case Phase::fly_to:
                    co_await fly_to_leg();
                    break;
                case Phase::poi:
                    co_await poi_leg();
                    break;
                case Phase::wayline:
                    co_await wayline_leg();
                    break;
                case Phase::rth:
                    co_await rth_leg();
                    break;
                case Phase::landing:
                    co_await land();
                    break;
                default:
                    phase_ = Phase::hover;
                    break;
            }
        }
    } catch (const std::system_error& e) {
        if (e.code() == std::errc::operation_canceled) throw;
        logger()->error("manual flight failed: {}", e.what());
    } catch (const std::exception& e) {
        logger()->error("manual flight failed: {}", e.what());
    }
    co_await close_interrupted();
    finish();
}

asio::awaitable<void> ManualFlight::prepare() {
    auto& dock = ctx_.dock;
    dock.mode_code = DockMode::working;
    co_await progress(true, FlyStatus::task_ready, target_);
    co_await ctx_.clock().sleep(ctx_.cfg().flight.prepare_seconds);
    dock.cover_state = 1;
    dock.putter_state = 1;
    dock.drone_in_dock = 0;
    dock.drone_charge_state = 0;
    ctx_.drone.mode_code = DroneMode::auto_takeoff;
    airborne_ = true;
    phase_ = Phase::takeoff;
}

asio::awaitable<void> ManualFlight::takeoff_leg() {
    const double tick = ctx_.cfg().flight.tick;
    auto& drone = ctx_.drone;
    const Target climb{.latitude = drone.latitude,
                       .longitude = drone.longitude,
                       .height = security_height_,
                       .speed = target_.speed};
    while (phase_ == Phase::takeoff) {
        if (progress_due()) co_await progress(true, FlyStatus::wayline_progress, target_);
        if (step_towards(climb, tick)) break;
        air_tick(tick);
        co_await ctx_.clock().sleep(tick);
    }
    if (phase_ == Phase::takeoff) drone.mode_code = DroneMode::live_flight_control;
    bool arrived = false;
    while (phase_ == Phase::takeoff) {
        if (step_towards(target_, tick)) {
            arrived = true;
            break;
        }
        if (progress_due()) co_await progress(true, FlyStatus::wayline_progress, target_);
        air_tick(tick);
        co_await ctx_.clock().sleep(tick);
    }
    drone.horizontal_speed = 0.0;
    drone.vertical_speed = 0.0;
    if (phase_ == Phase::takeoff) phase_ = Phase::hover;
    if (phase_ == Phase::hover) drone.mode_code = DroneMode::manual;
    co_await progress(true, arrived ? FlyStatus::wayline_ok : FlyStatus::wayline_cancel, target_);
    // The cloud closes the takeoff job on task_finish.
    co_await progress(true, FlyStatus::task_finish, target_);
}

asio::awaitable<void> ManualFlight::fly_to_leg() {
    const double tick = ctx_.cfg().flight.tick;
    auto& drone = ctx_.drone;
    drone.mode_code = DroneMode::live_flight_control;
    bool arrived = false;
    while (phase_ == Phase::fly_to) {
        if (step_towards(target_, tick)) {
            arrived = true;
            break;
        }
        if (progress_due()) co_await progress(false, FlyStatus::wayline_progress, target_);
        air_tick(tick);
        co_await ctx_.clock().sleep(tick);
    }
    drone.horizontal_speed = 0.0;
    drone.vertical_speed = 0.0;
    if (arrived && phase_ == Phase::fly_to) phase_ = Phase::hover;
    if (phase_ == Phase::hover) drone.mode_code = DroneMode::manual;
    co_await progress(false, arrived ? FlyStatus::wayline_ok : FlyStatus::wayline_cancel, target_);
}

asio::awaitable<void> ManualFlight::poi_leg() {
    const double tick = ctx_.cfg().flight.tick;
    auto& drone = ctx_.drone;
    drone.mode_code = DroneMode::live_flight_control;
    while (phase_ == Phase::poi) {
        const double now = ctx_.clock().monotonic();
        if (now - last_poi_notify_ >= poi_notify_interval_s) {
            last_poi_notify_ = now;
            co_await poi_notify(FlighttaskStatus::in_progress, PoiReason::normal);
        }
        orbit_step(tick);
        air_tick(tick);
        co_await ctx_.clock().sleep(tick);
    }
    drone.horizontal_speed = 0.0;
    drone.vertical_speed = 0.0;
    if (phase_ == Phase::hover) drone.mode_code = DroneMode::manual;
}

asio::awaitable<void> ManualFlight::wayline_leg() {
    const auto& flight = ctx_.cfg().flight;
    auto& drone = ctx_.drone;
    co_await wayline_progress(InFlightStatus::uploading);
    wayline::Wayline route;
    std::string error;
    try {
        route = co_await download_(inflight_.url);
    } catch (const std::system_error& e) {
        if (e.code() == std::errc::operation_canceled) throw;
        error = e.what();
    } catch (const std::exception& e) {
        error = e.what();
    }
    if (error.empty() && route.waypoints.empty()) error = "the wayline has no waypoints";
    if (!error.empty()) {
        logger()->error("in-flight wayline download failed: {}", error);
        co_await wayline_progress(InFlightStatus::failed, Result::route_download_failed);
        if (phase_ == Phase::wayline) phase_ = Phase::hover;
        co_return;
    }
    if (phase_ != Phase::wayline) co_return;
    co_await wayline_progress(InFlightStatus::uploaded);
    drone.mode_code = DroneMode::wayline;
    const auto total = static_cast<int>(route.waypoints.size());
    double gimbal_pitch = drone.gimbal_pitch;
    bool paused = false;
    int reached = 0;
    for (const auto& point : route.waypoints) {
        const double speed = point.speed.value_or(0.0) != 0.0 ? *point.speed
                             : route.speed != 0.0             ? route.speed
                                                              : flight.speed;
        const Target target{.latitude = point.latitude,
                            .longitude = point.longitude,
                            .height = std::max(min_height_m, point.height),
                            .speed = speed};
        bool arrived = false;
        while (phase_ == Phase::wayline) {
            if (inflight_.paused != paused) {
                paused = inflight_.paused;
                drone.horizontal_speed = 0.0;
                drone.vertical_speed = 0.0;
                co_await wayline_progress(paused ? InFlightStatus::paused
                                                 : InFlightStatus::progress);
            }
            if (!paused && step_towards(target, flight.tick)) {
                arrived = true;
                break;
            }
            air_tick(flight.tick);
            co_await ctx_.clock().sleep(flight.tick);
        }
        if (!arrived) break;
        if (point.gimbal_pitch) gimbal_pitch = *point.gimbal_pitch;
        drone.gimbal_pitch = gimbal_pitch;
        if (point.heading) drone.attitude_head = num::floor_mod(*point.heading, 360.0);
        if (point.take_photo) {
            store_photo({
                .latitude = drone.latitude,
                .longitude = drone.longitude,
                .height = drone.height,
                .heading = drone.camera_heading(),
                .gimbal_pitch = drone.gimbal_pitch,
                .time = ctx_.clock().now(),
                .waypoint_index = point.index,
            });
        }
        ++reached;
        inflight_.waypoint_index = point.index;
        inflight_.percent = reached * 100 / total;
        co_await wayline_progress(InFlightStatus::progress);
    }
    drone.horizontal_speed = 0.0;
    drone.vertical_speed = 0.0;
    if (reached == total && phase_ == Phase::wayline) {
        phase_ = Phase::hover;
        co_await wayline_progress(InFlightStatus::ok);
    }
    if (phase_ == Phase::hover) drone.mode_code = DroneMode::manual;
}

asio::awaitable<void> ManualFlight::close_interrupted() {
    if (poi_.active && phase_ != Phase::poi) {
        poi_.active = false;
        co_await poi_notify(poi_.exit ? FlighttaskStatus::ok : FlighttaskStatus::failed,
                            poi_.exit ? PoiReason::normal : PoiReason::drone_mode_error);
    }
    if (inflight_.open && phase_ != Phase::wayline)
        co_await wayline_progress(InFlightStatus::cancel);
}

asio::awaitable<void> ManualFlight::poi_notify(FlighttaskStatus status, PoiReason reason) {
    const bool running = status == FlighttaskStatus::in_progress;
    co_await ctx_.emit_event("poi_status_notify",
                             {
                                 {"status", status},
                                 {"reason", reason},
                                 {"circle_radius", running ? num::round_to(poi_radius_, 1) : 0.0},
                                 {"circle_speed", running ? num::round_to(poi_speed_, 1) : 0.0},
                                 {"max_circle_speed", poi_max_speed},
                             });
}

asio::awaitable<void> ManualFlight::wayline_progress(InFlightStatus status, Result result) {
    const bool end = status == InFlightStatus::cancel || status == InFlightStatus::ok ||
                     status == InFlightStatus::failed || status == InFlightStatus::timeout;
    if (end) inflight_.open = false;
    if (status == InFlightStatus::ok) inflight_.percent = 100;
    co_await ctx_.emit_event("in_flight_wayline_progress",
                             {
                                 {"in_flight_wayline_id", inflight_.id},
                                 {"status", status},
                                 {"result", result},
                                 {"way_point_index", inflight_.waypoint_index},
                                 {"progress", {{"percent", inflight_.percent}}},
                             });
}

asio::awaitable<void> ManualFlight::rth_leg() {
    const auto& flight = ctx_.cfg().flight;
    const auto& home = ctx_.cfg().device;
    auto& drone = ctx_.drone;
    drone.mode_code = DroneMode::auto_rth;
    const double height = std::max(drone.height, drone.rth_altitude);
    co_await return_home_info(height);
    const Target up{.latitude = drone.latitude,
                    .longitude = drone.longitude,
                    .height = height,
                    .speed = flight.speed};
    const Target dock{.latitude = home.latitude,
                      .longitude = home.longitude,
                      .height = height,
                      .speed = flight.speed};
    bool climbed = false;
    bool arrived = false;
    while (phase_ == Phase::rth) {
        if (!climbed) {
            climbed = step_towards(up, flight.tick);
        } else if (step_towards(dock, flight.tick)) {
            arrived = true;
            break;
        }
        air_tick(flight.tick);
        co_await ctx_.clock().sleep(flight.tick);
    }
    drone.horizontal_speed = 0.0;
    drone.vertical_speed = 0.0;
    if (arrived && phase_ == Phase::rth) {
        landing_mode_ = DroneMode::auto_landing;
        phase_ = Phase::landing;
    }
    if (phase_ == Phase::hover) drone.mode_code = DroneMode::manual;
}

asio::awaitable<void> ManualFlight::return_home_info(double height) {
    const auto& flight = ctx_.cfg().flight;
    const auto& home = ctx_.cfg().device;
    const auto& drone = ctx_.drone;
    const double distance = drone.home_distance();
    const double vertical = (height - drone.height) + height;
    const double seconds = distance / flight.speed + vertical / flight.climb_rate;
    co_await ctx_.emit_event(
        "return_home_info",
        {
            {"home_dock_sn", ctx_.dock_sn()},
            {"last_point_type", 0},
            {"planned_path_points",
             Json::array({path_point(drone.latitude, drone.longitude, drone.height),
                          path_point(home.latitude, home.longitude, height)})},
            {"multi_dock_home_info", Json::array({{
                                         {"sn", ctx_.dock_sn()},
                                         {"plan_status", 3},
                                         {"home_distance", num::round_to(distance, 1)},
                                         {"estimated_battery_consumption", battery_for(seconds)},
                                     }})},
        });
}

asio::awaitable<void> ManualFlight::land() {
    const auto& flight = ctx_.cfg().flight;
    auto& dock = ctx_.dock;
    auto& drone = ctx_.drone;
    const Target ground{.latitude = drone.latitude,
                        .longitude = drone.longitude,
                        .height = 0.0,
                        .speed = flight.speed};
    while (phase_ == Phase::landing) {
        drone.mode_code = landing_mode_;
        if (step_towards(ground, flight.tick)) break;
        air_tick(flight.tick);
        co_await ctx_.clock().sleep(flight.tick);
    }
    drone.horizontal_speed = 0.0;
    drone.vertical_speed = 0.0;
    if (phase_ != Phase::landing) {
        if (phase_ == Phase::hover) drone.mode_code = DroneMode::manual;
        co_return;
    }
    // A landing away from the dock also ends in the dock. The simulator has no field recovery.
    drone.reset_to_dock();
    dock.drone_in_dock = 1;
    dock.cover_state = 0;
    dock.putter_state = 0;
    dock.drone_charge_state = 1;
    ++dock.job_number;
    airborne_ = false;
    co_await ctx_.clock().sleep(std::min(flight.land_seconds, wayline::cover_close_max_s));
    phase_ = Phase::idle;
}

asio::awaitable<void> ManualFlight::hover_tick() {
    const double tick = ctx_.cfg().flight.tick;
    apply_sticks(tick);
    air_tick(tick);
    co_await ctx_.clock().sleep(tick);
}

void ManualFlight::finish() {
    auto& dock = ctx_.dock;
    uploads_.release(media_id());
    if (airborne_) {
        ctx_.drone.reset_to_dock();
        dock.drone_in_dock = 1;
        dock.cover_state = 0;
        dock.putter_state = 0;
        airborne_ = false;
    }
    dock.mode_code = DockMode::idle;
    phase_ = Phase::idle;
    sticks_until_ = 0.0;
}

bool ManualFlight::step_towards(const Target& target, double dt) {
    const auto& flight = ctx_.cfg().flight;
    auto& drone = ctx_.drone;
    const auto from = geo::point(drone.latitude, drone.longitude);
    const auto to = geo::point(target.latitude, target.longitude);
    const double horizontal = units::in_metres(geo::haversine(from, to));
    const double climb = target.height - drone.height;
    if (horizontal < wayline::arrive_horizontal_m && std::abs(climb) < wayline::arrive_vertical_m) {
        drone.horizontal_speed = 0.0;
        drone.vertical_speed = 0.0;
        return true;
    }
    drone.horizontal_speed = 0.0;
    if (horizontal >= wayline::arrive_horizontal_m) {
        const double bearing = units::in_degrees(geo::bearing(from, to));
        // Turn toward the leg at a limited rate; an instant turn shows as a jolt.
        const double turn = geo::signed_angle(bearing - drone.attitude_head);
        const double max_turn = wayline::yaw_rate_deg_s * dt;
        drone.attitude_head =
            num::floor_mod(drone.attitude_head + std::clamp(turn, -max_turn, max_turn), 360.0);
        const double speed = device::ground_speed(ctx_.dock.weather, bearing, target.speed);
        const double step = std::min(speed * dt, horizontal);
        const auto next = geo::move_towards(from, to, step * units::metre);
        drone.latitude = units::in_degrees(next.lat);
        drone.longitude = units::in_degrees(next.lon);
        drone.course = bearing;
        drone.total_flight_distance += step;
        drone.horizontal_speed = step / dt;
    }
    const double vertical = std::min(flight.climb_rate * dt, std::abs(climb));
    const double sign = climb > 0 ? 1.0 : -1.0;
    drone.height += vertical * sign;
    drone.vertical_speed = vertical / dt * sign;
    return false;
}

void ManualFlight::orbit_step(double dt) {
    const auto& flight = ctx_.cfg().flight;
    auto& drone = ctx_.drone;
    const auto centre = geo::point(poi_.latitude, poi_.longitude);
    const auto here = geo::point(drone.latitude, drone.longitude);
    const double radius = units::in_metres(geo::haversine(centre, here));
    const double bearing =
        radius > 0.1 ? units::in_degrees(geo::bearing(centre, here)) : drone.attitude_head + 180.0;
    // Move out or in to the orbit radius at cruise speed, then circle.
    const double radial = std::clamp(poi_radius_ - radius, -flight.speed * dt, flight.speed * dt);
    const double next = radius + radial;
    const double angle = geo::radians(bearing) + poi_speed_ * dt / std::max(next, 1.0);
    const double lat = geo::radians(poi_.latitude);
    drone.latitude = poi_.latitude + geo::degrees(next * std::cos(angle) / geo::earth_radius_m);
    drone.longitude = poi_.longitude +
                      geo::degrees(next * std::sin(angle) / (geo::earth_radius_m * std::cos(lat)));
    const double moved = std::hypot(poi_speed_ * dt, radial);
    drone.total_flight_distance += moved;
    drone.horizontal_speed = moved / dt;
    drone.vertical_speed = 0.0;
    drone.course = num::floor_mod(geo::degrees(angle) + (poi_speed_ >= 0.0 ? 90.0 : -90.0), 360.0);
    drone.gimbal_yaw = 0.0;
    gimbal::look_at(drone, drone.home.altitude, poi_.latitude, poi_.longitude, poi_.height,
                    true);
}

void ManualFlight::apply_sticks(double dt) {
    auto& drone = ctx_.drone;
    const bool held = ctx_.clock().monotonic() < sticks_until_;
    const Sticks s = held ? sticks_ : Sticks{};
    drone.attitude_head = num::floor_mod(drone.attitude_head + s.yaw * dt, 360.0);
    const double heading = geo::radians(drone.attitude_head);
    const double north = s.forward * std::cos(heading) - s.right * std::sin(heading);
    const double east = s.forward * std::sin(heading) + s.right * std::cos(heading);
    const double speed = std::hypot(north, east);
    if (speed > 0.0) {
        const double lat = geo::radians(drone.latitude);
        drone.latitude += geo::degrees(north * dt / geo::earth_radius_m);
        drone.longitude += geo::degrees(east * dt / (geo::earth_radius_m * std::cos(lat)));
        drone.course = num::floor_mod(geo::degrees(std::atan2(east, north)), 360.0);
        drone.total_flight_distance += speed * dt;
    }
    drone.horizontal_speed = speed;
    const double before = drone.height;
    const double next = before + s.up * dt;
    drone.height = s.up < 0.0 ? std::max(next, std::min(before, min_height_m)) : next;
    drone.vertical_speed = (drone.height - before) / dt;
}

void ManualFlight::air_tick(double dt) {
    auto& drone = ctx_.drone;
    drone.total_flight_time += dt;
    device::drain(drone, dt, device::drain_factor(ctx_.dock.weather));
    device::apply_drift(drone, ctx_.dock.weather, dt);
    if (observe_ && observe_() && drc_up())
        notify_joystick(code::JoystickInvalidReason::near_restricted_zone);
    const bool interruptible = phase_ == Phase::takeoff || phase_ == Phase::hover ||
                               phase_ == Phase::fly_to || phase_ == Phase::poi ||
                               phase_ == Phase::wayline;
    if (!low_battery_done_ && interruptible &&
        drone.battery_percent() <= device::return_home_power_percent) {
        low_battery_done_ = true;
        logger()->info("battery {} %: return to home", drone.battery_percent());
        phase_ = Phase::rth;
        if (drc_up()) notify_joystick(code::JoystickInvalidReason::low_battery_return);
    }
}

double ManualFlight::remaining(const Target& target) const {
    const auto& drone = ctx_.drone;
    return units::in_metres(geo::haversine(geo::point(drone.latitude, drone.longitude),
                                           geo::point(target.latitude, target.longitude))) +
           std::abs(target.height - drone.height);
}

Json ManualFlight::path_point(double latitude, double longitude, double height) const {
    return {
        {"latitude", num::round_to(latitude, 8)},
        {"longitude", num::round_to(longitude, 8)},
        {"height", num::round_to(ctx_.drone.home.altitude + height, 3)},
    };
}

bool ManualFlight::progress_due() {
    const double now = ctx_.clock().monotonic();
    if (now - last_progress_ < progress_interval_s) return false;
    last_progress_ = now;
    return true;
}

asio::awaitable<void> ManualFlight::progress(bool takeoff, FlyStatus status, Target target) {
    const double left = remaining(target);
    Json data = takeoff ? Json{{"flight_id", flight_id_}, {"track_id", track_id_}}
                        : Json{{"fly_to_id", fly_to_id_}};
    data["status"] = status;
    data["result"] = Result::ok;
    data["way_point_index"] =
        left < wayline::arrive_horizontal_m + wayline::arrive_vertical_m ? 1 : 0;
    data["remaining_distance"] = num::round_to(left, 1);
    data["remaining_time"] = num::round_to(left / target.speed, 1);
    data["planned_path_points"] =
        Json::array({path_point(target.latitude, target.longitude, target.height)});
    co_await ctx_.emit_event(takeoff ? "takeoff_to_point_progress" : "fly_to_point_progress",
                             std::move(data));
}

}  // namespace docksim::flight
