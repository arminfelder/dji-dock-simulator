#include "wayline/engine.hpp"

#include "device/geo.hpp"
#include "device/weather.hpp"
#include "log/log.hpp"
#include "services/registry.hpp"
#include "sim/context.hpp"
#include "util/numeric.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <utility>

namespace docksim::wayline {

namespace {

using code::DockMode;
using code::DroneMode;
using code::FlighttaskStatus;
using code::FlighttaskStep;
using code::Result;
using code::WaylineState;

const auto& logger() {
    static const auto log = log::get("dock_sim.mission");
    return log;
}

}  // namespace

MissionEngine::MissionEngine(sim::DockContext& ctx, Download download, LandedSink on_landed)
    : ctx_(ctx),
      download_(std::move(download)),
      on_landed_(std::move(on_landed)),
      flights_(ctx.executor(), [](const std::string& name, std::exception_ptr e) {
          try {
              std::rethrow_exception(std::move(e));
          } catch (const std::exception& error) {
              logger()->error("{} failed: {}", name, error.what());
          }
      }) {}

asio::awaitable<Result> MissionEngine::prepare(const Json& data) {
    const auto flight_id = services::text(data, "flight_id", "");
    if (task_) {
        // The cloud can send the same prepare twice. Answer the repeat with
        // success, so that the cloud does not fail the task.
        if (task_->flight_id == flight_id) co_return Result::ok;
        if (flying()) co_return Result::task_busy;
    }
    const Json file =
        data.contains("file") && data["file"].is_object() ? data["file"] : Json::object();
    const auto now = std::chrono::floor<std::chrono::seconds>(ctx_.clock().now());
    Task task{
        .flight_id = flight_id,
        .file_url = services::text(file, "url", ""),
        .track_id = std::format("track-{}", now.time_since_epoch().count()),
        .task_type = static_cast<int>(services::integer(data, "task_type", 0)),
        .rth_altitude = services::number(data, "rth_altitude", 100.0),
    };
    task.wayline = co_await fetch(task);
    if (!task.wayline) co_return Result::wayline_download_failed;
    // An execute can start the old task while the download runs. Replacing task_ under a flight
    // would put its breakpoints and its end into the new task.
    if (flying()) co_return Result::task_busy;
    task_ = std::move(task);
    co_await report(FlighttaskStatus::sent, WaylineState::prepare, FlighttaskStep::download_kmz,
                    100);
    // A conditional task waits on the dock for its ready conditions. The
    // simulator meets them at once, after the reply.
    if (task_->task_type == conditional_task_type) {
        ctx_.spawn(
            ctx_.emit_event("flighttask_ready", {{"flight_ids", Json::array({task_->flight_id})}}),
            "flighttask-ready");
    }
    co_return Result::ok;
}

asio::awaitable<std::optional<Wayline>> MissionEngine::fetch(Task& task) {
    std::string error;
    try {
        co_return co_await download_(task.file_url);
    } catch (const std::exception& e) {
        error = e.what();
    }
    logger()->warn("wayline download failed: {}; asking for a new URL", error);
    Json reply;
    try {
        reply = co_await ctx_.request("flighttask_resource_get", {{"flight_id", task.flight_id}});
    } catch (const transport::RequestTimeout& e) {
        logger()->error("flighttask_resource_get: {}", e.what());
        co_return std::nullopt;
    }
    const Json output = reply.value("output", Json::object());
    const Json file = output.is_object() ? output.value("file", Json::object()) : Json::object();
    const auto url = file.is_object() ? file.value("url", "") : std::string{};
    if (reply.value("result", 0) != 0 || url.empty()) {
        logger()->error("flighttask_resource_get gave no URL: {}", reply.dump());
        co_return std::nullopt;
    }
    task.file_url = url;
    try {
        co_return co_await download_(url);
    } catch (const std::exception& e) {
        logger()->error("wayline download failed again: {}", e.what());
    }
    co_return std::nullopt;
}

Result MissionEngine::execute(const std::string& flight_id) {
    if (!task_ || task_->flight_id != flight_id) return Result::task_not_prepared;
    if (ctx_.dock.mode_code == DockMode::remote_debug) return Result::remote_debug_active;
    if (ctx_.dock.mode_code == DockMode::upgrading) return Result::ota_upgrading;
    if (flying()) return Result::task_running;
    if (busy_ && busy_()) return Result::task_busy;
    if (preflight_) {
        if (const auto refused = preflight_()) return *refused;
    }
    pause_ = false;
    stop_ = false;
    flight_ = flights_.spawn(run(), std::format("flight-{}", flight_id));
    return Result::ok;
}

Result MissionEngine::pause() {
    if (!flying()) return Result::task_not_running;
    pause_ = true;
    return Result::ok;
}

Result MissionEngine::recovery() {
    if (!flying()) return Result::task_not_running;
    pause_ = false;
    return Result::ok;
}

Result MissionEngine::cancel() {
    if (!flying()) {
        task_.reset();
        return Result::ok;
    }
    stop_ = true;
    pause_ = false;
    return Result::ok;
}

Result MissionEngine::undo(const std::vector<std::string>& flight_ids) {
    if (task_ && std::ranges::contains(flight_ids, task_->flight_id) && !flying()) task_.reset();
    return Result::ok;
}

asio::awaitable<void> MissionEngine::report(FlighttaskStatus status, WaylineState state,
                                            FlighttaskStep step, int percent, int waypoint) {
    if (!task_) co_return;
    ctx_.dock.flighttask_step_code = step;
    progress_ = {.status = status, .percent = percent, .waypoint = waypoint};
    co_await ctx_.emit_event("flighttask_progress",
                             {
                                 {"output",
                                  {
                                      {"ext",
                                       {
                                           {"current_waypoint_index", waypoint},
                                           {"flight_id", task_->flight_id},
                                           {"track_id", task_->track_id},
                                           {"media_count", task_->media_count},
                                           {"wayline_id", 0},
                                           {"wayline_mission_state", state},
                                       }},
                                      {"progress", {{"current_step", step}, {"percent", percent}}},
                                      {"status", status},
                                  }},
                                 {"result", 0},
                             });
}

asio::awaitable<void> MissionEngine::run() {
    const Wayline wayline = active().wayline.value();
    const int last = static_cast<int>(wayline.waypoints.size()) - 1;
    bool failed = false;
    try {
        co_await takeoff();
        if (!stop_) {
            co_await fly(wayline);
            if (stop_) {
                co_await report(FlighttaskStatus::canceled, WaylineState::interrupted,
                                FlighttaskStep::landing, 0);
            } else {
                co_await report(FlighttaskStatus::in_progress, WaylineState::complete,
                                FlighttaskStep::landing, 100, last);
            }
            // A cancel during the landing changes nothing: the aircraft lands anyway.
            const bool cancelled = stop_;
            co_await land();
            if (on_landed_ && active().media_count > 0) on_landed_(active());
            co_await report(cancelled ? FlighttaskStatus::canceled : FlighttaskStatus::ok,
                            WaylineState::complete, FlighttaskStep::notify_result, 100, last);
        }
    } catch (const std::system_error& e) {
        if (e.code() == std::errc::operation_canceled) throw;
        logger()->error("mission failed: {}", e.what());
        failed = true;
    } catch (const std::exception& e) {
        logger()->error("mission failed: {}", e.what());
        failed = true;
    }
    if (failed) {
        co_await report(FlighttaskStatus::failed, WaylineState::interrupted, FlighttaskStep::none,
                        0);
    }
    ctx_.dock.flighttask_step_code = FlighttaskStep::no_task;
    ctx_.dock.mode_code = DockMode::idle;
    ctx_.drone.reset_to_dock();
    task_.reset();
}

asio::awaitable<void> MissionEngine::takeoff() {
    const auto& flight = ctx_.cfg().flight;
    const double third = flight.prepare_seconds / 3;
    auto& dock = ctx_.dock;
    auto& drone = ctx_.drone;

    dock.mode_code = DockMode::working;
    co_await report(FlighttaskStatus::in_progress, WaylineState::triggered,
                    FlighttaskStep::dock_working, 0);
    co_await ctx_.clock().sleep(third);

    dock.cover_state = 1;
    dock.putter_state = 1;
    co_await report(FlighttaskStatus::in_progress, WaylineState::triggered,
                    FlighttaskStep::cover_open, 30);
    co_await ctx_.clock().sleep(third);

    drone.mode_code = DroneMode::ready;
    dock.drone_in_dock = 0;
    co_await report(FlighttaskStatus::in_progress, WaylineState::triggered,
                    FlighttaskStep::set_home, 60);
    co_await ctx_.clock().sleep(third);

    drone.mode_code = DroneMode::auto_takeoff;
    co_await climb_to(flight.takeoff_altitude);
    drone.mode_code = DroneMode::wayline;
    co_await report(FlighttaskStatus::in_progress, WaylineState::entering,
                    FlighttaskStep::trigger_wayline, 0);
}

asio::awaitable<void> MissionEngine::climb_to(double target) {
    const auto& flight = ctx_.cfg().flight;
    auto& drone = ctx_.drone;
    while (!stop_ && std::abs(drone.height - target) > arrive_vertical_m) {
        co_await wait_while_paused();
        const double delta = target - drone.height;
        const double move =
            std::min(flight.climb_rate * flight.tick, std::abs(delta)) * (delta > 0 ? 1 : -1);
        drone.height += move;
        drone.vertical_speed = move / flight.tick;
        stepped();
        co_await ctx_.clock().sleep(flight.tick);
    }
    drone.vertical_speed = 0.0;
}

asio::awaitable<void> MissionEngine::fly(const Wayline& wayline) {
    const auto& flight = ctx_.cfg().flight;
    auto& drone = ctx_.drone;
    const auto total = static_cast<int>(wayline.waypoints.size());
    double gimbal_pitch = -90.0;
    for (int position = 0; position < total; ++position) {
        const auto& point = wayline.waypoints[static_cast<std::size_t>(position)];
        if (stop_) co_return;
        active().breakpoint_index = point.index;
        const double point_speed = point.speed.value_or(0.0);
        const double speed = point_speed != 0.0     ? point_speed
                             : wayline.speed != 0.0 ? wayline.speed
                                                    : flight.speed;
        co_await go_to(point.latitude, point.longitude, point.height, speed);
        if (point.gimbal_pitch) gimbal_pitch = *point.gimbal_pitch;
        drone.gimbal_pitch = gimbal_pitch;
        if (point.heading) drone.attitude_head = num::floor_mod(*point.heading, 360.0);
        if (point.take_photo) {
            ++active().media_count;
            ++drone.camera_photos;
            active().shots.push_back({
                .latitude = drone.latitude,
                .longitude = drone.longitude,
                .height = drone.height,
                .heading = drone.attitude_head,
                .gimbal_pitch = gimbal_pitch,
                .time = ctx_.clock().now(),
                .waypoint_index = point.index,
            });
        }
        co_await report(FlighttaskStatus::in_progress, WaylineState::executing,
                        FlighttaskStep::trigger_wayline, (position + 1) * 100 / std::max(total, 1),
                        point.index);
    }
}

asio::awaitable<void> MissionEngine::go_to(double latitude, double longitude, double height,
                                           double speed) {
    const auto& flight = ctx_.cfg().flight;
    auto& drone = ctx_.drone;
    const auto to = geo::point(latitude, longitude);
    const double target =
        units::in_degrees(geo::bearing(geo::point(drone.latitude, drone.longitude), to));
    while (!stop_) {
        co_await wait_while_paused();
        // Turn toward the leg at a limited rate; an instant turn shows as a jolt.
        const double turn = geo::signed_angle(target - drone.attitude_head);
        const double max_turn = yaw_rate_deg_s * flight.tick;
        drone.attitude_head =
            num::floor_mod(drone.attitude_head + std::clamp(turn, -max_turn, max_turn), 360.0);
        const auto from = geo::point(drone.latitude, drone.longitude);
        const double remaining = units::in_metres(geo::haversine(from, to));
        const double climb = height - drone.height;
        if (remaining < arrive_horizontal_m && std::abs(climb) < arrive_vertical_m) break;

        const double step = device::ground_speed(ctx_.dock.weather, target, speed) * flight.tick;
        const auto next = geo::move_towards(from, to, step * units::metre);
        drone.latitude = units::in_degrees(next.lat);
        drone.longitude = units::in_degrees(next.lon);
        drone.total_flight_distance += std::min(step, remaining);
        drone.horizontal_speed = std::min(step, remaining) / flight.tick;

        const double vertical = std::min(flight.climb_rate * flight.tick, std::abs(climb));
        const double sign = climb > 0 ? 1.0 : -1.0;
        drone.height += vertical * sign;
        drone.vertical_speed = vertical / flight.tick * sign;

        drone.total_flight_time += flight.tick;
        device::drain(drone, flight.tick, device::drain_factor(ctx_.dock.weather));
        device::apply_drift(drone, ctx_.dock.weather, flight.tick);
        stepped();
        co_await ctx_.clock().sleep(flight.tick);
    }
    drone.horizontal_speed = 0.0;
    drone.vertical_speed = 0.0;
}

asio::awaitable<void> MissionEngine::land() {
    const auto& flight = ctx_.cfg().flight;
    const auto& home = ctx_.cfg().device;
    auto& dock = ctx_.dock;
    auto& drone = ctx_.drone;

    drone.mode_code = DroneMode::auto_rth;
    co_await go_to(home.latitude, home.longitude, active().rth_altitude, flight.speed);
    drone.mode_code = DroneMode::auto_landing;
    // A cancelled mission must still land, so release the stop flag here.
    stop_ = false;
    co_await climb_to(0.0);
    drone.reset_to_dock();
    dock.drone_in_dock = 1;
    dock.cover_state = 0;
    dock.putter_state = 0;
    ++dock.job_number;
    dock.drone_charge_state = 1;
    dock.media_remain_upload = active().media_count;
    co_await report(FlighttaskStatus::in_progress, WaylineState::complete,
                    FlighttaskStep::cover_close, 100);
    co_await ctx_.clock().sleep(std::min(flight.land_seconds, cover_close_max_s));
}

asio::awaitable<void> MissionEngine::wait_while_paused() {
    bool was_paused = false;
    while (pause_ && !stop_) {
        if (!was_paused) {
            was_paused = true;
            ctx_.drone.horizontal_speed = 0.0;
            ctx_.drone.vertical_speed = 0.0;
            co_await report(FlighttaskStatus::paused, WaylineState::interrupted,
                            FlighttaskStep::trigger_wayline, 0);
        }
        co_await ctx_.clock().sleep(pause_poll_s);
    }
    if (was_paused && !stop_) {
        co_await report(FlighttaskStatus::in_progress, WaylineState::recovering,
                        FlighttaskStep::trigger_wayline, 0);
    }
}

}  // namespace docksim::wayline
