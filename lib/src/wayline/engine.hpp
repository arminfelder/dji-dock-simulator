#pragma once

#include "protocol/codes.hpp"
#include "sim/async.hpp"
#include "sim/clock.hpp"
#include "util/json.hpp"
#include "wayline/wpml.hpp"

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace docksim::sim {
class DockContext;
}

// Wayline mission engine. One flight is one coroutine. The step and status
// sequence of flighttask_progress follows the real dock; the mission golden
// snapshot pins it.
namespace docksim::wayline {

// Distances below these count as arrived, metres.
inline constexpr double arrive_horizontal_m = 0.5;
inline constexpr double arrive_vertical_m = 0.3;
inline constexpr double pause_poll_s = 0.2;
// Yaw rate of the aircraft while it turns onto a new leg, degrees per second.
inline constexpr double yaw_rate_deg_s = 45.0;
// The real dock closes the cover in about this time after touchdown.
inline constexpr double cover_close_max_s = 3.0;
// flighttask_prepare task_type of a conditional task (cloud TaskTypeEnum CONDITIONAL).
inline constexpr int conditional_task_type = 2;

// Aircraft and camera pose at the moment of one photo.
struct Shot {
    double latitude{};
    double longitude{};
    // Metres above the dock.
    double height{};
    // Degrees clockwise from north.
    double heading{};
    // Degrees, -90 is nadir.
    double gimbal_pitch{};
    sim::WallTime time;
    int waypoint_index{};
};

struct Task {
    std::string flight_id;
    std::string file_url;
    std::string track_id;
    int task_type = 0;
    double rth_altitude = 100.0;
    std::optional<Wayline> wayline;
    int media_count = 0;
    int breakpoint_index = 0;
    std::vector<Shot> shots;
};

using Download = std::function<asio::awaitable<Wayline>(std::string url)>;
using LandedSink = std::function<void(const Task&)>;

// The last flighttask_progress of the task.
struct Progress {
    code::FlighttaskStatus status = code::FlighttaskStatus::sent;
    int percent = 0;
    int waypoint = 0;
};

// Drives the aircraft through a wayline and reports flighttask_progress.
class MissionEngine {
public:
    MissionEngine(sim::DockContext& ctx, Download download, LandedSink on_landed);

    [[nodiscard]] bool flying() const { return !flight_.done(); }
    [[nodiscard]] const std::optional<Task>& task() const { return task_; }
    [[nodiscard]] const Progress& progress() const { return progress_; }
    asio::awaitable<void> wait() const { return flight_.wait(); }

    // execute answers task_busy while this is true. Another flight owns the aircraft then.
    void refuse_while(std::function<bool()> busy) { busy_ = std::move(busy); }
    // execute answers the code of this check if it has one: weather, faults.
    void preflight(std::function<std::optional<code::Result>()> check) {
        preflight_ = std::move(check);
    }
    // Called after each motion step of the aircraft.
    void on_step(std::function<void()> step) { on_step_ = std::move(step); }

    asio::awaitable<code::Result> prepare(const Json& data);
    code::Result execute(const std::string& flight_id);
    code::Result pause();
    code::Result recovery();
    // Stop the wayline. A flying aircraft still returns and lands.
    code::Result cancel();
    code::Result undo(const std::vector<std::string>& flight_ids);
    // Cancel a running flight and wait for it. For shutdown.
    asio::awaitable<void> shutdown() { return flights_.cancel_all(); }

private:
    asio::awaitable<void> report(code::FlighttaskStatus status, code::WaylineState state,
                                 code::FlighttaskStep step, int percent, int waypoint = 0);
    asio::awaitable<void> run();
    asio::awaitable<void> takeoff();
    asio::awaitable<void> climb_to(double target);
    asio::awaitable<void> fly(const Wayline& wayline);
    asio::awaitable<void> go_to(double latitude, double longitude, double height, double speed);
    asio::awaitable<void> land();
    asio::awaitable<void> wait_while_paused();
    // The wayline of the task. One failed download asks flighttask_resource_get for a new URL.
    asio::awaitable<std::optional<Wayline>> fetch(Task& task);
    void stepped() const {
        if (on_step_) on_step_();
    }

    sim::DockContext& ctx_;
    Download download_;
    LandedSink on_landed_;
    std::function<bool()> busy_;
    std::function<std::optional<code::Result>()> preflight_;
    std::function<void()> on_step_;
    std::optional<Task> task_;
    // The task of the flight that runs. A flight always has one.
    [[nodiscard]] Task& active() { return task_.value(); }
    sim::TaskGroup flights_;
    sim::TaskGroup::Handle flight_;
    Progress progress_;
    bool pause_ = false;
    bool stop_ = false;
};

}  // namespace docksim::wayline
