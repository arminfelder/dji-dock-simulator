#pragma once

#include "protocol/codes.hpp"
#include "sim/async.hpp"
#include "util/json.hpp"
#include "wayline/engine.hpp"

#include <functional>
#include <limits>
#include <string>

namespace docksim::sim {
class DockContext;
}
namespace docksim::services {
class Registry;
}
namespace docksim::media {
class MediaUploads;
}

// Manual flight: one-key takeoff, fly-to, sticks over DRC, POI orbit, in-flight
// wayline, return to home and landing. One flight is one coroutine, from
// takeoff_to_point to touchdown. The wire rules are in docs/protocol-quirks.md
// under Manual flight.
namespace docksim::flight {

// A drone_control command holds for this time. The web UI sends one every 50 ms.
inline constexpr double stick_hold_s = 0.5;
// joystick_invalid_notify repeats at most at this interval, seconds.
inline constexpr double joystick_notify_interval_s = 2.0;
// Sticks and fly-to targets do not go below this height above the dock, metres.
inline constexpr double min_height_m = 2.0;
// takeoff_to_point_progress and fly_to_point_progress interval, seconds.
inline constexpr double progress_interval_s = 1.0;
// fly_to_point without max_speed, metres per second.
inline constexpr double fly_to_default_speed = 5.0;
// POI orbit. Speeds are metres per second, positive is clockwise.
inline constexpr double poi_min_radius_m = 5.0;
inline constexpr double poi_default_speed = 3.0;
inline constexpr double poi_max_speed = 10.0;
// poi_status_notify in_progress interval, seconds.
inline constexpr double poi_notify_interval_s = 5.0;

// Stick limits of drone_control (DroneControlRequest of the cloud).
inline constexpr double stick_horizontal_max = 17.0;
inline constexpr double stick_up_max = 5.0;
inline constexpr double stick_down_max = 4.0;
inline constexpr double stick_yaw_max = 90.0;

enum class Phase : std::uint8_t {
    idle,
    prepare,
    takeoff,
    hover,
    fly_to,
    poi,
    wayline,
    rth,
    landing
};

// Body-frame stick speeds.
struct Sticks {
    // Metres per second, positive is forward.
    double forward = 0.0;
    // Metres per second, positive is to the right.
    double right = 0.0;
    // Metres per second, positive is up.
    double up = 0.0;
    // Degrees per second, positive is clockwise.
    double yaw = 0.0;
};

class ManualFlight {
public:
    ManualFlight(sim::DockContext& ctx, const wayline::MissionEngine& mission,
                 wayline::Download download, media::MediaUploads& uploads);

    // takeoff_to_point, fly_to_point*, poi_*, in_flight_wayline_*, authority
    // grabs, return_home and return_home_cancel. The last two fall back to the
    // mission behaviour when no manual flight runs.
    void register_services(services::Registry& registry, wayline::MissionEngine& mission);

    // Custom flight areas. forbidden tells if a point is in an enabled no-fly
    // zone. observe runs after each motion step; true means the aircraft has
    // just entered a no-fly zone or left a geofence.
    // takeoff_to_point answers the code of this check if it has one: weather, faults.
    void preflight(std::function<std::optional<code::Result>()> check) {
        preflight_ = std::move(check);
    }
    void guard(std::function<bool(double latitude, double longitude)> forbidden,
               std::function<bool()> observe) {
        forbidden_ = std::move(forbidden);
        observe_ = std::move(observe);
    }

    // True from takeoff_to_point until the aircraft is back in the dock.
    [[nodiscard]] bool flying() const { return !flight_.done(); }
    [[nodiscard]] bool airborne() const { return flying() && airborne_; }
    [[nodiscard]] Phase phase() const { return phase_; }
    asio::awaitable<void> wait() const { return flight_.wait(); }
    asio::awaitable<void> shutdown() { return flights_.cancel_all(); }

    code::Result takeoff(const Json& data);
    code::Result fly_to(const Json& data);
    code::Result fly_to_update(const Json& data);
    code::Result fly_to_stop();
    code::Result return_home();
    code::Result return_home_cancel();

    // POI orbit around a point. Needs the aircraft in the air in manual or virtual-stick mode.
    code::Result poi_enter(const Json& data);
    code::Result poi_exit();
    code::Result poi_speed(const Json& data);
    [[nodiscard]] double poi_radius() const { return poi_radius_; }
    [[nodiscard]] double poi_circle_speed() const { return poi_speed_; }

    // In-flight wayline: a KMZ that the aircraft flies from its hover.
    code::Result wayline_deliver(const Json& data);
    code::Result wayline_pause(const Json& data);
    code::Result wayline_resume(const Json& data);
    code::Result wayline_cancel();

    // A photo of the camera. It uploads with the media of this flight after the
    // landing, or at once with an id of its own when no manual flight is in the air.
    void store_photo(const wayline::Shot& shot);
    // The flight id of the media of this flight.
    [[nodiscard]] std::string media_id() const;

    // DRC commands. Sticks move the aircraft only in manual or virtual-stick mode.
    void stick(const Json& data);
    code::Result emergency_stop();
    code::Result land_here();

private:
    struct Target {
        double latitude{};
        double longitude{};
        // Metres above the dock.
        double height{};
        double speed{};
    };

    struct Poi {
        double latitude{};
        double longitude{};
        // Ellipsoid height, metres.
        double height{};
        // True from poi_mode_enter until poi_status_notify reports the end.
        bool active = false;
        bool exit = false;
    };

    struct InFlight {
        std::string id;
        std::string url;
        bool paused = false;
        // True from in_flight_wayline_deliver until a progress with an end status.
        bool open = false;
        int waypoint_index = 0;
        int percent = 0;
    };

    asio::awaitable<void> run();
    asio::awaitable<void> prepare();
    asio::awaitable<void> takeoff_leg();
    asio::awaitable<void> fly_to_leg();
    asio::awaitable<void> poi_leg();
    asio::awaitable<void> wayline_leg();
    asio::awaitable<void> rth_leg();
    asio::awaitable<void> land();
    asio::awaitable<void> hover_tick();
    asio::awaitable<void> publish_control_source();
    asio::awaitable<void> return_home_info(double height);
    // Report the end of a POI orbit or an in-flight wayline that another command stopped.
    asio::awaitable<void> close_interrupted();
    asio::awaitable<void> poi_notify(code::FlighttaskStatus status, code::PoiReason reason);
    asio::awaitable<void> wayline_progress(code::InFlightStatus status,
                                           code::Result result = code::Result::ok);
    void finish();

    // One motion step towards the target. True when the aircraft is there.
    bool step_towards(const Target& target, double dt);
    void orbit_step(double dt);
    void apply_sticks(double dt);
    // Flight time, battery drain and the low-battery return.
    void air_tick(double dt);
    [[nodiscard]] double remaining(const Target& target) const;
    [[nodiscard]] Json path_point(double latitude, double longitude, double height) const;
    asio::awaitable<void> progress(bool takeoff, code::FlyStatus status, Target target);
    [[nodiscard]] bool progress_due();
    void notify_joystick(code::JoystickInvalidReason reason);
    [[nodiscard]] bool drc_up() const;
    [[nodiscard]] bool manual_mode() const;
    // The first entry of points. Throws BadParameter if there is none.
    [[nodiscard]] Target parse_point(const Json& data, double speed) const;
    // flight_safety_advance_check of takeoff_to_point asks for this check.
    [[nodiscard]] bool restricted(double latitude, double longitude) const;

    sim::DockContext& ctx_;
    const wayline::MissionEngine& mission_;
    wayline::Download download_;
    media::MediaUploads& uploads_;
    sim::TaskGroup flights_;
    sim::TaskGroup::Handle flight_;
    Phase phase_ = Phase::idle;
    bool airborne_ = false;
    bool low_battery_done_ = false;
    code::DroneMode landing_mode_ = code::DroneMode::auto_landing;
    std::string flight_id_;
    std::string track_id_;
    std::string fly_to_id_;
    Target target_;
    Poi poi_;
    double poi_radius_ = poi_min_radius_m;
    double poi_speed_ = poi_default_speed;
    double last_poi_notify_ = -std::numeric_limits<double>::infinity();
    InFlight inflight_;
    std::uint32_t photo_seq_ = 0;
    // Metres above the dock.
    double security_height_ = 20.0;
    Sticks sticks_;
    double sticks_until_ = 0.0;
    double last_progress_ = -std::numeric_limits<double>::infinity();
    double last_joystick_notify_ = -std::numeric_limits<double>::infinity();
    std::function<std::optional<code::Result>()> preflight_;
    std::function<bool(double, double)> forbidden_;
    std::function<bool()> observe_;
    bool safety_check_ = false;
};

}  // namespace docksim::flight
