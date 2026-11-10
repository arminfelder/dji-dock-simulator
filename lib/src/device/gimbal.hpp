#pragma once

#include "device/geo.hpp"

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <string_view>
#include <utility>

// Camera pointing for the payload-control services (Dock 2 DRC thing model).
// Not checked against firmware: OSD gimbal_yaw is the geographic heading of
// the camera, and a positive pitch_speed tilts the camera up.
namespace docksim::gimbal {

inline constexpr double pitch_min = -90.0;
inline constexpr double pitch_max = 35.0;
inline constexpr double zoom_max = 200.0;
inline constexpr double ir_zoom_max = 20.0;
// Approximate diagonal field of view of the thermal lens at 1x, degrees.
inline constexpr double ir_dfov = 61.0;

// height is metres above the dock. Angles are degrees: attitude_head clockwise
// from north, gimbal_pitch -90 at nadir, gimbal_yaw relative to attitude_head.
template <class T>
concept Aircraft = requires(T& a) {
    { a.latitude } -> std::convertible_to<double>;
    { a.longitude } -> std::convertible_to<double>;
    { a.height } -> std::convertible_to<double>;
    a.attitude_head = 0.0;
    a.gimbal_pitch = 0.0;
    a.gimbal_yaw = 0.0;
    a.zoom_factor = 0.0;
    a.ir_zoom_factor = 0.0;
    { a.camera_heading() } -> std::convertible_to<double>;
};

enum class Lens : std::uint8_t { wide, zoom, ir };

[[nodiscard]] constexpr Lens lens_from_name(std::string_view name) noexcept {
    if (name == "zoom") return Lens::zoom;
    if (name == "ir") return Lens::ir;
    return Lens::wide;
}

// locked turns the aircraft body, else only the gimbal.
void turn(Aircraft auto& a, double yaw_deg, bool locked) {
    if (locked) {
        a.attitude_head = std::fmod(std::fmod(a.attitude_head + yaw_deg, 360.0) + 360.0, 360.0);
    } else {
        a.gimbal_yaw = geo::signed_angle(a.gimbal_yaw + yaw_deg);
    }
}

void tilt(Aircraft auto& a, double pitch_deg) {
    a.gimbal_pitch = std::clamp(pitch_deg, pitch_min, pitch_max);
}

// camera_screen_drag. Speeds are rad/s, applied for the given time.
void drag(Aircraft auto& a, double pitch_speed, double yaw_speed, bool locked, double seconds) {
    tilt(a, a.gimbal_pitch + geo::degrees(pitch_speed * seconds));
    turn(a, geo::degrees(yaw_speed * seconds), locked);
}

// camera_aim. Turn so that the point x, y (0..1 from top left) moves to the centre.
void aim(Aircraft auto& a, double x, double y, bool locked, double dfov_deg,
         std::pair<int, int> frame) {
    const auto [width, height] = frame;
    const double half_diag = std::tan(geo::radians(dfov_deg) / 2.0);
    const double diag = std::hypot(width, height);
    const double dx = (x - 0.5) * 2.0 * half_diag * width / diag;
    const double dy = (y - 0.5) * 2.0 * half_diag * height / diag;
    tilt(a, a.gimbal_pitch - geo::degrees(std::atan(dy)));
    turn(a, geo::degrees(std::atan(dx)), locked);
}

// camera_look_at. height is ellipsoidal in the API. The simulator compares it
// with home_altitude plus the aircraft height.
void look_at(Aircraft auto& a, double home_altitude, double latitude, double longitude,
             double height, bool locked) {
    const auto from = geo::point(a.latitude, a.longitude);
    const auto to = geo::point(latitude, longitude);
    const double target = units::in_degrees(geo::bearing(from, to));
    const double distance = units::in_metres(geo::haversine(from, to));
    const double rise = height - (home_altitude + a.height);
    tilt(a, geo::degrees(std::atan2(rise, std::max(distance, 0.1))));
    turn(a, geo::signed_angle(target - a.camera_heading()), locked);
}

// gimbal_reset. 0 recentre, 1 recentre and look down, 2 yaw only, 3 pitch down only.
void reset(Aircraft auto& a, int mode) {
    if (mode == 0 || mode == 2) a.gimbal_yaw = 0.0;
    if (mode == 0) a.gimbal_pitch = 0.0;
    if (mode == 1 || mode == 3) a.gimbal_pitch = -90.0;
    if (mode == 1) a.gimbal_yaw = 0.0;
}

// camera_focal_length_set.
void zoom(Aircraft auto& a, double factor, Lens lens) {
    if (lens == Lens::ir) {
        a.ir_zoom_factor = std::clamp(factor, 1.0, ir_zoom_max);
    } else {
        a.zoom_factor = std::clamp(factor, 1.0, zoom_max);
    }
}

// Diagonal field of view of a lens at its current zoom, degrees.
[[nodiscard]] double lens_dfov(const Aircraft auto& a, Lens lens, double wide_dfov) {
    double factor = 1.0;
    double base = wide_dfov;
    switch (lens) {
        case Lens::wide:
            return wide_dfov;
        case Lens::zoom:
            factor = a.zoom_factor;
            break;
        case Lens::ir:
            factor = a.ir_zoom_factor;
            base = ir_dfov;
            break;
    }
    return geo::degrees(2.0 *
                        std::atan(std::tan(geo::radians(base) / 2.0) / std::max(factor, 1.0)));
}

}  // namespace docksim::gimbal
