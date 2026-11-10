#pragma once

#include "device/geo.hpp"
#include "render/vec.hpp"
#include "sim/clock.hpp"

#include <array>
#include <memory>
#include <utility>
#include <vector>

// Camera pose, view rays and the ground distance a view can show.
namespace docksim::render {

// Ground farther than this from the camera shows as haze.
inline constexpr double max_range_m = 5000.0;

struct Pose {
    geo::LatLon position{};
    // A dock on a roof puts the home point on the roof.
    units::HeightAboveHome height{};
    geo::LatLon home{};
    // Clockwise from north.
    units::Degrees heading{};
    // -90 is nadir, 0 is the horizon.
    units::Degrees gimbal_pitch = -90.0 * units::degree;
    sim::WallTime time;
};

// Unit rays in camera axes (right, down, forward), row-major, width * height.
// Cached per size and field of view (rounded to 0.01 degrees).
[[nodiscard]] std::shared_ptr<const std::vector<Vec3>> camera_rays(int width, int height,
                                                                   double dfov_deg);

// Rows map camera right, down and forward to east-north-up.
using Rotation = std::array<float, 9>;
[[nodiscard]] Rotation rotation(double heading_deg, double pitch_deg);

[[nodiscard]] inline Vec3 to_world(const Vec3& camera, const Rotation& m) noexcept {
    return {camera.x * m[0] + camera.y * m[3] + camera.z * m[6],
            camera.x * m[1] + camera.y * m[4] + camera.z * m[7],
            camera.x * m[2] + camera.y * m[5] + camera.z * m[8]};
}

// Horizontal distance that the view can show. Steep views need little terrain.
[[nodiscard]] double reach(double max_ray_up, double height);

// Angle of one pixel in radians, 1 / focal length in pixels.
[[nodiscard]] double pixel_angle(int width, int height, double dfov_deg);

// Camera position in metres east/north of the home point. Facade patterns add
// it, so that they stay fixed on the buildings while the camera moves.
[[nodiscard]] std::pair<float, float> window_anchor(const Pose& pose);

}  // namespace docksim::render
