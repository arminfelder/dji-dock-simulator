#pragma once

#include "image/image.hpp"

// Falling rain over a rendered frame: streaks of drops near the lens and a grey haze.
// A post effect in image space; the 3D renderers do not know the weather.
namespace docksim::render {

struct RainScene {
    // RainfallEnum: 1 light, 2 moderate, 3 heavy. 0 draws nothing.
    int rainfall = 0;
    // Velocity of the air, metres per second towards east and north.
    double air_east = 0.0;
    double air_north = 0.0;
    // Velocity of the camera, metres per second.
    double camera_east = 0.0;
    double camera_north = 0.0;
    double camera_up = 0.0;
    // Degrees clockwise from north, and degrees with -90 at nadir.
    double heading = 0.0;
    double pitch = 0.0;
    // Diagonal field of view, degrees.
    double dfov = 84.0;
};

// Exposure of one frame for the motion blur of a drop, seconds.
inline constexpr double rain_exposure_s = 1.0 / 60.0;
// Drops are only visible this near to the lens, metres.
inline constexpr double rain_near_m = 0.6;
inline constexpr double rain_far_m = 14.0;

// time_s: any clock in seconds. Equal times give equal drops; the drops move with time.
void apply_rain(image::RgbImage& frame, const RainScene& scene, double time_s);

// Fall speed of the drops of this RainfallEnum, metres per second.
[[nodiscard]] double rain_fall_speed(int rainfall);

}  // namespace docksim::render
