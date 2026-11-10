#include "render/rain.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace docksim::render {

namespace {

struct Level {
    int drops;
    double fall;
    // Part of the way to the grey haze colour.
    double haze;
    // Peak alpha of a drop next to the lens, 0..255.
    double alpha;
};

constexpr std::array<Level, 4> levels{{
    {.drops = 0, .fall = 0.0, .haze = 0.0, .alpha = 0.0},
    {.drops = 2500, .fall = 4.0, .haze = 0.10, .alpha = 90.0},
    {.drops = 4000, .fall = 6.5, .haze = 0.22, .alpha = 95.0},
    {.drops = 9000, .fall = 8.5, .haze = 0.36, .alpha = 120.0},
}};

constexpr image::Rgb drop_colour{214, 220, 226};
constexpr image::Rgb haze_colour{132, 138, 145};

std::uint64_t mix(std::uint64_t x) {
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27U)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31U);
}

// 0..1 from a hash.
double unit(std::uint64_t h) {
    return static_cast<double>(h >> 11U) * 0x1.0p-53;
}

double wrap(double v, double lo, double size) {
    const double r = std::fmod(v - lo, size);
    return lo + (r < 0.0 ? r + size : r);
}

double radians(double deg) {
    return deg * std::numbers::pi / 180.0;
}

void haze(image::RgbImage& frame, double amount) {
    if (amount <= 0.0) return;
    // Rain light is flat: less contrast and less colour, a little darker.
    const int keep = static_cast<int>(std::lround((1.0 - amount) * 256.0));
    const int add = 256 - keep;
    auto bytes = frame.bytes();
    for (std::size_t i = 0; i + 2 < bytes.size(); i += 3) {
        const int grey = (bytes[i] * 77 + bytes[i + 1] * 150 + bytes[i + 2] * 29) >> 8;
        for (std::size_t c = 0; c < 3; ++c) {
            const int muted = (bytes[i + c] + grey) / 2;
            const int target = (haze_colour[c] * 2 + muted) / 3;
            bytes[i + c] = static_cast<std::uint8_t>((bytes[i + c] * keep + target * add) >> 8);
        }
    }
}

void streak(image::RgbImage& frame, double x0, double y0, double x1, double y1, int alpha,
            int width) {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy))));
    for (int s = 0; s <= steps; ++s) {
        const double f = static_cast<double>(s) / steps;
        // The tail fades, so that the streak reads as motion.
        const int a = static_cast<int>(alpha * (0.35 + 0.65 * f));
        const int x = static_cast<int>(std::lround(x0 + dx * f));
        const int y = static_cast<int>(std::lround(y0 + dy * f));
        frame.blend(x, y, drop_colour, a);
        if (width > 1) frame.blend(x + 1, y, drop_colour, a / 2);
    }
}

}  // namespace

double rain_fall_speed(int rainfall) {
    return levels[static_cast<std::size_t>(std::clamp(rainfall, 0, 3))].fall;
}

void apply_rain(image::RgbImage& frame, const RainScene& scene, double time_s) {
    if (scene.rainfall <= 0 || frame.empty()) return;
    const auto& level = levels[static_cast<std::size_t>(std::clamp(scene.rainfall, 1, 3))];
    haze(frame, level.haze);

    const double w = frame.width();
    const double h = frame.height();
    const double focal = std::hypot(w, h) / 2.0 / std::tan(radians(scene.dfov) / 2.0);

    // Camera axes in east, north, up: right, up and forward.
    const double hd = radians(scene.heading);
    const double pt = radians(scene.pitch);
    const std::array<double, 3> right{std::cos(hd), -std::sin(hd), 0.0};
    const std::array<double, 3> up{-std::sin(hd) * std::sin(pt), -std::cos(hd) * std::sin(pt),
                                   std::cos(pt)};
    const std::array<double, 3> fwd{std::sin(hd) * std::cos(pt), std::cos(hd) * std::cos(pt),
                                    std::sin(pt)};
    // Drop velocity relative to the camera.
    const std::array<double, 3> rel{scene.air_east - scene.camera_east,
                                    scene.air_north - scene.camera_north,
                                    -level.fall - scene.camera_up};
    const auto dot = [&](const std::array<double, 3>& a) {
        return a[0] * rel[0] + a[1] * rel[1] + a[2] * rel[2];
    };
    const double vx = dot(right);
    const double vy = dot(up);
    const double vz = dot(fwd);

    // The drops live in a box in front of the lens that just holds the far view.
    const double half_x = rain_far_m * (w / 2.0) / focal;
    const double half_y = rain_far_m * (h / 2.0) / focal;
    const double depth = rain_far_m - rain_near_m;
    const double max_len = h * 0.3;

    for (int i = 0; i < level.drops; ++i) {
        const auto seed = mix(static_cast<std::uint64_t>(i));
        const double X = wrap(-half_x + 2.0 * half_x * unit(seed) + vx * time_s, -half_x,
                              2.0 * half_x);
        const double Y = wrap(-half_y + 2.0 * half_y * unit(mix(seed)) + vy * time_s, -half_y,
                              2.0 * half_y);
        const double Z =
            wrap(rain_near_m + depth * unit(mix(seed + 1)) + vz * time_s, rain_near_m, depth);
        const double sx = focal * X / Z;
        const double sy = -focal * Y / Z;
        if (std::abs(sx) > w / 2.0 || std::abs(sy) > h / 2.0) continue;
        // Screen motion of the drop during the exposure.
        double mx = (focal * vx - sx * vz) / Z * rain_exposure_s;
        double my = (-focal * vy - sy * vz) / Z * rain_exposure_s;
        const double len = std::hypot(mx, my);
        if (len > max_len) {
            mx *= max_len / len;
            my *= max_len / len;
        }
        if (len < 1.5) {
            const double s = 1.5 / std::max(len, 1e-6);
            mx = len > 1e-6 ? mx * s : 0.0;
            my = len > 1e-6 ? my * s : 1.5;
        }
        const double nearness = 1.0 - (Z - rain_near_m) / depth;
        const int alpha = static_cast<int>(level.alpha * (0.25 + 0.75 * nearness * nearness));
        const double x1 = sx + w / 2.0;
        const double y1 = sy + h / 2.0;
        streak(frame, x1 - mx, y1 - my, x1, y1, alpha, Z < 2.0 ? 2 : 1);
    }
}

}  // namespace docksim::render
