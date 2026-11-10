#include "render/shading.hpp"

#include "device/geo.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <span>

namespace docksim::render::shading {

namespace {

float clamp01(double v) {
    return static_cast<float>(std::clamp(v, 0.0, 1.0));
}

// Piecewise linear, clamped at both ends, like numpy.interp.
double interp(double x, std::span<const double> xs, std::span<const double> ys) {
    if (x <= xs.front()) return ys.front();
    if (x >= xs.back()) return ys.back();
    for (std::size_t i = 1; i < xs.size(); ++i) {
        if (x <= xs[i])
            return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
    }
    return ys.back();
}

}  // namespace

Light light_params(double sun_el, const Vec3& sun) {
    return {.day = clamp01(sun_el / 10.0), .flat = ambient + direct * std::max(sun.z, 0.0F)};
}

Sky sky_params(double sun_el, double sun_az) {
    const float day = clamp01((sun_el + 6.0) / 20.0);
    const float low = clamp01(1.0 - std::abs(sun_el) / 12.0);
    Vec3 horizon = Vec3{200, 215, 230} * day + Vec3{250, 160, 90} * (low * (1 - 0.5F * day));
    horizon = {std::clamp(horizon.x, 0.0F, 255.0F), std::clamp(horizon.y, 0.0F, 255.0F),
               std::clamp(horizon.z, 0.0F, 255.0F)};
    const Vec3 zenith = Vec3{70, 120, 200} * day + Vec3{30, 40, 80} * (low * (1 - day));
    const double psi = geo::radians(sun_az);
    return {.horizon = horizon,
            .zenith = zenith,
            .low = low,
            .sin_az = static_cast<float>(std::sin(psi)),
            .cos_az = static_cast<float>(std::cos(psi))};
}

Exposure exposure_params(double sun_el) {
    static constexpr std::array<double, 5> xs{-12, -6, 0, 10, 30};
    static constexpr std::array<double, 5> ys{0.05, 0.16, 0.55, 0.85, 1.0};
    const auto gain = static_cast<float>(interp(sun_el, xs, ys));
    const float warm = sun_el >= 0 ? clamp01(1.0 - sun_el / 15.0) : 0.0F;
    const float blue = clamp01(-sun_el / 6.0);
    const Vec3 balance{1 + 0.15F * warm - 0.20F * blue, 1 + 0.03F * warm - 0.05F * blue,
                       1 - 0.20F * warm + 0.10F * blue};
    return {.gain = balance * gain, .sigma = 1.5F + 8.0F * (1.0F - gain)};
}

}  // namespace docksim::render::shading
