#include "device/weather.hpp"

#include "device/geo.hpp"

#include <algorithm>
#include <cmath>

namespace docksim::device {

namespace {

bool calm(const Weather& weather) {
    return weather.noise || weather.wind_speed <= 0.0;
}

}  // namespace

std::optional<code::Result> weather_block(const Weather& weather, const WeatherConfig& limits) {
    if (weather.noise) return std::nullopt;
    if (weather.rainfall >= limits.max_rainfall_takeoff) return code::Result::heavy_rain;
    if (weather.wind_speed >= limits.max_wind_takeoff) return code::Result::wind_too_high;
    return std::nullopt;
}

double drain_factor(const Weather& weather) {
    if (calm(weather)) return 1.0;
    const double ratio = weather.wind_speed / reference_wind;
    return 1.0 + 0.5 * ratio * ratio;
}

double ground_speed(const Weather& weather, double course_deg, double speed) {
    if (calm(weather)) return speed;
    // wind_direction is where the wind comes from. Headwind is positive.
    const double headwind =
        weather.wind_speed * std::cos(geo::radians(weather.wind_direction - course_deg));
    return std::clamp(speed - headwind, speed * min_ground_speed_ratio, speed);
}

std::optional<Drift> drift(const Weather& weather, const Gnss& gnss, double seconds) {
    if (calm(weather) || gnss.gps_number >= atti_gps_number) return std::nullopt;
    const double towards = geo::radians(weather.wind_direction + 180.0);
    const double distance = weather.wind_speed * seconds;
    return Drift{.north = distance * std::cos(towards), .east = distance * std::sin(towards)};
}

void apply_drift(DroneState& drone, const Weather& weather, double seconds) {
    const auto d = drift(weather, drone.gnss, seconds);
    if (!d) return;
    const double lat = geo::radians(drone.latitude);
    drone.latitude += geo::degrees(d->north / geo::earth_radius_m);
    drone.longitude += geo::degrees(d->east / (geo::earth_radius_m * std::cos(lat)));
}

}  // namespace docksim::device
