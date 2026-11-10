#include "device/geo.hpp"

#include "util/numeric.hpp"

#include <cmath>
#include <numbers>

namespace docksim::geo {

double radians(double degrees) noexcept {
    return degrees * (std::numbers::pi / 180.0);
}

double degrees(double radians) noexcept {
    return radians * (180.0 / std::numbers::pi);
}

units::Metres haversine(LatLon from, LatLon to) noexcept {
    const double p1 = radians(units::in_degrees(from.lat));
    const double p2 = radians(units::in_degrees(to.lat));
    const double dp = p2 - p1;
    const double dl = radians(units::in_degrees(to.lon) - units::in_degrees(from.lon));
    const double a =
        std::pow(std::sin(dp / 2), 2) + std::cos(p1) * std::cos(p2) * std::pow(std::sin(dl / 2), 2);
    return 2 * earth_radius_m * std::asin(std::sqrt(a)) * units::metre;
}

units::Degrees bearing(LatLon from, LatLon to) noexcept {
    const double p1 = radians(units::in_degrees(from.lat));
    const double p2 = radians(units::in_degrees(to.lat));
    const double dl = radians(units::in_degrees(to.lon) - units::in_degrees(from.lon));
    const double y = std::sin(dl) * std::cos(p2);
    const double x = std::cos(p1) * std::sin(p2) - std::sin(p1) * std::cos(p2) * std::cos(dl);
    return num::floor_mod(degrees(std::atan2(y, x)) + 360.0, 360.0) * units::degree;
}

LatLon move_towards(LatLon from, LatLon to, units::Metres distance) noexcept {
    const double total = units::in_metres(haversine(from, to));
    const double step = units::in_metres(distance);
    if (total <= 0.0 || step >= total) return to;
    const double fraction = step / total;
    const double lat1 = units::in_degrees(from.lat);
    const double lon1 = units::in_degrees(from.lon);
    return point(lat1 + (units::in_degrees(to.lat) - lat1) * fraction,
                 lon1 + (units::in_degrees(to.lon) - lon1) * fraction);
}

double signed_angle(double degrees) noexcept {
    return num::floor_mod(degrees + 180.0, 360.0) - 180.0;
}

}  // namespace docksim::geo
