#pragma once

#include "util/units.hpp"

namespace docksim::geo {

// Mean earth radius (IUGG), metres.
inline constexpr double earth_radius_m = 6371008.8;

struct LatLon {
    units::Degrees lat;
    units::Degrees lon;

    friend constexpr bool operator==(const LatLon&, const LatLon&) = default;
};

[[nodiscard]] constexpr LatLon point(double lat_deg, double lon_deg) noexcept {
    return {.lat = lat_deg * units::degree, .lon = lon_deg * units::degree};
}

[[nodiscard]] double radians(double degrees) noexcept;
[[nodiscard]] double degrees(double radians) noexcept;

// Great-circle distance.
[[nodiscard]] units::Metres haversine(LatLon from, LatLon to) noexcept;

// Initial bearing, clockwise from north, 0..360.
[[nodiscard]] units::Degrees bearing(LatLon from, LatLon to) noexcept;

// The point that is distance from the start towards the target. The target, if it is nearer.
[[nodiscard]] LatLon move_towards(LatLon from, LatLon to, units::Metres distance) noexcept;

// Wrap an angle to -180..180. DJI reports yaw in this range.
[[nodiscard]] double signed_angle(double degrees) noexcept;

}  // namespace docksim::geo
