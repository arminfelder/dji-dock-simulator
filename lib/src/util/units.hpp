#pragma once

#include <mp-units/framework.h>
#include <mp-units/systems/isq/space_and_time.h>
#include <mp-units/systems/si/units.h>

namespace docksim::units {

inline constexpr auto metre = mp_units::si::metre;
inline constexpr auto degree = mp_units::si::degree;

using Metres = mp_units::quantity<metre, double>;
using Degrees = mp_units::quantity<degree, double>;

inline constexpr struct home_point final : mp_units::absolute_point_origin<mp_units::isq::height> {
} home_point;

using HeightAboveHome = mp_units::quantity_point<mp_units::isq::height[metre], home_point, double>;

// A unit conversion can change the last bits of a value, and the golden tests compare exact values.
// Thus, these helpers read a value only in its stored unit. Do not convert degrees to radians here.
[[nodiscard]] constexpr double in_metres(Metres q) noexcept {
    return q.numerical_value_in(metre);
}

[[nodiscard]] constexpr double in_degrees(Degrees q) noexcept {
    return q.numerical_value_in(degree);
}

[[nodiscard]] constexpr HeightAboveHome above_home(double metres) noexcept {
    return home_point + metres * metre;
}

[[nodiscard]] constexpr double metres_above_home(HeightAboveHome h) noexcept {
    return h.quantity_from(home_point).numerical_value_in(metre);
}

}  // namespace docksim::units
