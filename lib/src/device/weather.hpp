#pragma once

#include "device/state.hpp"
#include "protocol/codes.hpp"

#include <optional>

// Effects of the weather on the dock and the aircraft. The limits are DJI
// specification values, not values read from the dock firmware.
namespace docksim::device {

// Below this satellite count the aircraft flies in ATTI mode and drifts with the wind.
inline constexpr int atti_gps_number = 6;
// The aircraft cannot fly slower than this part of the commanded speed into a headwind.
inline constexpr double min_ground_speed_ratio = 0.3;
// Wind speed at which the drain is 1.5 times the calm drain, metres per second.
inline constexpr double reference_wind = 12.0;

[[nodiscard]] std::optional<code::Result> weather_block(const Weather& weather,
                                                        const WeatherConfig& limits);

// 1.0 in calm air and in noise mode.
[[nodiscard]] double drain_factor(const Weather& weather);

// Speed over ground on course for a commanded speed. Equal to speed in calm air.
// A tailwind does not make the aircraft go faster than the commanded speed.
[[nodiscard]] double ground_speed(const Weather& weather, double course_deg, double speed);

// Metres north and east that the wind moves an aircraft without satellite fix in seconds.
struct Drift {
    double north = 0.0;
    double east = 0.0;
};
[[nodiscard]] std::optional<Drift> drift(const Weather& weather, const Gnss& gnss, double seconds);

// Move the aircraft by the drift of one step.
void apply_drift(DroneState& drone, const Weather& weather, double seconds);

}  // namespace docksim::device
