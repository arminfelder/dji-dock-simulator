#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <string_view>

namespace docksim::num {

// Round to ndigits decimal places, on the decimal value, ties to even.
[[nodiscard]] double round_to(double x, int ndigits);

// Floored modulo: the result has the sign of the divisor. Wraps angles to [0, y).
[[nodiscard]] double floor_mod(double x, double y);

// Parse a decimal number. Surrounding whitespace, inf and nan are accepted.
[[nodiscard]] std::optional<double> parse_double(std::string_view text);

// UTC time as "2025-10-09T08:53:20+00:00". The DJI payloads use this form, not "Z".
[[nodiscard]] std::string iso8601_utc_seconds(std::chrono::system_clock::time_point t);

}  // namespace docksim::num
