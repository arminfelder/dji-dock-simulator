#include "util/numeric.hpp"

#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <format>
#include <limits>

namespace docksim::num {

double round_to(double x, int ndigits) {
    if (!std::isfinite(x)) return x;
    std::array<char, 400> buf{};
    const auto [end, ec] =
        std::to_chars(buf.data(), buf.data() + buf.size(), x, std::chars_format::fixed, ndigits);
    if (ec != std::errc{}) return x;
    double out = 0.0;
    std::from_chars(buf.data(), end, out);
    return out;
}

double floor_mod(double x, double y) {
    const double r = std::fmod(x, y);
    if (r == 0.0) return std::copysign(0.0, y);
    return (y < 0.0) != (r < 0.0) ? r + y : r;
}

std::optional<double> parse_double(std::string_view text) {
    const auto is_space = [](char c) { return std::isspace(static_cast<unsigned char>(c)) != 0; };
    while (!text.empty() && is_space(text.front())) text.remove_prefix(1);
    while (!text.empty() && is_space(text.back())) text.remove_suffix(1);
    if (text.empty()) return std::nullopt;

    double sign = 1.0;
    std::string_view body = text;
    if (body.front() == '+' || body.front() == '-') {
        if (body.front() == '-') sign = -1.0;
        body.remove_prefix(1);
    }
    // from_chars accepts its own '-', so a second sign must be refused here.
    if (body.empty() || body.front() == '+' || body.front() == '-') return std::nullopt;
    std::string lower;
    for (const char c : body)
        lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    if (lower == "inf" || lower == "infinity")
        return sign * std::numeric_limits<double>::infinity();
    if (lower == "nan") return std::numeric_limits<double>::quiet_NaN();

    double value = 0.0;
    const auto [ptr, ec] = std::from_chars(lower.data(), lower.data() + lower.size(), value);
    if (ec != std::errc{} || ptr != lower.data() + lower.size()) return std::nullopt;
    return sign * value;
}

std::string iso8601_utc_seconds(std::chrono::system_clock::time_point t) {
    return std::format("{:%Y-%m-%dT%H:%M:%S}+00:00", std::chrono::floor<std::chrono::seconds>(t));
}

}  // namespace docksim::num
