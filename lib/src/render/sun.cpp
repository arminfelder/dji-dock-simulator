#include "render/sun.hpp"

#include "device/geo.hpp"
#include "util/numeric.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <format>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace docksim::render {

using geo::degrees;
using geo::radians;

SunPosition sun_position(double lat, double lon, sim::WallTime when) {
    const double seconds = std::chrono::duration<double>(when.time_since_epoch()).count();
    const double n = seconds / 86400.0 + 2440587.5 - 2451545.0;
    const double mean_lon = num::floor_mod(280.460 + 0.9856474 * n, 360.0);
    const double anomaly = radians(num::floor_mod(357.528 + 0.9856003 * n, 360.0));
    const double ecl_lon =
        radians(mean_lon + 1.915 * std::sin(anomaly) + 0.020 * std::sin(2 * anomaly));
    const double obliquity = radians(23.439 - 0.0000004 * n);
    const double ra = std::atan2(std::cos(obliquity) * std::sin(ecl_lon), std::cos(ecl_lon));
    const double dec = std::asin(std::sin(obliquity) * std::sin(ecl_lon));
    const double gmst = num::floor_mod(18.697374558 + 24.06570982441908 * n, 24.0);
    const double hour_angle = radians(gmst * 15.0 + lon) - ra;
    const double phi = radians(lat);
    const double elevation = std::asin(std::sin(phi) * std::sin(dec) +
                                       std::cos(phi) * std::cos(dec) * std::cos(hour_angle));
    const double azimuth =
        std::atan2(-std::sin(hour_angle),
                   std::tan(dec) * std::cos(phi) - std::sin(phi) * std::cos(hour_angle));
    return {.elevation = degrees(elevation),
            .azimuth = num::floor_mod(degrees(azimuth) + 360.0, 360.0)};
}

Vec3 sun_vector(const SunPosition& sun) {
    const double el = radians(sun.elevation);
    const double az = radians(sun.azimuth);
    return {static_cast<float>(std::sin(az) * std::cos(el)),
            static_cast<float>(std::cos(az) * std::cos(el)), static_cast<float>(std::sin(el))};
}

std::optional<sim::WallTime> parse_time(const std::string& text) {
    // YYYY-MM-DD[T ]HH:MM[:SS[.fff]][Z|+HH:MM|+HHMM|-HH:MM|-HHMM]
    std::string s = text;
    if (s.size() > 10 && s[10] == ' ') s[10] = 'T';
    if (s.ends_with('Z')) s.pop_back();
    static constexpr std::array<const char*, 6> formats{"%FT%T%Ez", "%FT%T%z", "%FT%T",
                                                        "%FT%R%Ez", "%FT%R%z", "%FT%R"};
    for (const char* format : formats) {
        std::istringstream in(s);
        std::chrono::sys_time<std::chrono::nanoseconds> t;
        in >> std::chrono::parse(std::string(format), t);
        if (!in.fail() && in.peek() == std::char_traits<char>::eof()) {
            return sim::WallTime{std::chrono::floor<std::chrono::seconds>(t)};
        }
    }
    return std::nullopt;
}

LightingClock::LightingClock(const std::optional<std::string>& setting, const std::string& timezone,
                             bool frozen, sim::WallTime now)
    : started_(now), frozen_(frozen) {
    if (!setting || setting->empty()) return;
    std::string text = *setting;
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back())) != 0)
        text.pop_back();
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front())) != 0)
        text.erase(0, 1);
    static const std::regex hhmm{R"((\d{1,2}):(\d{2})(?::(\d{2}))?)"};
    std::smatch m;
    if (std::regex_match(text, m, hhmm)) {
        const auto* zone = std::chrono::locate_zone(timezone);
        const auto local = zone->to_local(std::chrono::floor<std::chrono::seconds>(now));
        const auto today = std::chrono::floor<std::chrono::days>(local);
        const auto wanted = today + std::chrono::hours{std::stoi(m[1].str())} +
                            std::chrono::minutes{std::stoi(m[2].str())} +
                            std::chrono::seconds{m[3].matched ? std::stoi(m[3].str()) : 0};
        base_ = zone->to_sys(wanted, std::chrono::choose::earliest);
        return;
    }
    base_ = parse_time(text);
    if (!base_) {
        throw std::invalid_argument(
            std::format("media.lighting_time {} is neither HH:MM nor ISO 8601", text));
    }
}

sim::WallTime LightingClock::at(sim::WallTime real) const {
    if (!base_) return real;
    if (frozen_) return *base_;
    return *base_ + (real - started_);
}

}  // namespace docksim::render
