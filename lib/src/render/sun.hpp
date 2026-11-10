#pragma once

#include "render/vec.hpp"
#include "sim/clock.hpp"

#include <optional>
#include <string>
#include <utility>

namespace docksim::render {

struct SunPosition {
    // Degrees.
    double elevation;
    // Degrees clockwise from north, 0..360.
    double azimuth;
};

// Low-precision almanac method, error below 0.5 degrees.
[[nodiscard]] SunPosition sun_position(double lat, double lon, sim::WallTime when);

// Unit vector towards the sun in east-north-up.
[[nodiscard]] Vec3 sun_vector(const SunPosition& sun);

// Maps real time to the time used for the sun position. The lighting time
// starts at the configured value when the clock is made, then runs on with
// real time unless it is frozen.
class LightingClock {
public:
    // setting: empty is the real time; "HH:MM[:SS]" is that time today in
    // timezone; anything else is ISO 8601. Throws std::invalid_argument.
    LightingClock(const std::optional<std::string>& setting, const std::string& timezone,
                  bool frozen, sim::WallTime now = std::chrono::system_clock::now());

    [[nodiscard]] sim::WallTime at(sim::WallTime real) const;
    [[nodiscard]] sim::WallTime now() const { return at(std::chrono::system_clock::now()); }
    [[nodiscard]] const std::optional<sim::WallTime>& base() const { return base_; }
    [[nodiscard]] bool frozen() const { return frozen_; }

private:
    sim::WallTime started_;
    bool frozen_;
    std::optional<sim::WallTime> base_;
};

// ISO 8601 date-time; no offset means UTC. Empty if the text does not parse.
[[nodiscard]] std::optional<sim::WallTime> parse_time(const std::string& text);

}  // namespace docksim::render
