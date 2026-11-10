#pragma once

#include "control/wire_log.hpp"
#include "util/json.hpp"

#include <stdexcept>

namespace docksim::sim {
class DockSim;
class RealClock;
}  // namespace docksim::sim

// Instructor operations on one simulated dock. Every call must run on the
// executor of the dock. The API layer posts to it.
namespace docksim::control {

// The request has a bad or missing value. HTTP 400.
class BadRequest : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

// The state of the simulation does not allow the change now. HTTP 409.
class Conflict : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class SimControl {
public:
    // clock can be null: then the time scale cannot change.
    SimControl(sim::DockSim& dock, sim::RealClock* clock, WireLog& wire);

    [[nodiscard]] Json snapshot() const;

    // {latitude, longitude[, altitude]}. Only while the aircraft is in the dock.
    Json move_dock(const Json& body);
    // {latitude, longitude[, height]}. Height in metres above the dock. Only in flight.
    Json teleport_drone(const Json& body);
    // {percent}.
    Json set_battery(const Json& body);
    // Any of {wind_speed, wind_direction, rainfall, temperature, humidity}. Ends noise mode.
    Json set_weather(const Json& body);
    // Back to random calm weather.
    Json reset_weather();
    // target "dock" or "drone". Any of {gps_number, rtk_number, quality, fix}.
    Json set_gnss(const std::string& target, const Json& body);
    // {time_scale}.
    Json set_time_scale(const Json& body);
    // Any of {emergency_stop, cover_fault} as booleans.
    Json set_faults(const Json& body);
    // {code[, level, module, device, in_the_sky, imminent]}. Sends the hms event.
    Json raise_hms(const Json& body);
    Json clear_hms(const std::string& code);

    [[nodiscard]] Json wire(std::uint64_t since) const;
    [[nodiscard]] WireLog& wire_log() { return *wire_; }
    [[nodiscard]] sim::DockSim& dock() { return *dock_; }

private:
    [[nodiscard]] bool aircraft_busy() const;

    sim::DockSim* dock_;
    sim::RealClock* clock_;
    WireLog* wire_;
};

}  // namespace docksim::control
