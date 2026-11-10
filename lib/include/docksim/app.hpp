#pragma once

#include <docksim/config.hpp>

#include <optional>
#include <string>

namespace docksim {

struct RunOptions {
    // Log every MQTT message in full (very verbose).
    bool log_wire = false;
};

// Connect to the broker and run one simulated dock until SIGINT or SIGTERM.
// Returns the process exit code.
[[nodiscard]] int run_dock(const Config& cfg, const RunOptions& options);

// The result of the backend login: MQTT credentials and the organization.
struct Provisioning {
    std::string auth_token;
    std::string mqtt_username;
    std::string mqtt_password;
    std::optional<std::string> mqtt_addr;
    std::optional<std::string> organization_id;
    std::optional<std::string> organization_name;
    std::optional<std::string> binding_code;
};

// POST auth_path for a session token, then GET org_path for the MQTT
// credentials and the binding code. The DJI sample login is not usable on this
// backend: the backend answers 406.
// Throws std::runtime_error.
[[nodiscard]] Provisioning provision(const HttpConfig& cfg);

void apply(Config& cfg, const Provisioning& p);

// Set the level of every docksim logger.
void set_log_level(LogLevel level, bool log_wire);

}  // namespace docksim
