#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

// KMZ wayline reader. Extracts the waypoint list from a DJI WPML archive.
namespace docksim::wayline {

struct Waypoint {
    int index = 0;
    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    std::optional<double> speed;
    bool take_photo = false;
    // Degrees, -90 is nadir. Empty keeps the gimbal pitch of the previous waypoint.
    std::optional<double> gimbal_pitch;
    // Degrees clockwise from north. Empty means the aircraft faces along the leg.
    std::optional<double> heading;
};

struct Wayline {
    std::vector<Waypoint> waypoints;
    double speed = 10.0;
    double takeoff_height = 30.0;

    [[nodiscard]] int photo_count() const;
};

// Throws std::runtime_error if the archive has no wayline or no waypoints.
[[nodiscard]] Wayline parse_kmz(std::span<const std::byte> blob);
[[nodiscard]] Wayline parse_wpml(std::string_view xml);

}  // namespace docksim::wayline
