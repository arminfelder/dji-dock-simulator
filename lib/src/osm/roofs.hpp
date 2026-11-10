#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

// Roof shapes from the OSM roof:* tags. A roof is a height field over the
// footprint: walls up to the eaves, then the roof surface. The ridge follows
// the long side of the minimum-area rectangle of the footprint, or the short
// side with roof:orientation=across.
namespace docksim::osm {

enum class RoofShape : std::uint8_t { gabled, hipped, pyramidal, skillion, dome };

// The drawn shape of a roof:shape value, also for close relatives. Empty for
// flat and for unknown values.
[[nodiscard]] std::optional<RoofShape> roof_shape(std::string_view value);

// Default pitch, when roof:height and roof:levels are missing.
inline constexpr double pitch_deg = 35.0;
inline constexpr double skillion_pitch_deg = 15.0;
// A roof takes at most this share of the building height, and walls stay this tall.
inline constexpr double max_roof_share = 0.6;
inline constexpr double min_wall_m = 2.0;

struct Roof {
    RoofShape shape = RoofShape::gabled;
    // Explicit roof height from roof:height or roof:levels, metres; empty derives it.
    std::optional<double> height;
    // roof:orientation=across: the ridge runs along the short side.
    bool across = false;

    friend bool operator==(const Roof&, const Roof&) = default;
};

using Point = std::pair<double, double>;

// Minimum-area rectangle: centre, unit axis along the long side, half sizes.
struct Box {
    double cx = 0;
    double cy = 0;
    double ux = 1;
    double uy = 0;
    double half_long = 0;
    double half_short = 0;
};

// Minimum-area bounding rectangle of 2-D points (rotating hull edges).
[[nodiscard]] Box min_area_box(std::span<const Point> points);

// Wall height (eaves) and roof height, metres. Give top (height tag: top of
// the roof) or wall (levels: top of the walls).
[[nodiscard]] std::pair<double, double> split_height(const Roof& roof, const Box& box,
                                                     std::optional<double> top,
                                                     std::optional<double> wall);

// Roof surface height at x, y (metres, same frame as the box).
[[nodiscard]] double roof_height(const Roof& roof, const Box& box, double x, double y,
                                 std::optional<double> top, std::optional<double> wall);

// The roof as the minimum of planes f = a * x + b * y + c; empty for a dome.
// roof_height() is wall + roof * clamp(f, 0, 1); inside the box f stays in
// 0 .. 1, so the minimum of these planes is the same surface.
[[nodiscard]] std::optional<std::vector<std::array<double, 3>>> roof_planes(const Roof& roof,
                                                                            const Box& box);

}  // namespace docksim::osm
