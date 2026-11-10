#pragma once

#include "osm/roofs.hpp"
#include "util/json.hpp"

#include <docksim/config.hpp>

#include <cstdint>
#include <map>
#include <optional>
#include <string_view>
#include <vector>

// Building footprints from Overpass elements: rings, height, roof and facade style.
namespace docksim::osm {

struct LatLon {
    double lat = 0;
    double lon = 0;

    friend bool operator==(const LatLon&, const LatLon&) = default;
};
using Ring = std::vector<LatLon>;

inline constexpr double foot_m = 0.3048;
// Height of a roof-only structure (carport, canopy) without tags, metres.
inline constexpr double roof_only_m = 5.0;
// Added to levels x level height for the roof slab and parapet, metres.
inline constexpr double roof_slab_m = 1.0;
// Roof height assumed before the footprint size is known, metres.
inline constexpr double default_roof_m = 3.0;

struct Footprint {
    // OSM way or relation id. Negative for the extras.
    std::int64_t id = 0;
    // Top of the building, roof included, metres.
    double height = 0;
    std::vector<Ring> outer;
    std::vector<Ring> inner;
    // Facade style id in osm::styles().
    int style = 0;
    // A building:part. Near the camera parts replace the outline that holds them.
    bool part = false;
    std::optional<Roof> roof;
    // Top of the walls, when the tags give levels and no height; the roof sits on it.
    std::optional<double> wall;
    // A building:part=yes: its type is unknown, so it takes the window pattern
    // and weathering of its outline. own_look: it has its own colour or material.
    bool plain_part = false;
    bool own_look = false;
    // Bottom of a raised part (min_height), metres above the ground.
    double bottom = 0;
    // Height and roof are guesses (no height tag, no roof:shape, not a part):
    // a measured source such as the laser scan may replace them.
    bool guessed = false;
    // building:levels: floors from the ground to the eaves, also for a raised part. 0: not tagged.
    int levels = 0;
};

// First number in an OSM value. "12 m" and "12,5" work; feet are converted.
[[nodiscard]] std::optional<double> parse_metres(std::string_view text);
[[nodiscard]] std::optional<double> explicit_height(const Json& tags);
[[nodiscard]] double height_from_tags(const Json& tags, double default_height, double level_height);
// Empty if the roof is flat or of an unknown shape.
[[nodiscard]] std::optional<Roof> roof_from_tags(const Json& tags, double level_height);
[[nodiscard]] bool is_part(const Json& tags);

// OSM id; cache files from before ids were stored use the first vertex.
[[nodiscard]] std::int64_t stable_id(const Json& item);

// Overpass element to the cache form: {id, tags, outer, inner}, rings of [lat, lon].
[[nodiscard]] Json simplify(const Json& element);

struct FootprintOptions {
    double default_height = 8.0;
    double level_height = 3.0;
    // Height in metres per OSM id. It wins over the tags; 0 removes a building.
    std::map<std::int64_t, double> overrides;
    FacadeStyle default_style = FacadeStyle::classic;
};

// A cache item ({id, tags, outer, inner}) to a footprint. Registers its facade in osm::styles().
[[nodiscard]] Footprint from_cache(const Json& item, const FootprintOptions& options);

}  // namespace docksim::osm
