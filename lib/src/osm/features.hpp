#pragma once

#include "osm/footprints.hpp"
#include "util/json.hpp"

#include <cstdint>
#include <optional>
#include <string_view>

// Ground floor features from OSM nodes: entrances on building outlines, and
// shops or cafés inside buildings. The facade shader draws doors and shopfronts there.
namespace docksim::osm {

// entrance=no marks a place without a door, so it is left out. amenity values
// with a shopfront, not every amenity (a bench has none).
inline constexpr std::string_view features_query =
    "[out:json][timeout:25][maxsize:16777216];("
    "node[\"entrance\"][\"entrance\"!=\"no\"]({s},{w},{n},{e});"
    "node[\"shop\"]({s},{w},{n},{e});"
    "node[\"amenity\"~\"^(restaurant|cafe|bar|pub|fast_food|ice_cream|pharmacy|bank)$\"]({s},{w},{"
    "n},{e});"
    ");out;";

struct Feature {
    enum class Kind : std::uint8_t { entrance, shop };
    Kind kind = Kind::entrance;
    LatLon at;

    friend bool operator==(const Feature&, const Feature&) = default;
};

// Overpass node to the cache form {kind, lat, lon}. Empty for anything else.
[[nodiscard]] std::optional<Json> simplify_feature(const Json& element);
// Cache form to a feature. Empty if the entry is broken.
[[nodiscard]] std::optional<Feature> feature_from_cache(const Json& item);

}  // namespace docksim::osm
