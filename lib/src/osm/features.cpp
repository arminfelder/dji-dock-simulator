#include "osm/features.hpp"

namespace docksim::osm {

std::optional<Json> simplify_feature(const Json& element) {
    if (element.value("type", "") != "node" || !element.contains("lat") || !element.contains("lon"))
        return std::nullopt;
    const auto tags = element.value("tags", Json::object());
    // A shop node on the outline is also its entrance; the door is the better guess there.
    const char* kind = tags.contains("entrance") ? "entrance" : "shop";
    return Json{{"kind", kind}, {"lat", element["lat"]}, {"lon", element["lon"]}};
}

std::optional<Feature> feature_from_cache(const Json& item) {
    if (!item.is_object() || !item.contains("lat") || !item.contains("lon")) return std::nullopt;
    if (!item["lat"].is_number() || !item["lon"].is_number()) return std::nullopt;
    const auto kind = item.value("kind", "");
    if (kind != "entrance" && kind != "shop") return std::nullopt;
    return Feature{.kind = kind == "entrance" ? Feature::Kind::entrance : Feature::Kind::shop,
                   .at = {.lat = item["lat"].get<double>(), .lon = item["lon"].get<double>()}};
}

}  // namespace docksim::osm
