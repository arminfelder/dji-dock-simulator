#include "osm/footprints.hpp"

#include "log/log.hpp"
#include "osm/facades.hpp"
#include "osm/tags.hpp"
#include "util/numeric.hpp"

#include <algorithm>
#include <cmath>
#include <regex>
#include <string_view>
#include <vector>

namespace docksim::osm {

namespace {

bool closed(const Json& ring) {
    return ring.size() >= 4 && ring.front() == ring.back();
}

// OSM may split one ring of a multipolygon into several ways. Join the open pieces end to end,
// reversed where needed, until each ring closes. Shared nodes have the same coordinates, so the
// points compare exactly. A piece that does not close is dropped.
std::vector<Json> assemble_rings(std::vector<Json> pieces, const Json& id, std::string_view role) {
    std::vector<Json> rings;
    std::vector<Json> open;
    for (auto& p : pieces) {
        if (closed(p)) {
            rings.push_back(std::move(p));
        } else if (p.size() >= 2) {
            open.push_back(std::move(p));
        }
    }
    std::size_t dropped = 0;
    while (!open.empty()) {
        Json ring = std::move(open.back());
        open.pop_back();
        bool joined = true;
        while (!closed(ring) && joined) {
            joined = false;
            for (auto it = open.begin(); it != open.end(); ++it) {
                const Json& piece = *it;
                if (ring.back() == piece.front()) {
                    for (std::size_t i = 1; i < piece.size(); ++i) ring.push_back(piece[i]);
                } else if (ring.back() == piece.back()) {
                    for (std::size_t i = piece.size() - 1; i-- > 0;) ring.push_back(piece[i]);
                } else {
                    continue;
                }
                open.erase(it);
                joined = true;
                break;
            }
        }
        if (closed(ring)) {
            rings.push_back(std::move(ring));
        } else {
            ++dropped;
        }
    }
    if (dropped > 0) {
        log::get("dock_sim.osm")
            ->debug("relation {}: {} {} pieces do not close, dropped", id.dump(), dropped, role);
    }
    return rings;
}

Ring ring_from(const Json& points) {
    Ring ring;
    if (!points.is_array()) return ring;
    for (const auto& p : points) {
        if (p.is_array() && p.size() >= 2) ring.push_back({p[0].get<double>(), p[1].get<double>()});
    }
    return ring;
}

std::vector<Ring> rings_from(const Json& item, const char* key) {
    std::vector<Ring> out;
    if (!item.contains(key) || !item[key].is_array()) return out;
    for (const auto& r : item[key]) out.push_back(ring_from(r));
    return out;
}

}  // namespace

std::optional<double> parse_metres(std::string_view text) {
    static const std::regex number{R"([-+]?\d+(?:[.,]\d+)?)"};
    if (text.empty()) return std::nullopt;
    std::match_results<std::string_view::const_iterator> m;
    if (!std::regex_search(text.begin(), text.end(), m, number)) return std::nullopt;
    std::string digits = m.str();
    std::ranges::replace(digits, ',', '.');
    const auto value = num::parse_double(digits);
    if (!value) return std::nullopt;
    const bool feet =
        text.find("ft") != std::string_view::npos || text.find('\'') != std::string_view::npos;
    return feet ? *value * foot_m : *value;
}

std::optional<double> explicit_height(const Json& tags) {
    for (const auto* key : {"height", "building:height"}) {
        const auto value = parse_metres(tag_or(tags, key, ""));
        if (value && *value > 0) return value;
    }
    return std::nullopt;
}

double height_from_tags(const Json& tags, double default_height, double level_height) {
    if (const auto given = explicit_height(tags)) return *given;
    if (const auto levels = parse_metres(tag_or(tags, "building:levels", ""));
        levels && *levels > 0) {
        const double roof = parse_metres(tag_or(tags, "roof:levels", "")).value_or(0.0);
        return (*levels + roof) * level_height + roof_slab_m;
    }
    const auto kind = tag(tags, "building");
    if (kind == "roof" || kind == "carport") return roof_only_m;
    return default_height;
}

std::optional<Roof> roof_from_tags(const Json& tags, double level_height) {
    const auto shape = roof_shape(tag_or(tags, "roof:shape", ""));
    if (!shape) return std::nullopt;
    auto height = parse_metres(tag_or(tags, "roof:height", ""));
    if (!height || *height == 0) {
        const auto levels = parse_metres(tag_or(tags, "roof:levels", ""));
        height = levels && *levels != 0 ? std::optional(*levels * level_height) : std::nullopt;
    }
    return Roof{
        .shape = *shape, .height = height, .across = tag(tags, "roof:orientation") == "across"};
}

bool is_part(const Json& tags) {
    return has_tag(tags, "building:part") && tag_or(tags, "building", "no") == "no";
}

std::int64_t stable_id(const Json& item) {
    if (item.contains("id") && item["id"].is_number()) return item["id"].get<std::int64_t>();
    double lat = 0;
    double lon = 0;
    for (const auto& ring : rings_from(item, "outer")) {
        if (!ring.empty()) {
            lat = ring.front().lat;
            lon = ring.front().lon;
            break;
        }
    }
    return static_cast<std::int64_t>(std::abs(lat * 1e7) + std::abs(lon * 1e7));
}

Json simplify(const Json& element) {
    Json outer = Json::array();
    Json inner = Json::array();
    const auto ring = [](const Json& geometry) {
        Json r = Json::array();
        if (geometry.is_array()) {
            for (const auto& p : geometry)
                r.push_back(Json::array({p.value("lat", 0.0), p.value("lon", 0.0)}));
        }
        return r;
    };
    if (element.value("type", "") == "way") {
        outer.push_back(ring(element.value("geometry", Json::array())));
    } else {
        std::vector<Json> outer_pieces;
        std::vector<Json> inner_pieces;
        for (const auto& member : element.value("members", Json::array())) {
            auto r = ring(member.value("geometry", Json::array()));
            (member.value("role", "") == "inner" ? inner_pieces : outer_pieces)
                .push_back(std::move(r));
        }
        const auto id = element.value("id", Json(0));
        for (auto& r : assemble_rings(std::move(outer_pieces), id, "outer"))
            outer.push_back(std::move(r));
        for (auto& r : assemble_rings(std::move(inner_pieces), id, "inner"))
            inner.push_back(std::move(r));
    }
    return {{"id", element.value("id", Json(0))},
            {"tags", element.value("tags", Json::object())},
            {"outer", std::move(outer)},
            {"inner", std::move(inner)}};
}

Footprint from_cache(const Json& item, const FootprintOptions& options) {
    const Json tags = item.value("tags", Json::object());
    const auto facade = facade_from_tags(tags, stable_id(item), options.default_style);
    double level_height = options.level_height;
    if (facade.windows == Windows::classic) level_height = std::max(level_height, classic_level_m);
    const auto roof = roof_from_tags(tags, level_height);

    std::optional<double> wall;
    double height = 0;
    const bool has_id = item.contains("id") && item["id"].is_number_integer();
    const auto override =
        has_id ? options.overrides.find(item["id"].get<std::int64_t>()) : options.overrides.end();
    if (override != options.overrides.end()) {
        // An override of 0 is valid: it removes a building that OSM tags wrongly.
        height = override->second;
    } else if (roof && !explicit_height(tags)) {
        const auto levels = parse_metres(tag_or(tags, "building:levels", ""));
        wall =
            levels && *levels != 0 ? *levels * level_height + roof_slab_m : options.default_height;
        const double roof_h = roof->height.value_or(0.0);
        height = *wall + (roof_h != 0.0 ? roof_h : default_roof_m);
    } else {
        height = height_from_tags(tags, options.default_height, level_height);
    }
    const bool part = is_part(tags);
    return {
        .id = stable_id(item),
        .height = height,
        .outer = rings_from(item, "outer"),
        .inner = rings_from(item, "inner"),
        .style = styles().id(facade),
        .part = part,
        .roof = roof,
        .wall = wall,
        .plain_part = part && lower(tag_or(tags, "building:part", "yes")) == "yes",
        .own_look = has_tag(tags, "building:colour") || has_tag(tags, "colour") ||
                    has_tag(tags, "building:material"),
        .bottom = std::max(0.0, parse_metres(tag_or(tags, "min_height", "")).value_or(0.0)),
        .guessed = !explicit_height(tags) && !roof && !part,
        .levels = static_cast<int>(std::lround(
            std::max(0.0, parse_metres(tag_or(tags, "building:levels", "")).value_or(0.0)))),
    };
}

}  // namespace docksim::osm
