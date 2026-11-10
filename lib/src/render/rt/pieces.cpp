#include "render/rt/pieces.hpp"

#include "device/geo.hpp"

#include <algorithm>
#include <cmath>

namespace docksim::render::rt {

namespace {

constexpr double cell_deg = osm::cell_deg;

int finest_zoom(const geodata::Terrain& terrain) {
    return std::max(terrain.max_zoom(), terrain.near_zoom());
}

}  // namespace

GroundFn ground_of(geodata::Terrain& terrain, int cy, int cx, double fallback) {
    const int zoom = finest_zoom(terrain);
    const double lat0 = cy * cell_deg - 0.002;
    const double lon0 = cx * cell_deg - 0.002;
    const double lat1 = (cy + 1) * cell_deg + 0.002;
    const double lon1 = (cx + 1) * cell_deg + 0.002;
    const auto [ax, ay] = raster::global_pixel(lat0, lon0, zoom);
    const auto [bx, by] = raster::global_pixel(lat1, lon1, zoom);
    const auto dem = terrain.mosaics().get(zoom, {.x0 = std::min(ax, bx),
                                                  .y0 = std::min(ay, by),
                                                  .x1 = std::max(ax, bx),
                                                  .y1 = std::max(ay, by)});
    return [dem, fallback](double lat, double lon) {
        if (!dem) return fallback;
        return static_cast<double>(
            geodata::elevation(*dem, lat, lon).value_or(static_cast<float>(fallback)));
    };
}

Mesh build_piece(const PieceKey& key, geodata::Terrain& terrain, osm::CellStore* cells,
                 const raster::LocalFrame& origin, float z0, raster::als::HeightAboveGround* scan) {
    if (const auto* t = std::get_if<TileKey>(&key)) {
        const int max_zoom = finest_zoom(terrain);
        constexpr double tile = raster::tile_px;
        const geodata::Bounds bounds{.x0 = t->tx * tile - 1,
                                     .y0 = t->ty * tile - 1,
                                     .x1 = (t->tx + 1) * tile + 1,
                                     .y1 = (t->ty + 1) * tile + 1};
        const auto dem = terrain.mosaics().get(std::min(t->z, max_zoom), bounds);
        if (!dem) return {};
        return terrain_tile(*dem, t->z, t->tx, t->ty, t->z == max_zoom ? near_quads : far_quads,
                            origin, z0);
    }
    if (cells == nullptr) return {};
    if (const auto* t = std::get_if<TreesKey>(&key)) {
        if (scan == nullptr) return {};
        const LatLonBounds bounds{.south = t->cy * cell_deg,
                                  .west = t->cx * cell_deg,
                                  .north = (t->cy + 1) * cell_deg,
                                  .east = (t->cx + 1) * cell_deg};
        const auto heights = scan->area(bounds.south, bounds.west, bounds.north, bounds.east);
        if (!heights) return {};
        const auto footprints = cells->footprints(t->cy, t->cx);
        if (!footprints) return {};
        return trees(*footprints, origin, z0, ground_of(terrain, t->cy, t->cx, z0), *heights,
                     bounds);
    }
    const auto& c = std::get<CellKey>(key);
    std::shared_ptr<const raster::als::HeightGrid> heights;
    if (c.near && scan != nullptr) {
        constexpr double margin = 0.0005;
        heights = scan->area(c.cy * cell_deg - margin, c.cx * cell_deg - margin,
                             (c.cy + 1) * cell_deg + margin, (c.cx + 1) * cell_deg + margin);
    }
    const auto footprints = cells->footprints(c.cy, c.cx);
    if (!footprints) return {};
    // Doors and shopfronts are too small to see on far cells; their queries would double the
    // Overpass load. A frame never waits for them: a cell without them shows plain ground floors.
    const auto features = c.near ? cells->cached_features(c.cy, c.cx) : nullptr;
    return buildings(*footprints, origin, z0, ground_of(terrain, c.cy, c.cx, z0), true,
                     heights.get(),
                     features ? std::span(*features) : std::span<const osm::Feature>{}, c.near);
}

float home_roof(geodata::Terrain& terrain, osm::CellStore& cells, const raster::LocalFrame& origin,
                float z0, raster::als::HeightAboveGround* scan) {
    const int cy = static_cast<int>(std::floor(origin.lat0 / cell_deg));
    const int cx = static_cast<int>(std::floor(origin.lon0 / cell_deg));
    const auto ground = ground_of(terrain, cy, cx, z0);
    std::shared_ptr<const raster::als::HeightGrid> heights;
    if (scan != nullptr) {
        constexpr double d = 0.0005;
        heights = scan->area(origin.lat0 - d, origin.lon0 - d, origin.lat0 + d, origin.lon0 + d);
    }
    const auto footprints = cells.footprints(cy, cx);
    if (!footprints) return 0.0F;
    return static_cast<float>(
        roof_height_at(*footprints, origin, z0, ground, origin.lat0, origin.lon0, heights.get()));
}

void curve(Mesh& mesh, const raster::LocalFrame& /*origin*/) {
    if (mesh.empty()) return;
    std::vector<double> drop(mesh.vertices.size());
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
        auto& v = mesh.vertices[i];
        drop[i] = (double{v[0]} * v[0] + double{v[1]} * v[1]) / (2 * geo::earth_radius_m);
        v[2] -= static_cast<float>(drop[i]);
    }
    for (std::size_t t = 0; t < mesh.indices.size(); ++t) {
        auto& r = mesh.records[t];
        if (r[record_kind] == kind_terrain) continue;
        const auto first = static_cast<float>(drop[mesh.indices[t][0]]);
        r[record_base] -= first;
        r[record_top] -= first;
    }
}

}  // namespace docksim::render::rt
