#pragma once

#include "osm/cells.hpp"
#include "raster/als.hpp"
#include "raster/mercator.hpp"
#include "render/geodata/terrain.hpp"
#include "render/rt/meshes.hpp"

#include <compare>
#include <variant>

// One piece of the ray tracing scene: a terrain tile, the buildings of one OSM
// cell, or the trees of one cell. Each becomes one bottom-level structure.
namespace docksim::render::rt {

// Terrain tiles per side: the camera tile at the finest DEM zoom gets more.
inline constexpr int near_quads = 128;
inline constexpr int far_quads = 64;

struct TileKey {
    int z;
    int tx;
    int ty;
    auto operator<=>(const TileKey&) const = default;
};

struct CellKey {
    int cy;
    int cx;
    // Roofs from the laser scan where there is one.
    bool near;
    auto operator<=>(const CellKey&) const = default;
};

struct TreesKey {
    int cy;
    int cx;
    auto operator<=>(const TreesKey&) const = default;
};

using PieceKey = std::variant<TileKey, CellKey, TreesKey>;

// The mesh of one piece, positions relative to origin and z0. cells and scan
// may be null: no buildings, no laser scan. Not curved yet; see curve().
[[nodiscard]] Mesh build_piece(const PieceKey& key, geodata::Terrain& terrain,
                               osm::CellStore* cells, const raster::LocalFrame& origin, float z0,
                               raster::als::HeightAboveGround* scan);

// Absolute ground height over one OSM cell, from the finest DEM; fallback
// where the DEM has no data.
[[nodiscard]] GroundFn ground_of(geodata::Terrain& terrain, int cy, int cx, double fallback);

// Roof height above z0 where the dock stands (the origin); 0 on open ground.
// The same roofs as the scene draws, so the dock stands on its building.
[[nodiscard]] float home_roof(geodata::Terrain& terrain, osm::CellStore& cells,
                              const raster::LocalFrame& origin, float z0,
                              raster::als::HeightAboveGround* scan);

// Lower far vertices for the curvature of the Earth, in place. Facade heights
// count from the wall foot, so base and top of building records move with it.
void curve(Mesh& mesh, const raster::LocalFrame& origin);

}  // namespace docksim::render::rt
