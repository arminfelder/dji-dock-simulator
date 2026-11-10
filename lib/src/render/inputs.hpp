#pragma once

#include "osm/grid.hpp"
#include "raster/grid.hpp"
#include "raster/mercator.hpp"
#include "render/geodata/tiles.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

// The data one frame reads: terrain, buildings and imagery, in camera-local
// metres. The CPU march/shade and the GPU uploads both take these. Producers
// (render/geodata, osm) hand out shared_ptr<const ...>, so a frame keeps its
// inputs alive while the loaders build the next ones.
namespace docksim::render {

// A DEM mosaic (values in metres, valid) with its map for the current frame.
// The mosaic is shared between frames; only the map is per frame.
struct DemView {
    std::shared_ptr<const geodata::Mosaic> mosaic;
    raster::PixelMap map;
};

// A fine DEM near the camera fading into a coarser one to the horizon.
struct Relief {
    DemView far;
    DemView near;
    // Metres, used where neither DEM has data.
    float fallback = 0.0F;
    float near_radius = 0.0F;
    // The blend runs over the last part of near_radius, from this fraction.
    float blend_start = 0.8F;
};

// One building grid with its map for the current frame. The map depends on
// the frame origin, so the renderer computes it per frame (LocalGrid::map).
struct GridView {
    const osm::LocalGrid* grid = nullptr;
    osm::GridMap map;
};

// The near grid wins where it covers a point; the far grid fills in beyond.
// The renderer keeps the osm::Buildings alive while a frame uses the view.
struct BuildingView {
    GridView near;
    GridView far;
};

// One orthophoto mosaic (rgb, valid) with its map for the current frame.
struct LevelView {
    std::shared_ptr<const geodata::Mosaic> mosaic;
    raster::PixelMap map;
    float zoom = 0;
};

// Up to four levels, finest first.
inline constexpr int max_levels = 4;
using Levels = std::vector<LevelView>;

}  // namespace docksim::render
