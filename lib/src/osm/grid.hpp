#pragma once

#include "osm/footprints.hpp"
#include "raster/grid.hpp"
#include "raster/mercator.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

// Building heights rasterised onto a north-up grid around a centre point.
namespace docksim::osm {

// Outline edges are drawn this far to each side, metres. It must be more than
// the distance inside the wall at which the renderer reads them (0.6 m).
inline constexpr double edge_band_m = 1.2;
// Edge rows fit in the upper 24 bits of the style channel.
inline constexpr std::uint32_t max_edges = (1U << 24U) - 1U;

// Grid cell of a point at east/north metres of a camera frame:
// gx = gx0 + east * bx, gy = gy0 - north * by. The render kernels read it so.
struct GridMap {
    float gx0 = 0;
    float gy0 = 0;
    float bx = 0;
    float by = 0;
};

struct WallSample {
    // Signed distance to the nearest outline edge, metres. 0 where no edge is near.
    float distance = 0;
    // Edge normal (east, north). Zero where no edge is near; the sign is arbitrary.
    float east = 0;
    float north = 0;
};

// Heights in metres above the terrain on a north-up grid around a centre point.
// Cell (gx, gy) has its centre at pixel(lat, lon) == (gx, gy).
struct LocalGrid {
    double lat_c = 0;
    double lon_c = 0;
    // Grid step in degrees.
    double step_lat = 0;
    double step_lon = 0;
    double resolution_m = 0;
    raster::Grid2D<float> height;
    // Low byte: facade style id + 1 (osm::styles()), 0 where there is no
    // building. Upper 24 bits: row + 1 in edges of the nearest outline edge.
    raster::Grid2D<std::uint32_t> style;
    // Outline edges as lines a * gx + b * gy = c in pixel() units, (a, b) of
    // unit length. The renderer puts wall hits exactly on them.
    std::vector<std::array<float, 3>> edges;
    // Highest cell, metres above the terrain.
    float top = 0;

    [[nodiscard]] int width() const noexcept { return height.width(); }
    [[nodiscard]] int rows() const noexcept { return height.height(); }

    // True if the point lies at least margin_m inside the grid edge.
    [[nodiscard]] bool contains(double lat, double lon, double margin_m) const;
    // True if the grid can interpolate the point.
    [[nodiscard]] bool covers(double lat, double lon) const;
    [[nodiscard]] std::pair<double, double> pixel(double lat, double lon) const;
    // Bilinear building height. 0 outside the grid.
    [[nodiscard]] float height_at(double lat, double lon) const;
    // Style id + 1 of the nearest cell, 0 where there is no building.
    [[nodiscard]] int style_at(double lat, double lon) const;
    [[nodiscard]] WallSample wall_at(double lat, double lon) const;
    [[nodiscard]] std::uint32_t packed_at(double lat, double lon) const;

    // The frame-relative map the kernels need. Depends on the frame origin,
    // so a renderer computes it per frame.
    [[nodiscard]] GridMap map(const raster::LocalFrame& frame) const;
};

// A fine grid near the camera and a coarse one out to the skyline. The near
// grid wins where it covers a point; the far grid fills in beyond.
struct Buildings {
    std::shared_ptr<const LocalGrid> near;
    std::shared_ptr<const LocalGrid> far;

    [[nodiscard]] float top() const;
    [[nodiscard]] float height_at(double lat, double lon) const;
    [[nodiscard]] int style_at(double lat, double lon) const;
    [[nodiscard]] WallSample wall_at(double lat, double lon) const;
    // The grid that serves a point: near where it covers, else far (may be null).
    [[nodiscard]] const LocalGrid* grid_for(double lat, double lon) const;
};

// Draw the footprints into a square grid of radius_m around lat/lon. detail
// draws roof shapes and building parts; an outline that holds parts is then
// left out. Without detail (the far ring) every outline is a flat prism.
[[nodiscard]] LocalGrid rasterize(std::span<const Footprint> footprints, double lat, double lon,
                                  double radius_m, double resolution_m, bool detail = true);

// Parts replace their outline near the camera; far away only outlines count.
// A part without facade tags gets the windows and weathering of its outline.
[[nodiscard]] std::vector<Footprint> select_footprints(std::span<const Footprint> items,
                                                       bool detail);

// Line a * gx + b * gy = c through two points in drawing coordinates (pixel
// corner origin), in LocalGrid::pixel() units (pixel centre origin).
[[nodiscard]] std::optional<std::array<float, 3>> edge_line(std::pair<double, double> p,
                                                            std::pair<double, double> q);

// Even-odd ray test in lat/lon.
[[nodiscard]] bool inside_ring(const Ring& ring, double lat, double lon);

}  // namespace docksim::osm
