#pragma once

#include "osm/features.hpp"
#include "osm/footprints.hpp"
#include "osm/roofs.hpp"
#include "raster/als.hpp"
#include "raster/mercator.hpp"
#include "render/geodata/tiles.hpp"
#include "render/rt/mesh.hpp"

#include <array>
#include <functional>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace docksim::render::rt {

// Walls reach this far below the lowest ground at the footprint, metres, so
// that a coarser terrain tile far away does not show a gap under the building.
inline constexpr double bury_m = 5.0;
// Plausible storey heights from building:levels, metres. Outside this range the
// tags disagree with the height, and the window layout floor stays.
inline constexpr double min_storey_m = 2.2;
inline constexpr double max_storey_m = 6.0;
// Balconies, metres: slab thickness, parapet height, margin beside the window.
// Share of walls with balconies and their depth, per facade Windows value
// (none, punched, ribbon, curtain, industrial, historic, classic). Courtyard
// walls have them more often. Classic blocks have few, small iron ones: one
// column in the middle of the wall, up to classic_balcony_floors.
inline constexpr double balcony_slab_m = 0.18;
inline constexpr double balcony_parapet_m = 1.0;
inline constexpr double balcony_margin_m = 0.35;
inline constexpr std::array<double, 7> balcony_share{0.0, 0.25, 0.2, 0.0, 0.0, 0.0, 0.12};
inline constexpr std::array<double, 7> balcony_depth_m{0.0, 1.3, 1.5, 0.0, 0.0, 0.0, 0.7};
inline constexpr double courtyard_balcony_factor = 1.5;
// Walls shorter than this, or with fewer floors, get no balconies.
inline constexpr double balcony_wall_m = 6.0;
inline constexpr int balcony_floors = 3;
inline constexpr int classic_balcony_floors = 3;
// Terrain tile skirts hang this many metres per tile metre below the tile edge;
// they hide the cracks between tiles of different zoom.
inline constexpr double skirt_share = 0.02;
// Laser-scan roofs: below measured_min_m median height the scan shows no
// building; a rise below measured_flat_m counts as flat. Within
// measured_edge_band_m of the outline the highest nearby scan value counts. A
// standard roof shape counts when its RMS error to the scan is at most
// fit_rms_m, tested on about fit_samples points.
inline constexpr double measured_min_m = 2.0;
inline constexpr double measured_flat_m = 0.8;
inline constexpr double measured_edge_band_m = 1.5;
inline constexpr double fit_rms_m = 1.5;
inline constexpr double fit_samples = 400.0;
// Trees from the laser scan: canopy above tree_min_m on a tree_step_m grid;
// the crown sides end at tree_crown_share of the tree height.
inline constexpr double tree_min_m = 3.0;
inline constexpr double tree_step_m = 2.0;
inline constexpr double tree_crown_share = 0.4;
// A scan point is flat when it differs less than tree_flat_m from its
// neighbours' mean. A 3 x 3 block of flat points, grown tree_structure_grow
// fine steps, is a structure (a roof that OSM does not have).
inline constexpr double tree_flat_m = 0.15;
inline constexpr int tree_structure_grow = 3;
// The crown base sits this far out from the canopy edge.
inline constexpr double tree_side_slope_m = 1.0;
// Curved roofs: triangle subdivision levels, and wall segments under the roof line.
inline constexpr int dome_subdivisions = 3;
inline constexpr int dome_segments = 8;

// Absolute ground height in metres at lat, lon.
using GroundFn = std::function<double(double lat, double lon)>;

using Point2 = osm::Point;
using Polygon = std::vector<Point2>;
using Plane = std::array<double, 3>;

// Roof height over a footprint: base + eaves + rise * clamp(min of planes, 0, 1).
// Flat roofs have no planes. Domes have none either (empty optional) but a
// curved rise; their mesh is subdivided.
struct RoofSurface {
    double base = 0;
    double eaves = 0;
    double rise = 0;
    std::optional<std::vector<Plane>> planes = std::vector<Plane>{};
    std::optional<osm::Roof> roof;
    std::optional<osm::Box> box;

    // The roof of one outline from the OSM tags. ring in local metres.
    [[nodiscard]] static RoofSurface of(const osm::Footprint& item, std::span<const Point2> ring,
                                        double base);

    [[nodiscard]] bool curved() const noexcept { return !planes.has_value(); }
    [[nodiscard]] bool flat() const noexcept { return planes && planes->empty(); }
    [[nodiscard]] double z(double x, double y) const;
    // Edge parameters 0 .. 1 where the roof line along p-q may bend.
    [[nodiscard]] std::vector<double> breaks(Point2 p, Point2 q) const;
};

// A roof from the laser scan; empty where the scan shows no building or no
// standard shape fits within fit_rms_m. outer counter-clockwise, local metres.
[[nodiscard]] std::optional<RoofSurface> measured_roof(std::span<const Point2> outer,
                                                       std::span<const Polygon> holes, double base,
                                                       const raster::als::HeightGrid& grid,
                                                       const raster::LocalFrame& origin);

// One DEM tile as a grid of quads x quads cells, with skirts on all four sides.
[[nodiscard]] Mesh terrain_tile(const geodata::Mosaic& dem, int zoom, int tx, int ty, int quads,
                                const raster::LocalFrame& origin, double z0);

// Walls and roofs of the footprints. detail keeps building parts (near ring).
// With heights (laser scan), buildings whose height and roof are only guesses
// take the scan's height and best fitting standard roof.
[[nodiscard]] Mesh buildings(std::span<const osm::Footprint> footprints,
                             const raster::LocalFrame& origin, double z0, const GroundFn& ground,
                             bool detail, const raster::als::HeightGrid* heights = nullptr,
                             std::span<const osm::Feature> features = {}, bool balconies = false);

// Height above z0 of the highest roof over lat/lon; 0 where no building stands.
[[nodiscard]] double roof_height_at(std::span<const osm::Footprint> footprints,
                                    const raster::LocalFrame& origin, double z0,
                                    const GroundFn& ground, double lat, double lon,
                                    const raster::als::HeightGrid* heights = nullptr);

// Latitude bounds south, west, north, east in degrees.
struct LatLonBounds {
    double south;
    double west;
    double north;
    double east;
};

// Tree canopies from the laser scan: where it stands tree_min_m above the
// ground outside every building, a tree_step_m grid at the canopy height, with
// sides down to the crown base. Shaded like terrain.
[[nodiscard]] Mesh trees(std::span<const osm::Footprint> footprints,
                         const raster::LocalFrame& origin, double z0, const GroundFn& ground,
                         const raster::als::HeightGrid& heights, const LatLonBounds& bounds);

// Unit face normals.
[[nodiscard]] std::vector<std::array<float, 3>> face_normals(const Mesh& mesh);

// x, y of a unit normal as two float16 in one float32 (x in the low half).
[[nodiscard]] float pack_normal(double x, double y);
[[nodiscard]] std::pair<float, float> unpack_normal(float slot);

}  // namespace docksim::render::rt
