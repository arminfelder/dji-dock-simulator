#pragma once

#include "raster/mercator.hpp"
#include "render/geodata/tiles.hpp"
#include "render/inputs.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

// Terrain heights from Terrarium DEM tiles.
namespace docksim::render::geodata {

// A DEM mosaic spans at most this many tiles per side; farther views use a lower zoom.
inline constexpr double max_tiles_across = 7.0;
inline constexpr int min_dem_zoom = 8;
// The near DEM fades into the far one over the outer share of its radius.
inline constexpr float near_blend_start = 0.8F;

// Terrarium encoding: metres = R * 256 + G + B / 256 - 32768.
[[nodiscard]] raster::Grid2D<float> decode_terrarium(const Mosaic& m);

// Height at lat/lon of a DEM mosaic; empty without data.
[[nodiscard]] std::optional<float> elevation(const Mosaic& dem, double lat, double lon);

// The two DEM mosaics of one frame.
struct ReliefMosaics {
    std::shared_ptr<const Mosaic> far;
    std::shared_ptr<const Mosaic> near;
    double near_radius = 0.0;

    [[nodiscard]] float top() const noexcept;
    // Height at lat/lon; east/north (metres from the camera) set the near/far
    // blend. The same rules as terrain_at in the render kernels.
    [[nodiscard]] float height(double lat, double lon, double east, double north,
                               float fallback) const;
};

class Terrain {
public:
    Terrain(std::string url, int max_zoom, std::string_view cache_dir, int near_zoom = 0,
            double near_radius = 0.0, Fetcher fetch = {});

    // DEM mosaic that covers reach metres around lat/lon.
    [[nodiscard]] std::shared_ptr<const Mosaic> mosaic(double lat, double lon, double reach,
                                                       bool wait = true);
    // Far DEM for the whole view plus, if set, the near DEM around the camera.
    [[nodiscard]] ReliefMosaics relief(double lat, double lon, double reach, bool wait = true);
    // Terrain height at one point, cached. Uses the finest DEM zoom. 0 without data.
    [[nodiscard]] float point(double lat, double lon);

    [[nodiscard]] int max_zoom() const noexcept { return max_zoom_; }
    [[nodiscard]] int near_zoom() const noexcept { return near_zoom_; }
    [[nodiscard]] MosaicCache& mosaics() noexcept { return mosaics_; }

private:
    int max_zoom_;
    int near_zoom_;
    double near_radius_;
    MosaicCache mosaics_;
    std::mutex mutex_;
    std::map<std::pair<std::int64_t, std::int64_t>, float> points_;
};

// The kernel view of a DEM mosaic in a camera frame. The result is cached on
// the mosaic for the last frame origin, so a fixed frame costs no copy.
[[nodiscard]] DemView to_dem(const std::shared_ptr<const Mosaic>& mosaic,
                             const raster::LocalFrame& frame);
[[nodiscard]] Relief to_relief(const ReliefMosaics& mosaics, const raster::LocalFrame& frame,
                               float fallback);

}  // namespace docksim::render::geodata
