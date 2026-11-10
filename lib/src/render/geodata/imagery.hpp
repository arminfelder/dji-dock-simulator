#pragma once

#include "raster/mercator.hpp"
#include "render/geodata/tiles.hpp"
#include "render/inputs.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

// Orthophoto texture: a few mosaics, sharp near the camera and coarse far away.
namespace docksim::render::geodata {

// Tiles per side of one level. Each level covers a larger area at a lower zoom.
inline constexpr int level_span = 8;
// Zooms between levels. A pixel blends the two levels around its wanted zoom.
inline constexpr int level_step = 2;
inline constexpr int level_count = 4;
// Pick the zoom whose pixel is a bit finer than the ground sample distance.
inline constexpr double gsd_margin = 0.7;
// Ground outside the imagery coverage.
inline constexpr std::array<float, 3> no_data_rgb{96.0F, 104.0F, 84.0F};

// Latitude/longitude extent of what a frame shows.
struct LatLonBox {
    double south;
    double west;
    double north;
    double east;
};

class Imagery {
public:
    Imagery(std::string url, int max_zoom, std::string_view cache_dir, Fetcher fetch = {});

    // Mosaics that cover the extent, sharpest first. gsd is the ground sample
    // distance near the camera, metres per pixel.
    [[nodiscard]] std::vector<std::shared_ptr<const Mosaic>> levels(const LatLonBox& extent,
                                                                    double lat0, double lon0,
                                                                    double gsd, bool wait = true);

    [[nodiscard]] int max_zoom() const noexcept { return max_zoom_; }
    [[nodiscard]] MosaicCache& mosaics() noexcept { return mosaics_; }

private:
    int max_zoom_;
    MosaicCache mosaics_;
};

// Ground size of one tile pixel at this latitude and (fractional) zoom.
[[nodiscard]] double texel_metres(double lat, double zoom) noexcept;

// The kernel view of the levels in a camera frame.
[[nodiscard]] Levels to_levels(const std::vector<std::shared_ptr<const Mosaic>>& mosaics,
                               const raster::LocalFrame& frame);

// RGB of one point; the rules of texture_at in the render kernels. zoom is
// log2(texel at zoom 0 / footprint). For tests and CPU fallbacks.
[[nodiscard]] std::array<float, 3> sample_levels(const Levels& lv, float east, float north,
                                                 float zoom);

}  // namespace docksim::render::geodata
