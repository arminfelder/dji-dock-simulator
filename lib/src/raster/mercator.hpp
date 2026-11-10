#pragma once

#include <utility>

// Web Mercator tile maths, and a local east-north frame around a camera.
namespace docksim::raster {

inline constexpr int tile_px = 256;
inline constexpr double equator_m = 40075016.686;
// Web Mercator is undefined at the poles.
inline constexpr double max_latitude = 85.05;

// Global pixel position of lat/lon at one zoom level.
[[nodiscard]] std::pair<double, double> global_pixel(double lat, double lon, int zoom) noexcept;

// Ground width of one tile at this latitude and zoom.
[[nodiscard]] double tile_metres(double lat, int zoom) noexcept;

// Mosaic pixel (cell centre at .0) of a point at east/north metres:
// x = cx + east * kx, y = cy + north * (ky1 + north * (ky2 + north * ky3)).
struct PixelMap {
    float cx = 0;
    float cy = 0;
    float kx = 0;
    float ky1 = 0;
    float ky2 = 0;
    float ky3 = 0;

    [[nodiscard]] std::pair<float, float> apply(float east, float north) const noexcept {
        return {cx + east * kx, cy + north * (ky1 + north * (ky2 + north * ky3))};
    }
};

// Flat east-north metres around an origin. Good over a few kilometres.
struct LocalFrame {
    double lat0 = 0;
    double lon0 = 0;

    [[nodiscard]] double m_per_deg_lat() const noexcept;
    [[nodiscard]] double m_per_deg_lon() const noexcept;

    // Local metres to pixels of a mosaic whose pixel (0, 0) is global (x0, y0).
    // Mercator y is a third-order series in north; at 5 km and zoom 19 the
    // error stays below 0.01 px. GPUs without fp64 can then work in float32.
    [[nodiscard]] PixelMap pixel_map(int zoom, double x0, double y0) const noexcept;

    [[nodiscard]] std::pair<double, double> to_latlon(double east, double north) const noexcept;
    [[nodiscard]] std::pair<double, double> to_local(double lat, double lon) const noexcept;
};

}  // namespace docksim::raster
