#include "raster/mercator.hpp"

#include "device/geo.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace docksim::raster {

std::pair<double, double> global_pixel(double lat, double lon, int zoom) noexcept {
    const double scale = tile_px * std::exp2(zoom);
    const double px = (lon + 180.0) / 360.0 * scale;
    const double lat_r = geo::radians(std::clamp(lat, -max_latitude, max_latitude));
    const double py = (1.0 - std::asinh(std::tan(lat_r)) / std::numbers::pi) / 2.0 * scale;
    return {px, py};
}

double tile_metres(double lat, int zoom) noexcept {
    return equator_m * std::cos(geo::radians(lat)) / std::exp2(zoom);
}

double LocalFrame::m_per_deg_lat() const noexcept {
    return geo::radians(1.0) * geo::earth_radius_m;
}

double LocalFrame::m_per_deg_lon() const noexcept {
    return m_per_deg_lat() * std::cos(geo::radians(lat0));
}

PixelMap LocalFrame::pixel_map(int zoom, double x0, double y0) const noexcept {
    const double scale = tile_px * std::exp2(zoom);
    const auto [px0, py0] = global_pixel(lat0, lon0, zoom);
    const double phi = geo::radians(lat0);
    const double sec = 1.0 / std::cos(phi);
    const double tan = std::tan(phi);
    const double dy_dphi = -scale / (2.0 * std::numbers::pi);
    const double r = geo::earth_radius_m;
    return {
        .cx = static_cast<float>(px0 - x0 - 0.5),
        .cy = static_cast<float>(py0 - y0 - 0.5),
        .kx = static_cast<float>(scale / 360.0 / m_per_deg_lon()),
        .ky1 = static_cast<float>(dy_dphi * sec / r),
        .ky2 = static_cast<float>(dy_dphi * 0.5 * sec * tan / (r * r)),
        .ky3 =
            static_cast<float>(dy_dphi * (sec * tan * tan + sec * sec * sec) / 6.0 / (r * r * r)),
    };
}

std::pair<double, double> LocalFrame::to_latlon(double east, double north) const noexcept {
    return {lat0 + north / m_per_deg_lat(), lon0 + east / m_per_deg_lon()};
}

std::pair<double, double> LocalFrame::to_local(double lat, double lon) const noexcept {
    return {(lon - lon0) * m_per_deg_lon(), (lat - lat0) * m_per_deg_lat()};
}

}  // namespace docksim::raster
