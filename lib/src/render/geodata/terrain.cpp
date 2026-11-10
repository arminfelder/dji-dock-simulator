#include "render/geodata/terrain.hpp"

#include <algorithm>
#include <cmath>

namespace docksim::render::geodata {

namespace {

constexpr double tile = raster::tile_px;

double smoothstep(double edge0, double edge1, double x) {
    const double t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

Bounds around(double lat, double lon, double metres, int zoom) {
    const auto [cx, cy] = raster::global_pixel(lat, lon, zoom);
    const double half = metres / raster::tile_metres(lat, zoom) * tile + 1;
    return {.x0 = cx - half, .y0 = cy - half, .x1 = cx + half, .y1 = cy + half};
}

}  // namespace

raster::Grid2D<float> decode_terrarium(const Mosaic& m) {
    raster::Grid2D<float> out(m.width(), m.height());
    const auto data = out.data();
    for (std::size_t i = 0; i < data.size(); ++i) {
        const auto* p = m.rgb.data() + i * 3;
        data[i] = static_cast<float>(p[0]) * 256.0F + static_cast<float>(p[1]) +
                  static_cast<float>(p[2]) / 256.0F - 32768.0F;
    }
    return out;
}

std::optional<float> elevation(const Mosaic& dem, double lat, double lon) {
    const auto [px, py] = raster::global_pixel(lat, lon, dem.zoom);
    return dem.sample(px, py);
}

float ReliefMosaics::top() const noexcept {
    float t = far ? far->top : 0.0F;
    if (near) t = std::max(t, near->top);
    return t;
}

float ReliefMosaics::height(double lat, double lon, double east, double north,
                            float fallback) const {
    const auto far_height = [&] {
        return far ? elevation(*far, lat, lon).value_or(fallback) : fallback;
    };
    if (!near) return far_height();
    double weight =
        smoothstep(near_blend_start * near_radius, near_radius, std::hypot(east, north));
    float near_value = 0.0F;
    if (weight < 1.0) {
        if (const auto v = elevation(*near, lat, lon)) {
            near_value = *v;
        } else {
            weight = 1.0;
        }
    }
    if (weight <= 0.0) return near_value;
    const float f = far_height();
    if (weight >= 1.0) return f;
    return static_cast<float>(near_value * (1 - weight) + f * weight);
}

Terrain::Terrain(std::string url, int max_zoom, std::string_view cache_dir, int near_zoom,
                 double near_radius, Fetcher fetch)
    : max_zoom_(max_zoom),
      near_zoom_(near_zoom),
      near_radius_(near_radius),
      mosaics_(std::make_shared<TileSource>(std::move(url), cache_dir, std::move(fetch)),
               decode_terrarium) {}

std::shared_ptr<const Mosaic> Terrain::mosaic(double lat, double lon, double reach, bool wait) {
    int zoom = max_zoom_;
    while (zoom > min_dem_zoom && 2 * reach / raster::tile_metres(lat, zoom) > max_tiles_across)
        --zoom;
    return mosaics_.get(zoom, around(lat, lon, reach, zoom), wait);
}

ReliefMosaics Terrain::relief(double lat, double lon, double reach, bool wait) {
    auto far = mosaic(lat, lon, reach, wait);
    if (near_zoom_ <= far->zoom || near_radius_ <= 0)
        return {.far = std::move(far), .near = nullptr, .near_radius = 0.0};
    const double span = std::min(reach, near_radius_);
    auto near = mosaics_.get(near_zoom_, around(lat, lon, span, near_zoom_), wait);
    return {.far = std::move(far), .near = std::move(near), .near_radius = span};
}

float Terrain::point(double lat, double lon) {
    const std::pair key{std::llround(lat * 1e6), std::llround(lon * 1e6)};
    {
        const std::scoped_lock lock(mutex_);
        if (const auto it = points_.find(key); it != points_.end()) return it->second;
    }
    const int zoom = std::max(max_zoom_, near_zoom_);
    const auto [cx, cy] = raster::global_pixel(lat, lon, zoom);
    const auto dem = mosaics_.get(zoom, {.x0 = cx - 1, .y0 = cy - 1, .x1 = cx + 1, .y1 = cy + 1});
    const float value = elevation(*dem, lat, lon).value_or(0.0F);
    const std::scoped_lock lock(mutex_);
    points_[key] = value;
    return value;
}

DemView to_dem(const std::shared_ptr<const Mosaic>& mosaic, const raster::LocalFrame& frame) {
    if (!mosaic) return {};
    return {.mosaic = mosaic,
            .map = frame.pixel_map(mosaic->zoom, static_cast<double>(mosaic->x0),
                                   static_cast<double>(mosaic->y0))};
}

Relief to_relief(const ReliefMosaics& mosaics, const raster::LocalFrame& frame, float fallback) {
    return {
        .far = to_dem(mosaics.far, frame),
        .near = to_dem(mosaics.near, frame),
        .fallback = fallback,
        .near_radius = static_cast<float>(mosaics.near_radius),
        .blend_start = near_blend_start,
    };
}

}  // namespace docksim::render::geodata
