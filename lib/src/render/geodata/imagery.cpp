#include "render/geodata/imagery.hpp"

#include <algorithm>
#include <cmath>

namespace docksim::render::geodata {

namespace {

constexpr double tile = raster::tile_px;

std::optional<std::array<float, 3>> sample_level(const LevelView& view, float east, float north) {
    if (!view.mosaic || view.mosaic->height() == 0) return std::nullopt;
    const Mosaic& l = *view.mosaic;
    auto [lx, ly] = view.map.apply(east, north);
    if (lx < 0.0F || ly < 0.0F || lx > static_cast<float>(l.width() - 1) ||
        ly > static_cast<float>(l.height() - 1)) {
        return std::nullopt;
    }
    lx = std::min(lx, static_cast<float>(l.width()) - raster::edge);
    ly = std::min(ly, static_cast<float>(l.height()) - raster::edge);
    const int ix = static_cast<int>(lx);
    const int iy = static_cast<int>(ly);
    if (l.valid(ix, iy) == 0 || l.valid(ix + 1, iy + 1) == 0) return std::nullopt;
    const float fx = lx - static_cast<float>(ix);
    const float fy = ly - static_cast<float>(iy);
    const auto w = static_cast<std::size_t>(l.width());
    const std::size_t a = 3 * (static_cast<std::size_t>(iy) * w + static_cast<std::size_t>(ix));
    const std::size_t c = a + 3 * w;
    std::array<float, 3> out{};
    for (std::size_t k = 0; k < 3; ++k) {
        const auto v = [&](std::size_t i) { return static_cast<float>(l.rgb[i + k]); };
        out[k] =
            (v(a) * (1 - fx) + v(a + 3) * fx) * (1 - fy) + (v(c) * (1 - fx) + v(c + 3) * fx) * fy;
    }
    return out;
}

}  // namespace

Imagery::Imagery(std::string url, int max_zoom, std::string_view cache_dir, Fetcher fetch)
    : max_zoom_(max_zoom),
      mosaics_(std::make_shared<TileSource>(std::move(url), cache_dir, std::move(fetch))) {}

std::vector<std::shared_ptr<const Mosaic>> Imagery::levels(const LatLonBox& extent, double lat0,
                                                           double lon0, double gsd, bool wait) {
    int zoom = max_zoom_;
    while (zoom > 1 && raster::tile_metres(lat0, zoom) / tile < gsd * gsd_margin) --zoom;
    std::vector<std::shared_ptr<const Mosaic>> out;
    for (int level = 0; level < level_count; ++level) {
        int z = std::max(1, zoom - level * level_step);
        const auto [ax, ay] = raster::global_pixel(extent.south, extent.west, z);
        const auto [bx, by] = raster::global_pixel(extent.north, extent.east, z);
        const double px_lo = std::min(ax, bx);
        const double px_hi = std::max(ax, bx);
        const double py_lo = std::min(ay, by);
        const double py_hi = std::max(ay, by);
        double x_lo = px_lo;
        double x_hi = px_hi;
        double y_lo = py_lo;
        double y_hi = py_hi;
        if (level < level_count - 1) {
            const auto [cx, cy] = raster::global_pixel(lat0, lon0, z);
            const double half = level_span * tile / 2;
            x_lo = std::max(x_lo, cx - half);
            x_hi = std::min(x_hi, cx + half);
            y_lo = std::max(y_lo, cy - half);
            y_hi = std::min(y_hi, cy + half);
        } else {
            while (z > 1 &&
                   (x_hi - x_lo) * (y_hi - y_lo) / (tile * tile) > level_span * level_span) {
                --z;
                x_lo /= 2;
                x_hi /= 2;
                y_lo /= 2;
                y_hi /= 2;
            }
        }
        if (x_lo > x_hi || y_lo > y_hi) continue;
        out.push_back(mosaics_.get(z, {.x0 = x_lo, .y0 = y_lo, .x1 = x_hi, .y1 = y_hi}, wait));
        const bool whole = x_lo == px_lo && x_hi == px_hi && y_lo == py_lo && y_hi == py_hi;
        if (level < level_count - 1 && whole) break;
    }
    return out;
}

double texel_metres(double lat, double zoom) noexcept {
    return raster::tile_metres(lat, 0) / tile / std::exp2(zoom);
}

Levels to_levels(const std::vector<std::shared_ptr<const Mosaic>>& mosaics,
                 const raster::LocalFrame& frame) {
    Levels out;
    for (const auto& m : mosaics) {
        if (!m || out.size() == max_levels) continue;
        out.push_back(
            {m, frame.pixel_map(m->zoom, static_cast<double>(m->x0), static_cast<double>(m->y0)),
             static_cast<float>(m->zoom)});
    }
    return out;
}

std::array<float, 3> sample_levels(const Levels& lv, float east, float north, float zoom) {
    const auto n = lv.size();
    if (n == 0) return no_data_rgb;
    std::size_t k = 0;
    while (k + 1 < n && lv[k + 1].zoom >= zoom) ++k;
    const auto a = sample_level(lv[k], east, north);
    std::optional<std::array<float, 3>> b;
    float t = 0.0F;
    if (k + 1 < n && zoom < lv[k].zoom) {
        b = sample_level(lv[k + 1], east, north);
        t = std::clamp((lv[k].zoom - zoom) / (lv[k].zoom - lv[k + 1].zoom), 0.0F, 1.0F);
    }
    if (a && b) {
        std::array<float, 3> out{};
        for (std::size_t c = 0; c < 3; ++c) out[c] = (*a)[c] * (1 - t) + (*b)[c] * t;
        return out;
    }
    if (a) return *a;
    if (b) return *b;
    for (std::size_t j = k + 2; j < n; ++j) {
        if (const auto c = sample_level(lv[j], east, north)) return *c;
    }
    return no_data_rgb;
}

}  // namespace docksim::render::geodata
