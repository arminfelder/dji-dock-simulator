#include "osm/grid.hpp"

#include "log/log.hpp"
#include "osm/facades.hpp"
#include "osm/roofs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <ranges>

namespace docksim::osm {

namespace {

using Pt = std::pair<double, double>;

// Even-odd scanline fill at pixel centres. Points are in drawing coordinates:
// pixel (i, j) covers [i, i + 1) x [j, j + 1).
template <class Visit>
void fill_polygon(int width, int height, std::span<const Pt> pts, Visit&& visit) {
    if (pts.size() < 3) return;
    double ymin = std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();
    for (const auto& p : pts) {
        ymin = std::min(ymin, p.second);
        ymax = std::max(ymax, p.second);
    }
    const int j0 = std::max(0, static_cast<int>(std::floor(ymin - 0.5)));
    const int j1 = std::min(height - 1, static_cast<int>(std::ceil(ymax - 0.5)));
    std::vector<double> xs;
    for (int j = j0; j <= j1; ++j) {
        const double y = j + 0.5;
        xs.clear();
        for (std::size_t k = 0; k < pts.size(); ++k) {
            const auto& a = pts[k];
            const auto& b = pts[(k + 1) % pts.size()];
            if ((a.second <= y && y < b.second) || (b.second <= y && y < a.second)) {
                xs.push_back(a.first +
                             (y - a.second) * (b.first - a.first) / (b.second - a.second));
            }
        }
        std::ranges::sort(xs);
        for (std::size_t k = 0; k + 1 < xs.size(); k += 2) {
            const int i0 = std::max(0, static_cast<int>(std::ceil(xs[k] - 0.5)));
            const int i1 = std::min(width - 1, static_cast<int>(std::ceil(xs[k + 1] - 0.5)) - 1);
            for (int i = i0; i <= i1; ++i) visit(i, j);
        }
    }
}

// A segment drawn band pixels wide, as a filled rectangle.
template <class Visit>
void thick_line(int width, int height, Pt p, Pt q, double band, Visit&& visit) {
    const double dx = q.first - p.first;
    const double dy = q.second - p.second;
    const double length = std::hypot(dx, dy);
    if (length == 0) return;
    const double nx = -dy / length * band / 2;
    const double ny = dx / length * band / 2;
    const std::array<Pt, 4> quad{{{p.first + nx, p.second + ny},
                                  {q.first + nx, q.second + ny},
                                  {q.first - nx, q.second - ny},
                                  {p.first - nx, p.second - ny}}};
    fill_polygon(width, height, quad, visit);
}

int inherit_style(const Footprint& part, const Footprint& outline) {
    auto& table = styles();
    const auto mine = table.facade(part.style);
    const auto theirs = table.facade(outline.style);
    const auto& look = part.own_look ? mine : theirs;
    return table.id({.colour = look.colour,
                     .windows = theirs.windows,
                     .weathering = std::max(mine.weathering, theirs.weathering),
                     .material = look.material});
}

}  // namespace

bool LocalGrid::contains(double lat, double lon, double margin_m) const {
    const double margin = margin_m / resolution_m;
    return std::abs(lat - lat_c) / step_lat + margin < rows() / 2.0 &&
           std::abs(lon - lon_c) / step_lon + margin < width() / 2.0;
}

std::pair<double, double> LocalGrid::pixel(double lat, double lon) const {
    return {(lon - lon_c) / step_lon + width() / 2.0 - 0.5,
            (lat_c - lat) / step_lat + rows() / 2.0 - 0.5};
}

bool LocalGrid::covers(double lat, double lon) const {
    const auto [gx, gy] = pixel(lat, lon);
    return gx >= 0 && gy >= 0 && gx <= width() - 1 && gy <= rows() - 1;
}

float LocalGrid::height_at(double lat, double lon) const {
    const auto [gx, gy] = pixel(lat, lon);
    return raster::bilinear(height, static_cast<float>(gx), static_cast<float>(gy)).value_or(0.0F);
}

std::uint32_t LocalGrid::packed_at(double lat, double lon) const {
    if (style.empty()) return 0;
    const auto [gx, gy] = pixel(lat, lon);
    const auto ix = static_cast<long>(std::nearbyint(gx));
    const auto iy = static_cast<long>(std::nearbyint(gy));
    if (ix < 0 || iy < 0 || ix >= style.width() || iy >= style.height()) return 0;
    return style(static_cast<int>(ix), static_cast<int>(iy));
}

int LocalGrid::style_at(double lat, double lon) const {
    return static_cast<int>(packed_at(lat, lon) & 0xFFU);
}

WallSample LocalGrid::wall_at(double lat, double lon) const {
    const auto row = static_cast<long>(packed_at(lat, lon) >> 8U) - 1;
    if (row < 0 || edges.empty()) return {};
    const auto& [a, b, c] =
        edges[static_cast<std::size_t>(std::min<long>(row, static_cast<long>(edges.size()) - 1))];
    const auto [gx, gy] = pixel(lat, lon);
    return {.distance = static_cast<float>((a * gx + b * gy - c) * resolution_m),
            .east = a,
            .north = -b};
}

GridMap LocalGrid::map(const raster::LocalFrame& frame) const {
    return {
        .gx0 = static_cast<float>((frame.lon0 - lon_c) / step_lon + width() / 2.0 - 0.5),
        .gy0 = static_cast<float>((lat_c - frame.lat0) / step_lat + rows() / 2.0 - 0.5),
        .bx = static_cast<float>(1.0 / (step_lon * frame.m_per_deg_lon())),
        .by = static_cast<float>(1.0 / (step_lat * frame.m_per_deg_lat())),
    };
}

float Buildings::top() const {
    float t = 0;
    if (near) t = std::max(t, near->top);
    if (far) t = std::max(t, far->top);
    return t;
}

const LocalGrid* Buildings::grid_for(double lat, double lon) const {
    if (near && (!far || near->covers(lat, lon))) return near.get();
    return far.get();
}

float Buildings::height_at(double lat, double lon) const {
    const auto* g = grid_for(lat, lon);
    return g != nullptr ? g->height_at(lat, lon) : 0.0F;
}

int Buildings::style_at(double lat, double lon) const {
    const auto* g = grid_for(lat, lon);
    return g != nullptr ? g->style_at(lat, lon) : 0;
}

WallSample Buildings::wall_at(double lat, double lon) const {
    const auto* g = grid_for(lat, lon);
    return g != nullptr ? g->wall_at(lat, lon) : WallSample{};
}

std::optional<std::array<float, 3>> edge_line(std::pair<double, double> p,
                                              std::pair<double, double> q) {
    const double dx = q.first - p.first;
    const double dy = q.second - p.second;
    const double length = std::hypot(dx, dy);
    if (length < 1e-6) return std::nullopt;
    const double a = -dy / length;
    const double b = dx / length;
    // Drawing coordinates count from the pixel corner, pixel() from the pixel centre.
    return std::array{static_cast<float>(a), static_cast<float>(b),
                      static_cast<float>(a * (p.first - 0.5) + b * (p.second - 0.5))};
}

bool inside_ring(const Ring& ring, double lat, double lon) {
    bool inside = false;
    for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        const auto& pi = ring[i];
        const auto& pj = ring[j];
        if ((pi.lat > lat) != (pj.lat > lat) &&
            lon <= (pj.lon - pi.lon) * (lat - pi.lat) / (pj.lat - pi.lat) + pi.lon) {
            inside = !inside;
        }
    }
    return inside;
}

std::vector<Footprint> select_footprints(std::span<const Footprint> items, bool detail) {
    std::vector<std::size_t> parts;
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (items[i].part && !items[i].outer.empty() && !items[i].outer[0].empty())
            parts.push_back(i);
    }
    std::vector<Footprint> out;
    if (!detail || parts.empty()) {
        for (const auto& f : items) {
            if (!f.part) out.push_back(f);
        }
        return out;
    }
    std::vector<bool> holder(items.size(), false);
    std::map<std::size_t, int> styled;
    for (std::size_t o = 0; o < items.size(); ++o) {
        const auto& outline = items[o];
        if (outline.part || outline.outer.empty() || outline.outer[0].size() < 3) continue;
        const auto& ring = outline.outer[0];
        const auto [lat_lo, lat_hi] =
            std::ranges::minmax(ring | std::views::transform(&LatLon::lat));
        const auto [lon_lo, lon_hi] =
            std::ranges::minmax(ring | std::views::transform(&LatLon::lon));
        for (const auto p : parts) {
            const auto& first = items[p].outer[0][0];
            const bool in_box = lat_lo <= first.lat && first.lat <= lat_hi && lon_lo <= first.lon &&
                                first.lon <= lon_hi;
            if (in_box && inside_ring(ring, first.lat, first.lon)) {
                holder[o] = true;
                if (items[p].plain_part && !styled.contains(p))
                    styled[p] = inherit_style(items[p], outline);
            }
        }
    }
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (holder[i]) continue;
        out.push_back(items[i]);
        if (const auto it = styled.find(i); it != styled.end()) out.back().style = it->second;
    }
    return out;
}

namespace {

// Draws footprints into a LocalGrid. Pixel (0, 0) is the north-west corner; x grows east.
class Rasterizer {
public:
    Rasterizer(LocalGrid& grid, int size, double lat, double lon)
        : grid_(grid), size_(size), half_(size / 2.0), lat_(lat), lon_(lon) {}

    // Heights (flat or the roof shape) and the style of one footprint; holes cut back to 0.
    void footprint(const Footprint& item, bool detail) {
        for (const auto& ring : item.outer) {
            if (ring.size() < 3) continue;
            const auto pts = xy(ring);
            if (detail && item.roof) {
                roof(item, pts);
            } else {
                fill(pts, static_cast<float>(item.height), std::nullopt);
            }
            fill_polygon(size_, size_, pts, [&](int i, int j) {
                grid_.style(i, j) = static_cast<std::uint32_t>(item.style + 1);
            });
        }
        for (const auto& ring : item.inner) {
            if (ring.size() >= 3) fill(xy(ring), 0.0F, 0U);
        }
    }

    // One edge row per outline segment, and its index in walls along a band of pixels.
    void edges(const Footprint& item, double band, raster::Grid2D<std::uint32_t>& walls) {
        for (const auto* rings : {&item.outer, &item.inner}) {
            for (const auto& ring : *rings) {
                if (ring.size() < 3) continue;
                const auto pts = xy(ring);
                for (std::size_t k = 0; k < pts.size(); ++k) {
                    const auto& p = pts[k];
                    const auto& q = pts[(k + 1) % pts.size()];
                    const auto line = edge_line(p, q);
                    if (!line || grid_.edges.size() >= max_edges) continue;
                    grid_.edges.push_back(*line);
                    const auto row = static_cast<std::uint32_t>(grid_.edges.size());
                    thick_line(size_, size_, p, q, band, [&](int i, int j) { walls(i, j) = row; });
                }
            }
        }
    }

private:
    [[nodiscard]] std::vector<Pt> xy(const Ring& ring) const {
        std::vector<Pt> out;
        out.reserve(ring.size());
        for (const auto& p : ring)
            out.emplace_back((p.lon - lon_) / grid_.step_lon + half_,
                             (lat_ - p.lat) / grid_.step_lat + half_);
        return out;
    }

    void fill(std::span<const Pt> pts, float h, std::optional<std::uint32_t> style) {
        fill_polygon(size_, size_, pts, [&](int i, int j) {
            grid_.height(i, j) = h;
            if (style) grid_.style(i, j) = *style;
        });
    }

    void roof(const Footprint& item, std::span<const Pt> ring) {
        const double res = grid_.resolution_m;
        std::vector<Point> scaled;
        for (const auto& [x, y] : ring) scaled.emplace_back(x * res, y * res);
        const auto box = min_area_box(scaled);
        const auto top = item.wall ? std::nullopt : std::optional(item.height);
        fill_polygon(size_, size_, ring, [&](int i, int j) {
            grid_.height(i, j) = static_cast<float>(roof_height(
                item.roof.value(), box, (i + 0.5) * res, (j + 0.5) * res, top, item.wall));
        });
    }

    LocalGrid& grid_;
    int size_;
    double half_;
    double lat_;
    double lon_;
};

}  // namespace

LocalGrid rasterize(std::span<const Footprint> footprints, double lat, double lon, double radius_m,
                    double resolution_m, bool detail) {
    const raster::LocalFrame frame{.lat0 = lat, .lon0 = lon};
    const int size = static_cast<int>(std::nearbyint(2 * radius_m / resolution_m));
    LocalGrid grid{
        .lat_c = lat,
        .lon_c = lon,
        .step_lat = resolution_m / frame.m_per_deg_lat(),
        .step_lon = resolution_m / frame.m_per_deg_lon(),
        .resolution_m = resolution_m,
        .height = raster::Grid2D<float>(size, size, 0.0F),
        .style = raster::Grid2D<std::uint32_t>(size, size, 0U),
    };
    raster::Grid2D<std::uint32_t> walls(size, size, 0U);
    const double band = std::max(1.0, std::nearbyint(2 * edge_band_m / resolution_m));
    Rasterizer draw(grid, size, lat, lon);

    auto items = select_footprints(footprints, detail);
    // Taller parts last, so that they win where footprints overlap.
    std::ranges::stable_sort(items, {}, &Footprint::height);
    for (const auto& item : items) draw.footprint(item, detail);
    for (const auto& item : items) draw.edges(item, band, walls);

    auto style = grid.style.data();
    const auto wall = walls.data();
    for (std::size_t i = 0; i < style.size(); ++i) style[i] = (style[i] & 0xFFU) | (wall[i] << 8U);
    const auto heights = grid.height.data();
    grid.top = heights.empty() ? 0.0F : *std::ranges::max_element(heights);
    log::get("dock_sim.osm")
        ->debug("building grid at {:.5f} {:.5f}: {} footprints, {}x{} at {:.1f} m{}", lat, lon,
                items.size(), size, size, resolution_m, detail ? " with roofs" : "");
    return grid;
}

}  // namespace docksim::osm
