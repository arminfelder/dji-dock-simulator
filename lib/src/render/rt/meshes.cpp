#include "render/rt/meshes.hpp"

#include "image/opencv.hpp"
#include "osm/facades.hpp"
#include "osm/grid.hpp"
#include "raster/grid.hpp"
#include "render/shading.hpp"

#include <mapbox/earcut.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>

namespace docksim::render::rt {

namespace {

constexpr double nan = std::numeric_limits<double>::quiet_NaN();
constexpr double tile = raster::tile_px;

struct Shape {
    osm::RoofShape shape;
    bool across;
};
constexpr std::array<Shape, 4> fit_shapes{{
    {.shape = osm::RoofShape::gabled, .across = false},
    {.shape = osm::RoofShape::gabled, .across = true},
    {.shape = osm::RoofShape::hipped, .across = false},
    {.shape = osm::RoofShape::pyramidal, .across = false},
}};

using Vec = std::array<double, 3>;

Vec cross(const Vec& a, const Vec& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

Vec normalized(const Vec& v) {
    const double len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (len <= 0) return {0, 0, 0};
    return {v[0] / len, v[1] / len, v[2] / len};
}

Point2 local(const raster::LocalFrame& origin, double lat, double lon) {
    return {(lon - origin.lon0) * origin.m_per_deg_lon(),
            (lat - origin.lat0) * origin.m_per_deg_lat()};
}

// Inverse Web Mercator of a global pixel position.
std::pair<double, double> tile_latlon(double px, double py, int zoom) {
    const double scale = tile * std::exp2(zoom);
    const double lon = px / scale * 360.0 - 180.0;
    const double lat = std::atan(std::sinh(std::numbers::pi * (1.0 - 2.0 * py / scale))) * 180.0 /
                       std::numbers::pi;
    return {lat, lon};
}

Polygon ring_of(const raster::LocalFrame& origin, const osm::Ring& ring) {
    Polygon out;
    for (const auto& p : ring) out.push_back(local(origin, p.lat, p.lon));
    if (out.size() > 1 && ring.front() == ring.back()) out.pop_back();
    return out;
}

double signed_area(std::span<const Point2> ring) {
    double sum = 0;
    for (std::size_t i = 0; i < ring.size(); ++i) {
        const auto& p = ring[i];
        const auto& q = ring[(i + 1) % ring.size()];
        sum += p.first * q.second - q.first * p.second;
    }
    return sum;
}

Polygon ccw(Polygon ring) {
    if (signed_area(ring) <= 0) std::ranges::reverse(ring);
    return ring;
}

bool contains(std::span<const Point2> ring, Point2 point) {
    const auto [x, y] = point;
    bool inside = false;
    for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        const auto [xi, yi] = ring[i];
        const auto [xj, yj] = ring[j];
        if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
}

double edge_distance(Point2 point, std::span<const Polygon> rings) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& ring : rings) {
        for (std::size_t k = 0; k < ring.size(); ++k) {
            const auto& a = ring[k];
            const auto& b = ring[(k + 1) % ring.size()];
            const double abx = b.first - a.first;
            const double aby = b.second - a.second;
            const double len2 = std::max(abx * abx + aby * aby, 1e-12);
            const double t = std::clamp(
                ((point.first - a.first) * abx + (point.second - a.second) * aby) / len2, 0.0, 1.0);
            best = std::min(best, std::hypot(point.first - (a.first + t * abx),
                                             point.second - (a.second + t * aby)));
        }
    }
    return best;
}

// numpy.percentile with linear interpolation over the finite values; NaN if none.
double percentile(std::vector<double> values, double q) {
    std::erase_if(values, [](double v) { return !std::isfinite(v); });
    if (values.empty()) return nan;
    std::ranges::sort(values);
    const double pos = q / 100.0 * static_cast<double>(values.size() - 1);
    const auto lo = static_cast<std::size_t>(std::floor(pos));
    const auto hi = std::min(lo + 1, values.size() - 1);
    return values[lo] + (values[hi] - values[lo]) * (pos - static_cast<double>(lo));
}

// Laser-scan heights above the ground at local points of one outline. Points
// near the outline take the highest scan value close by: there the bilinear
// value mixes in the ground beside the wall.
struct ScanProbe {
    const raster::als::HeightGrid& grid;
    const raster::LocalFrame& origin;
    std::vector<Polygon> rings;

    [[nodiscard]] double above(Point2 p) const {
        const double lat = origin.lat0 + p.second / origin.m_per_deg_lat();
        const double lon = origin.lon0 + p.first / origin.m_per_deg_lon();
        const bool near = edge_distance(p, rings) < measured_edge_band_m;
        return grid.sample(lat, lon, near);
    }
};

Mesh roof_mesh(std::span<const Point2> outer, std::span<const Polygon> holes,
               const RoofSurface& roof, double base, int style);

// Storey height from the tagged level count; 0 where the tags give none or an
// implausible one, for example a tower with an override height.
double storey_of(int levels, double eaves) {
    if (levels <= 0) return 0.0;
    const double storey = eaves / levels;
    return storey >= min_storey_m && storey <= max_storey_m ? storey : 0.0;
}

struct LocalFeature {
    Point2 at;
    osm::Feature::Kind kind;
};

// A ground floor feature on one ring edge: parameter range on the edge and kind.
struct Span {
    std::size_t edge;
    double t0;
    double t1;
    float kind;
};

// Distance from p to the segment a-b; t is the parameter of the nearest point.
double segment_distance(Point2 p, Point2 a, Point2 b, double& t) {
    const double ex = b.first - a.first;
    const double ey = b.second - a.second;
    const double length2 = ex * ex + ey * ey;
    t = length2 > 0 ? std::clamp(((p.first - a.first) * ex + (p.second - a.second) * ey) / length2,
                                 0.0, 1.0)
                    : 0.0;
    return std::hypot(a.first + ex * t - p.first, a.second + ey * t - p.second);
}

// A door where an entrance node lies on the ring; the nearest long wall of a
// shop inside the ring becomes a shopfront. Shops only on outer rings.
std::vector<Span> feature_spans(std::span<const Point2> ring,
                                std::span<const LocalFeature> features, bool shops) {
    std::vector<Span> spans;
    const auto n = ring.size();
    for (const auto& f : features) {
        const bool shop = f.kind == osm::Feature::Kind::shop;
        if (shop && (!shops || !contains(ring, f.at))) continue;
        std::size_t best = n;
        double best_d = std::numeric_limits<double>::infinity();
        double best_t = 0;
        for (std::size_t k = 0; k < n; ++k) {
            const auto a = ring[k];
            const auto b = ring[(k + 1) % n];
            const double length = std::hypot(b.first - a.first, b.second - a.second);
            if (length < (shop ? shading::shop_wall_m : shading::door_width_m)) continue;
            double t = 0;
            const double d = segment_distance(f.at, a, b, t);
            if (d < best_d) {
                best = k;
                best_d = d;
                best_t = t;
            }
        }
        if (best == n) continue;
        if (shop) {
            spans.push_back({best, 0.0, 1.0, shading::feature_shop});
            continue;
        }
        if (best_d > shading::entrance_snap_m) continue;
        const auto a = ring[best];
        const auto b = ring[(best + 1) % n];
        const double width =
            shading::door_width_m / std::hypot(b.first - a.first, b.second - a.second);
        const double t0 = std::clamp(best_t - width / 2.0, 0.0, 1.0 - width);
        spans.push_back({best, t0, t0 + width, shading::feature_door});
    }
    return spans;
}

std::uint64_t mix(std::uint64_t x) {
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27U)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31U);
}

double unit(std::uint64_t x) {
    return static_cast<double>(mix(x) >> 11U) * 0x1.0p-53;
}

// A flat colour quad a-b-c-d with normal n.
void detail_quad(Mesh& mesh, const Vertex& a, const Vertex& b, const Vertex& c, const Vertex& d,
                 const Vertex& n, const Vec3& colour) {
    const auto first = static_cast<std::uint32_t>(mesh.vertices.size());
    mesh.vertices.insert(mesh.vertices.end(), {a, b, c, d});
    mesh.indices.push_back({first, first + 1, first + 2});
    mesh.indices.push_back({first, first + 2, first + 3});
    Record r{};
    r[record_kind] = kind_detail;
    r[record_normal] = n[0];
    r[record_normal + 1] = n[1];
    r[record_normal + 2] = n[2];
    r[record_feature] = colour.x;
    r[record_feature + 1] = colour.y;
    r[record_feature + 2] = colour.z;
    mesh.records.push_back(r);
    mesh.records.push_back(r);
}

// Balconies on some walls of a ring: slab and parapet in front of the window
// of each chosen bay, from the first floor up. The bays match walls(), so the
// balconies sit on the window grid. seed: the building, so a wall keeps its look.
Mesh balconies(std::span<const Point2> ring, double base, double eaves, std::size_t layout,
               double floor_m, double bay, const Vec3& colour, std::uint64_t seed, bool courtyard) {
    Mesh mesh;
    const double depth = balcony_depth_m[layout];
    const int floors = static_cast<int>(eaves / floor_m);
    if (depth <= 0.0 || floors < balcony_floors) return mesh;
    const auto& w = shading::window_layouts[layout];
    const double share =
        std::min(0.9, balcony_share[layout] * (courtyard ? courtyard_balcony_factor : 1.0));
    // Iron railings on classic blocks; plaster parapets elsewhere.
    const Vec3 parapet = layout == static_cast<std::size_t>(shading::windows_classic)
                             ? Vec3{46.0F, 46.0F, 50.0F}
                             : colour;
    const auto n = ring.size();
    for (std::size_t k = 0; k < n; ++k) {
        const auto p = ring[k];
        const auto q = ring[(k + 1) % n];
        const double ex = q.first - p.first;
        const double ey = q.second - p.second;
        const double length = std::hypot(ex, ey);
        const std::uint64_t key = mix(seed ^ (k * 0xD1B54A32D192ED03ULL));
        if (length < balcony_wall_m || unit(key) >= share) continue;
        const double ux = ex / length;
        const double uy = ey / length;
        const double nx = uy;
        const double ny = -ux;
        const bool classic = layout == static_cast<std::size_t>(shading::windows_classic);
        // 0: every bay, 1: every other bay, 2: one column per three bays, 3: the middle bay.
        const int pattern = classic ? 3 : static_cast<int>(unit(key ^ 0x5851F42D4C957F2DULL) * 3.0);
        const double margin = std::fmod(length, bay) / 2.0;
        const int bays = static_cast<int>(length / bay);
        const int top_floor = classic ? std::min(floors, classic_balcony_floors + 1) : floors;
        const double half =
            std::min(bay / 2.0 - 0.1, (w.bay_hi - w.bay_lo) * bay / 2.0 + balcony_margin_m);
        for (int b = 0; b < bays; ++b) {
            if ((pattern == 1 && b % 2 != 0) || (pattern == 2 && b % 3 != 1) ||
                (pattern == 3 && b != bays / 2)) {
                continue;
            }
            const double centre = margin + (b + (w.bay_lo + w.bay_hi) / 2.0) * bay;
            const double s0 = centre - half;
            const double s1 = centre + half;
            const auto at = [&](double s, double out, double z) {
                return Vertex{static_cast<float>(p.first + ux * s + nx * out),
                              static_cast<float>(p.second + uy * s + ny * out),
                              static_cast<float>(z)};
            };
            const Vertex front{static_cast<float>(nx), static_cast<float>(ny), 0.0F};
            const Vertex side0{static_cast<float>(-ux), static_cast<float>(-uy), 0.0F};
            const Vertex side1{static_cast<float>(ux), static_cast<float>(uy), 0.0F};
            for (int f = 1; f < top_floor; ++f) {
                const double z = base + f * floor_m;
                if (z + balcony_parapet_m > base + eaves - 0.5) break;
                const double zb = z - balcony_slab_m;
                const double zt = z + balcony_parapet_m;
                // Slab: top, bottom and front. Its ends are a few pixels at most.
                detail_quad(mesh, at(s0, 0, z), at(s1, 0, z), at(s1, depth, z), at(s0, depth, z),
                            {0, 0, 1}, colour);
                detail_quad(mesh, at(s0, 0, zb), at(s0, depth, zb), at(s1, depth, zb),
                            at(s1, 0, zb), {0, 0, -1}, colour);
                detail_quad(mesh, at(s0, depth, zb), at(s0, depth, z), at(s1, depth, z),
                            at(s1, depth, zb), front, colour);
                // Parapet: front and both sides, one sheet each; the shader lights the side it
                // sees.
                detail_quad(mesh, at(s0, depth, z), at(s1, depth, z), at(s1, depth, zt),
                            at(s0, depth, zt), front, parapet);
                detail_quad(mesh, at(s0, 0, z), at(s0, depth, z), at(s0, depth, zt), at(s0, 0, zt),
                            side0, parapet);
                detail_quad(mesh, at(s1, 0, z), at(s1, depth, z), at(s1, depth, zt), at(s1, 0, zt),
                            side1, parapet);
            }
        }
    }
    return mesh;
}

Mesh walls(std::span<const Point2> ring, double base, const RoofSurface& roof, int style,
           double bay, double low, double storey, std::span<const Span> spans) {
    struct Segment {
        Point2 a;
        Point2 b;
        std::size_t edge;
        // Feature kind and its parameter range on the edge.
        float kind;
        double from;
        double to;
    };
    std::vector<Segment> segments;
    const auto n = ring.size();
    if (n == 0) return {};
    for (std::size_t k = 0; k < n; ++k) {
        const auto p = ring[k];
        const auto q = ring[(k + 1) % n];
        if (std::hypot(q.first - p.first, q.second - p.second) <= 1e-3) continue;
        auto ts = roof.flat() ? std::vector<double>{0.0, 1.0} : roof.breaks(p, q);
        for (const auto& sp : spans) {
            if (sp.edge != k) continue;
            ts.push_back(sp.t0);
            ts.push_back(sp.t1);
        }
        std::ranges::sort(ts);
        for (std::size_t i = 0; i + 1 < ts.size(); ++i) {
            const double t0 = ts[i];
            const double t1 = ts[i + 1];
            if (t1 - t0 <= 1e-6) continue;
            const auto at = [&](double t) {
                return Point2{p.first + (q.first - p.first) * t,
                              p.second + (q.second - p.second) * t};
            };
            // A door wins over the shopfront around it.
            const double mid = (t0 + t1) / 2.0;
            Segment seg{.a = at(t0), .b = at(t1), .edge = k, .kind = 0.0F, .from = 0.0, .to = 0.0};
            for (const auto& sp : spans) {
                if (sp.edge != k || mid < sp.t0 || mid > sp.t1) continue;
                if (seg.kind == shading::feature_door) continue;
                seg.kind = sp.kind;
                seg.from = sp.t0;
                seg.to = sp.t1;
            }
            segments.push_back(seg);
        }
    }
    Mesh mesh;
    if (segments.empty()) return mesh;
    const auto k = static_cast<std::uint32_t>(segments.size());
    const auto f = [](double v) { return static_cast<float>(v); };
    mesh.vertices.resize(4 * static_cast<std::size_t>(k));
    for (std::uint32_t i = 0; i < k; ++i) {
        const auto& s = segments[i];
        mesh.vertices[i] = {f(s.a.first), f(s.a.second), f(low)};
        mesh.vertices[i + k] = {f(s.b.first), f(s.b.second), f(low)};
        mesh.vertices[i + 2 * k] = {f(s.b.first), f(s.b.second), f(roof.z(s.b.first, s.b.second))};
        mesh.vertices[i + 3 * k] = {f(s.a.first), f(s.a.second), f(roof.z(s.a.first, s.a.second))};
    }
    for (std::uint32_t i = 0; i < k; ++i) mesh.indices.push_back({i, i + k, i + 2 * k});
    for (std::uint32_t i = 0; i < k; ++i) mesh.indices.push_back({i, i + 2 * k, i + 3 * k});
    std::vector<Record> records(k);
    for (std::uint32_t i = 0; i < k; ++i) {
        const auto p = ring[segments[i].edge];
        const auto q = ring[(segments[i].edge + 1) % n];
        const double ex = q.first - p.first;
        const double ey = q.second - p.second;
        const double length = std::hypot(ex, ey);
        const double nx = ey / length;
        const double ny = -ex / length;
        // The pattern coordinate along the wall: -ny * east + nx * north + offset.
        // Bays sit centred on each wall, so corners get equal margins.
        const double start = -ny * p.first + nx * p.second;
        const double offset = -start - std::fmod(length, bay) / 2.0;
        Record& r = records[i];
        r.fill(0.0F);
        r[record_kind] = kind_wall;
        r[record_style] = static_cast<float>(style + 1);
        r[record_normal] = f(nx);
        r[record_normal + 1] = f(ny);
        r[record_along] = f(offset);
        r[record_base] = f(base);
        // Windows stay below the eaves; a gable triangle above is plain wall.
        r[record_top] = f(base + roof.eaves);
        r[record_storey] = f(storey);
        if (segments[i].kind != 0.0F) {
            // The along coordinate at edge parameter t is t * length + offset + start.
            r[record_feature] = segments[i].kind;
            r[record_feature_from] = f(segments[i].from * length + offset + start);
            r[record_feature_to] = f(segments[i].to * length + offset + start);
        }
    }
    mesh.records = records;
    mesh.records.insert(mesh.records.end(), records.begin(), records.end());
    return mesh;
}

// Part of a convex polygon where a * x + b * y + c <= 0 (Sutherland-Hodgman).
Polygon clip(const Polygon& poly, double a, double b, double c) {
    Polygon out;
    const auto n = poly.size();
    for (std::size_t k = 0; k < n; ++k) {
        const auto p = poly[k];
        const auto q = poly[(k + 1) % n];
        const double fp = a * p.first + b * p.second + c;
        const double fq = a * q.first + b * q.second + c;
        if (fp <= 0) out.push_back(p);
        if ((fp < 0) != (fq < 0) && fp != fq) {
            const double t = fp / (fp - fq);
            out.emplace_back(p.first + (q.first - p.first) * t,
                             p.second + (q.second - p.second) * t);
        }
    }
    return out;
}

using Tri = std::array<std::size_t, 3>;

// Cut each triangle into pieces where one plane is the lowest; each piece is flat.
void split_by_planes(std::vector<Point2>& points, std::vector<Tri>& tris,
                     const std::vector<Plane>& planes) {
    std::vector<Point2> out_points;
    std::vector<Tri> out_tris;
    for (const auto& t : tris) {
        const Polygon corners{points[t[0]], points[t[1]], points[t[2]]};
        for (std::size_t i = 0; i < planes.size(); ++i) {
            Polygon poly = corners;
            for (std::size_t j = 0; j < planes.size() && poly.size() >= 3; ++j) {
                if (j == i) continue;
                poly = clip(poly, planes[i][0] - planes[j][0], planes[i][1] - planes[j][1],
                            planes[i][2] - planes[j][2]);
            }
            if (poly.size() < 3) continue;
            const auto first = out_points.size();
            out_points.insert(out_points.end(), poly.begin(), poly.end());
            for (std::size_t m = 1; m + 1 < poly.size(); ++m)
                out_tris.push_back({first, first + m, first + m + 1});
        }
    }
    if (out_tris.empty()) return;
    points = std::move(out_points);
    tris = std::move(out_tris);
}

// Split every triangle into 4, levels times; for curved roofs.
void subdivide(std::vector<Point2>& pts, std::vector<Tri>& tris, int levels) {
    for (int level = 0; level < levels; ++level) {
        std::map<std::pair<std::size_t, std::size_t>, std::size_t> mid;
        const auto middle = [&](std::size_t i, std::size_t j) {
            const auto key = std::minmax(i, j);
            const auto it = mid.find(key);
            if (it != mid.end()) return it->second;
            pts.emplace_back((pts[i].first + pts[j].first) / 2,
                             (pts[i].second + pts[j].second) / 2);
            mid.emplace(key, pts.size() - 1);
            return pts.size() - 1;
        };
        std::vector<Tri> next;
        for (const auto& [a, b, c] : tris) {
            const auto ab = middle(a, b);
            const auto bc = middle(b, c);
            const auto ca = middle(c, a);
            next.push_back({a, ab, ca});
            next.push_back({ab, b, bc});
            next.push_back({ca, bc, c});
            next.push_back({ab, bc, ca});
        }
        tris = std::move(next);
    }
}

std::array<float, 3> face_normal(const Vertex& a, const Vertex& b, const Vertex& c) {
    const Vec ab{double{b[0]} - a[0], double{b[1]} - a[1], double{b[2]} - a[2]};
    const Vec ac{double{c[0]} - a[0], double{c[1]} - a[1], double{c[2]} - a[2]};
    const auto n = normalized(cross(ab, ac));
    return {static_cast<float>(n[0]), static_cast<float>(n[1]), static_cast<float>(n[2])};
}

void set_normal(Record& r, const std::array<float, 3>& n) {
    r[record_normal] = n[0];
    r[record_normal + 1] = n[1];
    r[record_normal + 2] = n[2];
}

Mesh roof_mesh(std::span<const Point2> outer, std::span<const Polygon> holes,
               const RoofSurface& roof, double base, int style) {
    std::vector<Polygon> rings{Polygon(outer.begin(), outer.end())};
    rings.insert(rings.end(), holes.begin(), holes.end());
    std::vector<Point2> points;
    for (const auto& r : rings) points.insert(points.end(), r.begin(), r.end());
    const auto flat_index = mapbox::earcut<std::uint32_t>(rings);
    std::vector<Tri> tris;
    for (std::size_t i = 0; i + 2 < flat_index.size(); i += 3) {
        Tri t{flat_index[i], flat_index[i + 1], flat_index[i + 2]};
        // The outer ring is counter-clockwise; keep every triangle so, normals up.
        const Point2 a = points[t[0]];
        const Point2 b = points[t[1]];
        const Point2 c = points[t[2]];
        if ((b.first - a.first) * (c.second - a.second) -
                (b.second - a.second) * (c.first - a.first) <
            0) {
            std::swap(t[1], t[2]);
        }
        tris.push_back(t);
    }
    Mesh mesh;
    if (tris.empty()) return mesh;
    if (roof.curved()) {
        subdivide(points, tris, dome_subdivisions);
    } else if (!roof.planes.value().empty()) {
        split_by_planes(points, tris, roof.planes.value());
    }
    for (const auto& [x, y] : points) {
        mesh.vertices.push_back(
            {static_cast<float>(x), static_cast<float>(y), static_cast<float>(roof.z(x, y))});
    }
    for (const auto& t : tris) {
        mesh.indices.push_back({static_cast<std::uint32_t>(t[0]), static_cast<std::uint32_t>(t[1]),
                                static_cast<std::uint32_t>(t[2])});
        Record r{};
        r[record_kind] = kind_roof;
        r[record_style] = static_cast<float>(style + 1);
        set_normal(r, face_normal(mesh.vertices[t[0]], mesh.vertices[t[1]], mesh.vertices[t[2]]));
        r[record_base] = static_cast<float>(base);
        r[record_top] = static_cast<float>(base + roof.eaves);
        mesh.records.push_back(r);
    }
    return mesh;
}

// Bottom face of a raised part, facing down.
Mesh underside(std::span<const Point2> outer, std::span<const Polygon> holes, double low,
               double base, int style) {
    const RoofSurface flat{.base = base, .eaves = low - base};
    Mesh mesh = roof_mesh(outer, holes, flat, base, style);
    for (auto& t : mesh.indices) std::swap(t[0], t[2]);
    for (auto& r : mesh.records) {
        for (std::size_t c = 0; c < 3; ++c) r[record_normal + c] = -r[record_normal + c];
        r[record_kind] = kind_wall;
    }
    return mesh;
}

RoofSurface surface_of(const osm::Footprint& item, std::span<const Point2> outer,
                       std::span<const Polygon> holes, double base,
                       const raster::als::HeightGrid* heights, const raster::LocalFrame& origin) {
    if (heights != nullptr && item.guessed) {
        if (auto measured = measured_roof(outer, holes, base, *heights, origin)) return *measured;
    }
    return RoofSurface::of(item, outer, base);
}

// Unit normals of a rows x cols vertex grid from central differences
// (one-sided at the border, as numpy.gradient).
std::vector<Vec> grid_normals(const std::vector<Vec>& p, int rows, int cols) {
    const auto at = [&](int r, int c) -> const Vec& {
        return p[static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) +
                 static_cast<std::size_t>(c)];
    };
    const auto diff = [](const Vec& a, const Vec& b, double scale) {
        return Vec{(a[0] - b[0]) * scale, (a[1] - b[1]) * scale, (a[2] - b[2]) * scale};
    };
    std::vector<Vec> out(p.size());
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            Vec de{0, 0, 0};
            Vec dn{0, 0, 0};
            if (cols > 1) {
                if (c == 0)
                    de = diff(at(r, 1), at(r, 0), 1.0);
                else if (c == cols - 1)
                    de = diff(at(r, c), at(r, c - 1), 1.0);
                else
                    de = diff(at(r, c + 1), at(r, c - 1), 0.5);
            }
            if (rows > 1) {
                if (r == 0)
                    dn = diff(at(1, c), at(0, c), 1.0);
                else if (r == rows - 1)
                    dn = diff(at(r, c), at(r - 1, c), 1.0);
                else
                    dn = diff(at(r + 1, c), at(r - 1, c), 0.5);
            }
            Vec n = cross(de, dn);
            if (n[2] < 0) n = {-n[0], -n[1], -n[2]};
            out[static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) +
                static_cast<std::size_t>(c)] = normalized(n);
        }
    }
    return out;
}

// Boolean grid helpers for the tree canopy, rows x cols.
// Row-major index, in size_t so that large grids do not overflow int.
std::size_t grid_index(int r, int c, int cols) {
    return static_cast<std::size_t>(r) * static_cast<std::size_t>(cols) +
           static_cast<std::size_t>(c);
}

struct Mask {
    int rows = 0;
    int cols = 0;
    std::vector<std::uint8_t> v;

    Mask(int r, int c)
        : rows(r), cols(c), v(static_cast<std::size_t>(r) * static_cast<std::size_t>(c), 0) {}
    [[nodiscard]] bool at(int r, int c) const {
        return r >= 0 && c >= 0 && r < rows && c < cols && v[grid_index(r, c, cols)] != 0;
    }
    void set(int r, int c, bool b) { v[grid_index(r, c, cols)] = b ? 1 : 0; }
};

cv::Mat to_mat(const Mask& m) {
    image::quiet_opencv();
    cv::Mat out(m.rows, m.cols, CV_8U);
    std::ranges::copy(m.v, out.ptr<std::uint8_t>());
    return out;
}

Mask from_mat(const cv::Mat& mat) {
    Mask out(mat.rows, mat.cols);
    std::copy_n(mat.ptr<std::uint8_t>(), out.v.size(), out.v.begin());
    return out;
}

// Points outside the grid count as unset. cv::erode treats the border as set unless told otherwise.
constexpr int outside = cv::BORDER_CONSTANT;

// Set where the whole 3 x 3 neighbourhood is set.
Mask erode3(const Mask& m) {
    cv::Mat out;
    cv::erode(to_mat(m), out, cv::Mat::ones(3, 3, CV_8U), {-1, -1}, 1, outside, cv::Scalar(0));
    return from_mat(out);
}

// Set where any point of the 3 x 3 neighbourhood is set.
Mask dilate3(const Mask& m) {
    cv::Mat out;
    cv::dilate(to_mat(m), out, cv::Mat::ones(3, 3, CV_8U), {-1, -1}, 1, outside, cv::Scalar(0));
    return from_mat(out);
}

// Set in both masks.
Mask both(const Mask& a, const Mask& b) {
    Mask out(a.rows, a.cols);
    for (std::size_t i = 0; i < out.v.size(); ++i) out.v[i] = a.v[i] != 0 && b.v[i] != 0 ? 1 : 0;
    return out;
}

// Mean of the set points in the 3 x 3 neighbourhood; 0 without a set neighbour.
std::vector<double> mean3(const std::vector<double>& values, const Mask& m) {
    cv::Mat set;
    to_mat(m).convertTo(set, CV_64F);
    cv::Mat masked(m.rows, m.cols, CV_64F);
    std::ranges::copy(values, masked.ptr<double>());
    masked = masked.mul(set);
    cv::Mat sum;
    cv::Mat count;
    cv::boxFilter(masked, sum, CV_64F, {3, 3}, {-1, -1}, false, outside);
    cv::boxFilter(set, count, CV_64F, {3, 3}, {-1, -1}, false, outside);
    const cv::Mat mean = sum / cv::max(count, 1.0);
    return {mean.ptr<double>(), mean.ptr<double>() + mean.total()};
}

// Grid points inside, or within one step of, any building outline.
Mask building_mask(std::span<const osm::Footprint> footprints, const raster::LocalFrame& origin,
                   std::span<const double> xs, std::span<const double> ys) {
    Mask inside(static_cast<int>(ys.size()), static_cast<int>(xs.size()));
    const double step = xs[1] - xs[0];
    for (const auto& item : footprints) {
        for (const auto& raw : item.outer) {
            if (raw.size() < 3) continue;
            Polygon ring = ring_of(origin, raw);
            for (auto& [x, y] : ring) {
                x = (x - xs[0]) / step;
                y = (y - ys[0]) / step;
            }
            const std::vector<Polygon> rings{ring};
            // Points farther than 0.5 from the bounding box cannot be inside or on the outline.
            double x0 = ring.front().first;
            double x1 = x0;
            double y0 = ring.front().second;
            double y1 = y0;
            for (const auto& [x, y] : ring) {
                x0 = std::min(x0, x);
                x1 = std::max(x1, x);
                y0 = std::min(y0, y);
                y1 = std::max(y1, y);
            }
            const auto lo = [](double v) {
                return static_cast<int>(std::max(std::ceil(v - 0.5), 0.0));
            };
            const auto hi = [](double v, int n) {
                return static_cast<int>(std::min(std::floor(v + 0.5), static_cast<double>(n - 1)));
            };
            for (int r = lo(y0); r <= hi(y1, inside.rows); ++r) {
                for (int c = lo(x0); c <= hi(x1, inside.cols); ++c) {
                    const Point2 p{c, r};
                    // Filled polygon plus its outline pixels.
                    if (contains(ring, p) || edge_distance(p, rings) <= 0.5) inside.set(r, c, true);
                }
            }
        }
    }
    return dilate3(inside);
}

std::vector<double> arange(double start, double stop, double step) {
    std::vector<double> out;
    const auto count = static_cast<std::ptrdiff_t>(std::ceil((stop - start) / step));
    out.reserve(static_cast<std::size_t>(std::max<std::ptrdiff_t>(count, 0)));
    for (std::ptrdiff_t i = 0; i < count; ++i) out.push_back(start + static_cast<double>(i) * step);
    return out;
}

}  // namespace

void Mesh::append(const Mesh& other) {
    const auto offset = static_cast<std::uint32_t>(vertices.size());
    vertices.insert(vertices.end(), other.vertices.begin(), other.vertices.end());
    for (const auto& t : other.indices)
        indices.push_back({t[0] + offset, t[1] + offset, t[2] + offset});
    records.insert(records.end(), other.records.begin(), other.records.end());
}

namespace {

// IEEE 754 binary16, round to nearest even. Written out because Clang has no
// std::float16_t; the shader decodes with f16tof32.
std::uint16_t to_half(float value) {
    const auto f = std::bit_cast<std::uint32_t>(value);
    const auto sign = static_cast<std::uint16_t>((f >> 16U) & 0x8000U);
    const std::uint32_t abs = f & 0x7FFFFFFFU;
    if (abs >= 0x7F800000U)
        return static_cast<std::uint16_t>(sign | (abs > 0x7F800000U ? 0x7E00U : 0x7C00U));
    if (abs >= 0x477FF000U) return static_cast<std::uint16_t>(sign | 0x7C00U);
    if (abs < 0x38800000U) {
        // Subnormal half, or zero.
        if (abs < 0x33000000U) return sign;
        // value = mantissa * 2^(e - 150); a half subnormal counts 2^-24 steps.
        const std::uint32_t mantissa = (abs & 0x7FFFFFU) | 0x800000U;
        const auto shift = 126U - (abs >> 23U);
        const std::uint32_t half = mantissa >> shift;
        const std::uint32_t rest = mantissa & ((1U << shift) - 1U);
        const std::uint32_t mid = 1U << (shift - 1U);
        const std::uint32_t round = rest > mid || (rest == mid && (half & 1U) != 0U) ? 1U : 0U;
        return static_cast<std::uint16_t>(sign | (half + round));
    }
    const std::uint32_t rebased = abs - 0x38000000U;
    std::uint32_t half = rebased >> 13U;
    const std::uint32_t rest = rebased & 0x1FFFU;
    if (rest > 0x1000U || (rest == 0x1000U && (half & 1U) != 0U)) ++half;
    return static_cast<std::uint16_t>(sign | half);
}

float from_half(std::uint16_t h) {
    const std::uint32_t sign = (std::uint32_t{h} & 0x8000U) << 16U;
    const std::uint32_t exponent = (h >> 10U) & 0x1FU;
    const std::uint32_t mantissa = h & 0x3FFU;
    if (exponent == 0) {
        const float v = std::ldexp(static_cast<float>(mantissa), -24);
        return sign != 0 ? -v : v;
    }
    if (exponent == 31) return std::bit_cast<float>(sign | 0x7F800000U | (mantissa << 13U));
    return std::bit_cast<float>(sign | ((exponent + 112U) << 23U) | (mantissa << 13U));
}

}  // namespace

float pack_normal(double x, double y) {
    const auto hx = to_half(static_cast<float>(x));
    const auto hy = to_half(static_cast<float>(y));
    return std::bit_cast<float>(static_cast<std::uint32_t>(hx) |
                                (static_cast<std::uint32_t>(hy) << 16U));
}

std::pair<float, float> unpack_normal(float slot) {
    const auto bits = std::bit_cast<std::uint32_t>(slot);
    return {from_half(static_cast<std::uint16_t>(bits & 0xFFFFU)),
            from_half(static_cast<std::uint16_t>(bits >> 16U))};
}

std::vector<std::array<float, 3>> face_normals(const Mesh& mesh) {
    std::vector<std::array<float, 3>> out;
    out.reserve(mesh.indices.size());
    for (const auto& t : mesh.indices)
        out.push_back(face_normal(mesh.vertices[t[0]], mesh.vertices[t[1]], mesh.vertices[t[2]]));
    return out;
}

RoofSurface RoofSurface::of(const osm::Footprint& item, std::span<const Point2> ring, double base) {
    if (!item.roof) return {.base = base, .eaves = item.wall.value_or(item.height)};
    const auto box = osm::min_area_box(ring);
    const std::optional<double> top = item.wall ? std::nullopt : std::optional<double>(item.height);
    const auto [eaves, rise] = osm::split_height(*item.roof, box, top, item.wall);
    return {.base = base,
            .eaves = eaves,
            .rise = rise,
            .planes = osm::roof_planes(*item.roof, box),
            .roof = item.roof,
            .box = box};
}

double RoofSurface::z(double x, double y) const {
    if (!planes)
        return base + osm::roof_height(roof.value(), box.value(), x, y, std::nullopt, eaves);
    if (planes->empty()) return base + eaves;
    double f = std::numeric_limits<double>::infinity();
    for (const auto& [a, b, c] : *planes) f = std::min(f, a * x + b * y + c);
    return base + eaves + rise * std::clamp(f, 0.0, 1.0);
}

std::vector<double> RoofSurface::breaks(Point2 p, Point2 q) const {
    std::vector<double> ts;
    if (!planes) {
        for (int i = 0; i <= dome_segments; ++i)
            ts.push_back(static_cast<double>(i) / dome_segments);
        return ts;
    }
    ts = {0.0, 1.0};
    std::vector<std::pair<double, double>> values;
    for (const auto& [a, b, c] : *planes) {
        values.emplace_back(a * p.first + b * p.second + c,
                            a * (q.first - p.first) + b * (q.second - p.second));
    }
    for (std::size_t i = 0; i < values.size(); ++i) {
        for (std::size_t j = i + 1; j < values.size(); ++j) {
            const double f0 = values[i].first - values[j].first;
            const double df = values[i].second - values[j].second;
            if (std::abs(df) > 1e-12) {
                const double t = -f0 / df;
                if (t > 0.0 && t < 1.0) ts.push_back(t);
            }
        }
    }
    std::ranges::sort(ts);
    return ts;
}

std::optional<RoofSurface> measured_roof(std::span<const Point2> outer,
                                         std::span<const Polygon> holes, double base,
                                         const raster::als::HeightGrid& grid,
                                         const raster::LocalFrame& origin) {
    ScanProbe probe{.grid = grid, .origin = origin, .rings = {Polygon(outer.begin(), outer.end())}};
    probe.rings.insert(probe.rings.end(), holes.begin(), holes.end());
    double x0 = std::numeric_limits<double>::infinity();
    double y0 = x0;
    double x1 = -x0;
    double y1 = -x0;
    for (const auto& [x, y] : outer) {
        x0 = std::min(x0, x);
        y0 = std::min(y0, y);
        x1 = std::max(x1, x);
        y1 = std::max(y1, y);
    }
    const double step = std::max(1.0, std::sqrt((x1 - x0) * (y1 - y0) / fit_samples));
    std::vector<Point2> pts;
    for (const double y : arange(y0, y1, step)) {
        for (const double x : arange(x0, x1, step)) {
            const Point2 p{x + step / 2, y + step / 2};
            if (contains(outer, p)) pts.push_back(p);
        }
    }
    if (pts.size() < 4) pts.assign(outer.begin(), outer.end());
    std::vector<double> h;
    h.reserve(pts.size());
    for (const auto& p : pts) h.push_back(probe.above(p));
    const auto finite =
        static_cast<double>(std::ranges::count_if(h, [](double v) { return std::isfinite(v); }));
    if (finite / static_cast<double>(h.size()) < 0.7 || !(percentile(h, 50) >= measured_min_m))
        return std::nullopt;
    std::vector<Point2> ok_pts;
    std::vector<double> ok_h;
    for (std::size_t i = 0; i < h.size(); ++i) {
        if (std::isfinite(h[i])) {
            ok_pts.push_back(pts[i]);
            ok_h.push_back(h[i]);
        }
    }
    std::vector<double> edge;
    for (const auto& p : outer) edge.push_back(probe.above(p));
    const double edge_eaves = percentile(edge, 20);
    const double eaves = std::isfinite(edge_eaves) ? edge_eaves : percentile(ok_h, 10);
    const double ridge = percentile(ok_h, 95);
    const auto box = osm::min_area_box(outer);
    const double median = percentile(ok_h, 50);
    const auto rms_of = [&](const auto& model) {
        double sum = 0;
        for (std::size_t i = 0; i < ok_h.size(); ++i) {
            const double d = ok_h[i] - model(ok_pts[i]);
            sum += d * d;
        }
        return std::sqrt(sum / static_cast<double>(ok_h.size()));
    };
    double best_rms = rms_of([&](Point2) { return median; });
    RoofSurface best{.base = base, .eaves = median};
    const double rise = ridge - eaves;
    if (rise > measured_flat_m) {
        for (const auto& [shape, across] : fit_shapes) {
            const osm::Roof roof{.shape = shape, .height = rise, .across = across};
            const double rms = rms_of([&](Point2 p) {
                return static_cast<double>(static_cast<float>(
                    osm::roof_height(roof, box, p.first, p.second, std::nullopt, eaves)));
            });
            if (rms < best_rms) {
                best_rms = rms;
                best = {.base = base,
                        .eaves = eaves,
                        .rise = rise,
                        .planes = osm::roof_planes(roof, box),
                        .roof = roof,
                        .box = box};
            }
        }
    }
    // No standard shape fits (towers, terraces, trees over the roof): keep the
    // OSM roof. A height field from the 1 m scan looked crumpled there.
    if (best_rms <= fit_rms_m) return best;
    return std::nullopt;
}

Mesh terrain_tile(const geodata::Mosaic& dem, int zoom, int tx, int ty, int quads,
                  const raster::LocalFrame& origin, double z0) {
    const int n = quads + 1;
    const double step = tile / quads;
    std::vector<double> heights(static_cast<std::size_t>(n * n), nan);
    std::vector<double> known;
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            const auto v = dem.sample(tx * tile + i * step, ty * tile + j * step);
            if (v) {
                heights[grid_index(j, i, n)] = *v;
                known.push_back(*v);
            }
        }
    }
    const double fallback = known.empty() ? z0 : percentile(known, 50);
    std::vector<Vec> grid(static_cast<std::size_t>(n * n));
    Mesh mesh;
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            const auto idx = grid_index(j, i, n);
            const double h = std::isfinite(heights[idx]) ? heights[idx] : fallback;
            const auto [lat, lon] = tile_latlon(tx * tile + i * step, ty * tile + j * step, zoom);
            const auto [e, nn] = local(origin, lat, lon);
            grid[idx] = {e, nn, h - z0};
            mesh.vertices.push_back(
                {static_cast<float>(e), static_cast<float>(nn), static_cast<float>(h - z0)});
        }
    }
    const auto id = [n](int j, int i) { return static_cast<std::uint32_t>(j * n + i); };
    // Rows run south (mercator y grows south); this winding gives upward normals.
    for (int j = 0; j < quads; ++j) {
        for (int i = 0; i < quads; ++i)
            mesh.indices.push_back({id(j, i), id(j + 1, i), id(j, i + 1)});
    }
    for (int j = 0; j < quads; ++j) {
        for (int i = 0; i < quads; ++i)
            mesh.indices.push_back({id(j, i + 1), id(j + 1, i), id(j + 1, i + 1)});
    }
    const auto top_count = mesh.indices.size();

    std::vector<std::uint32_t> border;
    border.reserve(4 * static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) border.push_back(id(0, i));
    for (int j = 1; j < n; ++j) border.push_back(id(j, n - 1));
    for (int i = n - 2; i >= 0; --i) border.push_back(id(n - 1, i));
    for (int j = n - 2; j >= 1; --j) border.push_back(id(j, 0));
    const auto& c0 = mesh.vertices[id(0, 0)];
    const auto& c1 = mesh.vertices[id(0, n - 1)];
    const auto depth =
        static_cast<float>(skirt_share * std::hypot(double{c1[0]} - c0[0], double{c1[1]} - c0[1]));
    const auto base = static_cast<std::uint32_t>(mesh.vertices.size());
    for (const auto b : border) {
        auto v = mesh.vertices[b];
        v[2] -= depth;
        mesh.vertices.push_back(v);
    }
    const auto ring = static_cast<std::uint32_t>(border.size());
    for (std::uint32_t r = 0; r < ring; ++r)
        mesh.indices.push_back({border[r], base + r, border[(r + 1) % ring]});
    for (std::uint32_t r = 0; r < ring; ++r) {
        mesh.indices.push_back({border[(r + 1) % ring], base + r, base + (r + 1) % ring});
    }

    const auto smooth = grid_normals(grid, n, n);
    for (std::size_t t = 0; t < mesh.indices.size(); ++t) {
        const auto& tri = mesh.indices[t];
        Record r{};
        r[record_kind] = kind_terrain;
        const auto face =
            face_normal(mesh.vertices[tri[0]], mesh.vertices[tri[1]], mesh.vertices[tri[2]]);
        set_normal(r, face);
        for (std::size_t k = 0; k < 3; ++k) {
            r[record_vertex_normals + k] = t < top_count
                                               ? pack_normal(smooth[tri[k]][0], smooth[tri[k]][1])
                                               : pack_normal(face[0], face[1]);
        }
        mesh.records.push_back(r);
    }
    return mesh;
}

namespace {

std::vector<Polygon> local_rings(const raster::LocalFrame& origin,
                                 std::span<const osm::Ring> rings) {
    std::vector<Polygon> out;
    for (const auto& r : rings) {
        if (r.size() >= 3) out.push_back(ring_of(origin, r));
    }
    return out;
}

// The holes inside one outline, clockwise.
std::vector<Polygon> holes_inside(const Polygon& outer, std::span<const Polygon> holes) {
    std::vector<Polygon> inside;
    for (const auto& h : holes) {
        if (contains(outer, h.front())) {
            Polygon cw = ccw(h);
            std::ranges::reverse(cw);
            inside.push_back(std::move(cw));
        }
    }
    return inside;
}

// Only features near this outline; a cell holds hundreds of buildings.
std::vector<LocalFeature> features_near(const Polygon& outer,
                                        std::span<const LocalFeature> features) {
    constexpr double margin = 2.0;
    double west = std::numeric_limits<double>::infinity();
    double south = west;
    double east = -west;
    double north = -west;
    for (const auto& [x, y] : outer) {
        west = std::min(west, x);
        east = std::max(east, x);
        south = std::min(south, y);
        north = std::max(north, y);
    }
    std::vector<LocalFeature> near;
    for (const auto& f : features) {
        const auto [x, y] = f.at;
        if (x > west - margin && x < east + margin && y > south - margin && y < north + margin)
            near.push_back(f);
    }
    return near;
}

// The window layout and colour of a style row.
struct FacadeLook {
    std::size_t layout;
    double bay;
    double floor;  // storey height of the layout
    Vec3 colour;
};

FacadeLook facade_look(const osm::StyleTable::Snapshot& table, int style) {
    const auto& row =
        table.rows[std::min(static_cast<std::size_t>(std::max(style, 0)), table.rows.size() - 1)];
    const auto layout = std::clamp(static_cast<std::size_t>(row[3]), std::size_t{0},
                                   shading::window_layouts.size() - 1);
    return {.layout = layout,
            .bay = shading::window_layouts[layout].bay_m,
            .floor = shading::window_layouts[layout].floor_m,
            .colour = Vec3{row[0], row[1], row[2]}};
}

struct BuildingPart {
    const osm::Footprint& item;
    const Polygon& outer;            // counter-clockwise
    std::span<const Polygon> holes;  // clockwise
    double base;
};

// Walls, balconies, roof and underside of one outline with its holes.
void append_part(Mesh& out, const BuildingPart& part, const FacadeLook& look,
                 std::span<const LocalFeature> near, const raster::als::HeightGrid* heights,
                 const raster::LocalFrame& origin, bool with_balconies) {
    const auto& item = part.item;
    const auto surface = surface_of(item, part.outer, part.holes, part.base, heights, origin);
    const double low = item.bottom > 0 ? part.base + item.bottom : part.base - bury_m;
    const double storey = storey_of(item.levels, surface.eaves);
    out.append(walls(part.outer, part.base, surface, item.style, look.bay, low, storey,
                     feature_spans(part.outer, near, true)));
    for (const auto& hole : part.holes) {
        out.append(walls(hole, part.base, surface, item.style, look.bay, low, storey,
                         feature_spans(hole, near, false)));
    }
    if (with_balconies && item.bottom <= 0) {
        const double floor_m = storey > 0 ? storey : look.floor;
        const auto seed = static_cast<std::uint64_t>(item.id);
        out.append(balconies(part.outer, part.base, surface.eaves, look.layout, floor_m, look.bay,
                             look.colour, seed, false));
        for (std::size_t h = 0; h < part.holes.size(); ++h) {
            out.append(balconies(part.holes[h], part.base, surface.eaves, look.layout, floor_m,
                                 look.bay, look.colour, mix(seed + h + 1), true));
        }
    }
    out.append(roof_mesh(part.outer, part.holes, surface, part.base, item.style));
    if (item.bottom > 0) out.append(underside(part.outer, part.holes, low, part.base, item.style));
}

}  // namespace

Mesh buildings(std::span<const osm::Footprint> footprints, const raster::LocalFrame& origin,
               double z0, const GroundFn& ground, bool detail,
               const raster::als::HeightGrid* heights, std::span<const osm::Feature> features,
               bool with_balconies) {
    const auto table = osm::styles().snapshot();
    std::vector<LocalFeature> local_features;
    for (const auto& f : features)
        local_features.push_back({local(origin, f.at.lat, f.at.lon), f.kind});
    Mesh out;
    for (const auto& item : osm::select_footprints(footprints, detail)) {
        const auto rings = local_rings(origin, item.outer);
        if (rings.empty() || item.height <= 0) continue;
        const auto holes = local_rings(origin, item.inner);
        double lowest = std::numeric_limits<double>::infinity();
        for (const auto& p : item.outer.front()) lowest = std::min(lowest, ground(p.lat, p.lon));
        const double base = lowest - z0;
        const auto look = facade_look(table, item.style);
        for (const auto& ring : rings) {
            const Polygon outer = ccw(ring);
            const auto inside = holes_inside(outer, holes);
            append_part(out, {.item = item, .outer = outer, .holes = inside, .base = base}, look,
                        features_near(outer, local_features), heights, origin, with_balconies);
        }
    }
    return out;
}

double roof_height_at(std::span<const osm::Footprint> footprints, const raster::LocalFrame& origin,
                      double z0, const GroundFn& ground, double lat, double lon,
                      const raster::als::HeightGrid* heights) {
    const Point2 point = local(origin, lat, lon);
    double best = 0.0;
    for (const auto& item : osm::select_footprints(footprints, true)) {
        for (const auto& raw : item.outer) {
            if (raw.size() < 3) continue;
            const Polygon ring = ccw(ring_of(origin, raw));
            if (!contains(ring, point)) continue;
            double lowest = std::numeric_limits<double>::infinity();
            for (const auto& p : raw) lowest = std::min(lowest, ground(p.lat, p.lon));
            const auto surface = surface_of(item, ring, {}, lowest - z0, heights, origin);
            best = std::max(best, surface.z(point.first, point.second));
        }
    }
    return best;
}

namespace {

// Scan heights on a grid twice as fine as the mesh grid: a roof slope of 5 m is too
// narrow for a flat 3 x 3 block at tree_step_m.
struct TreeSamples {
    std::vector<double> xs;  // mesh grid, every second fine point
    std::vector<double> ys;
    std::vector<double> fine_x;
    std::vector<double> fine_y;
    std::vector<double> fine;  // height above ground, row-major on the fine grid
    Mask tall{0, 0};

    [[nodiscard]] int fine_rows() const { return static_cast<int>(fine_y.size()); }
    [[nodiscard]] int fine_cols() const { return static_cast<int>(fine_x.size()); }
    [[nodiscard]] int rows() const { return static_cast<int>(ys.size()); }
    [[nodiscard]] int cols() const { return static_cast<int>(xs.size()); }
};

std::optional<TreeSamples> sample_trees(const raster::LocalFrame& origin,
                                        const raster::als::HeightGrid& heights,
                                        const LatLonBounds& bounds) {
    const auto [e0, n0] = local(origin, bounds.south, bounds.west);
    const auto [e1, n1] = local(origin, bounds.north, bounds.east);
    TreeSamples s;
    s.fine_x = arange(e0, e1, tree_step_m / 2);
    s.fine_y = arange(n0, n1, tree_step_m / 2);
    for (std::size_t i = 0; i < s.fine_x.size(); i += 2) s.xs.push_back(s.fine_x[i]);
    for (std::size_t i = 0; i < s.fine_y.size(); i += 2) s.ys.push_back(s.fine_y[i]);
    if (s.xs.size() < 2 || s.ys.size() < 2) return std::nullopt;
    const int fr = s.fine_rows();
    const int fc = s.fine_cols();
    s.fine.resize(static_cast<std::size_t>(fr) * static_cast<std::size_t>(fc));
    s.tall = Mask(fr, fc);
    for (int r = 0; r < fr; ++r) {
        for (int c = 0; c < fc; ++c) {
            const double v = heights.sample(
                origin.lat0 + s.fine_y[static_cast<std::size_t>(r)] / origin.m_per_deg_lat(),
                origin.lon0 + s.fine_x[static_cast<std::size_t>(c)] / origin.m_per_deg_lon());
            s.fine[grid_index(r, c, fc)] = v;
            s.tall.set(r, c, std::isfinite(v) && v > tree_min_m);
        }
    }
    return s;
}

// Crowns are rough. A smooth patch is a roof that OSM does not have.
Mask roof_structure(const TreeSamples& s) {
    std::vector<double> masked(s.fine.size());
    for (std::size_t i = 0; i < s.fine.size(); ++i) masked[i] = s.tall.v[i] != 0 ? s.fine[i] : 0.0;
    const auto mean = mean3(masked, s.tall);
    Mask flat(s.fine_rows(), s.fine_cols());
    for (std::size_t i = 0; i < s.fine.size(); ++i)
        flat.v[i] = s.tall.v[i] != 0 && std::abs(mean[i] - s.fine[i]) < tree_flat_m ? 1 : 0;
    Mask structure = erode3(flat);
    for (int g = 0; g < tree_structure_grow; ++g) structure = both(dilate3(structure), s.tall);
    return structure;
}

struct Canopy {
    Mask mask{0, 0};
    std::vector<double> above;  // crown height above ground, smoothed
};

Canopy canopy_of(const TreeSamples& s, const Mask& structure, const Mask& buildings) {
    const int h = s.rows();
    const int w = s.cols();
    const int fc = s.fine_cols();
    Canopy out{
        .mask = Mask(h, w),
        .above = std::vector<double>(static_cast<std::size_t>(h) * static_cast<std::size_t>(w))};
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            const auto fi = grid_index(2 * r, 2 * c, fc);
            out.above[grid_index(r, c, w)] = s.fine[fi];
            out.mask.set(r, c, s.tall.v[fi] != 0 && structure.v[fi] == 0 && !buildings.at(r, c));
        }
    }
    // Opening: a strip of two points or less is noise (a lamp post, the edge of
    // a roof that OSM does not have). A crown of three steps or more stays.
    out.mask = both(dilate3(erode3(out.mask)), out.mask);
    // The 1 m scan is noisy; without the mean the crowns become spikes.
    for (std::size_t i = 0; i < out.above.size(); ++i)
        out.above[i] = out.mask.v[i] != 0 ? out.above[i] : 0.0;
    out.above = mean3(out.above, out.mask);
    return out;
}

// Grid cells whose four corners are canopy.
Mask canopy_quads(const Mask& canopy) {
    Mask quads(canopy.rows, canopy.cols);
    for (int r = 0; r + 1 < canopy.rows; ++r) {
        for (int c = 0; c + 1 < canopy.cols; ++c) {
            quads.set(r, c,
                      canopy.at(r, c) && canopy.at(r + 1, c) && canopy.at(r, c + 1) &&
                          canopy.at(r + 1, c + 1));
        }
    }
    return quads;
}

struct TreeSurface {
    std::vector<Vec> grid;      // ground plus crown, local metres
    std::vector<double> floor;  // ground, local metres
};

TreeSurface tree_surface(const TreeSamples& s, const Canopy& canopy,
                         const raster::LocalFrame& origin, double z0, const GroundFn& ground) {
    const int h = s.rows();
    const int w = s.cols();
    TreeSurface out{
        .grid = std::vector<Vec>(static_cast<std::size_t>(h) * static_cast<std::size_t>(w)),
        .floor = {}};
    out.floor.resize(out.grid.size());
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            const auto i = grid_index(r, c, w);
            const double x = s.xs[static_cast<std::size_t>(c)];
            const double y = s.ys[static_cast<std::size_t>(r)];
            out.floor[i] = ground(origin.lat0 + y / origin.m_per_deg_lat(),
                                  origin.lon0 + x / origin.m_per_deg_lon()) -
                           z0;
            out.grid[i] = {x, y, out.floor[i] + (canopy.mask.v[i] != 0 ? canopy.above[i] : 0.0)};
        }
    }
    return out;
}

struct TreeQuad {
    int r;
    int c;
    std::size_t a;  // corners: a (r, c), b (r, c + 1), cc (r + 1, c), d (r + 1, c + 1)
    std::size_t b;
    std::size_t cc;
    std::size_t d;
};
using IndexTri = std::array<std::size_t, 3>;

std::vector<TreeQuad> quads_of(const Mask& quads) {
    const int w = quads.cols;
    const auto id = [w](int r, int c) { return grid_index(r, c, w); };
    std::vector<TreeQuad> qs;
    for (int r = 0; r < quads.rows; ++r) {
        for (int c = 0; c < w; ++c) {
            if (!quads.at(r, c)) continue;
            qs.push_back({r, c, id(r, c), id(r, c + 1), id(r + 1, c), id(r + 1, c + 1)});
        }
    }
    return qs;
}

// Two triangles per quad: first every a-b-d, then every a-d-c.
std::vector<IndexTri> tree_tops(const std::vector<TreeQuad>& qs) {
    std::vector<IndexTri> all;
    all.reserve(2 * qs.size());
    for (const auto& q : qs) all.push_back({q.a, q.b, q.d});
    for (const auto& q : qs) all.push_back({q.a, q.d, q.cc});
    return all;
}

// Sides where the canopy ends: from the canopy edge down to the crown base.
void tree_sides(const std::vector<TreeQuad>& qs, const Mask& quads, const TreeSurface& surface,
                std::vector<Vec>& vertices, std::vector<IndexTri>& all) {
    struct SideDir {
        int dr;
        int dc;
        int p;  // 0 a, 1 b, 2 c, 3 d
        int q;
    };
    constexpr std::array<SideDir, 4> dirs{{{.dr = -1, .dc = 0, .p = 0, .q = 1},
                                           {.dr = 1, .dc = 0, .p = 3, .q = 2},
                                           {.dr = 0, .dc = -1, .p = 2, .q = 0},
                                           {.dr = 0, .dc = 1, .p = 1, .q = 3}}};
    for (const auto& dir : dirs) {
        std::vector<std::size_t> ps;
        std::vector<std::size_t> qv;
        for (const auto& q : qs) {
            if (quads.at(q.r + dir.dr, q.c + dir.dc)) continue;
            const std::array<std::size_t, 4> corner{q.a, q.b, q.cc, q.d};
            ps.push_back(corner[static_cast<std::size_t>(dir.p)]);
            qv.push_back(corner[static_cast<std::size_t>(dir.q)]);
        }
        if (ps.empty()) continue;
        const auto lower = [&](std::size_t v) {
            Vec low = surface.grid[v];
            low[2] = surface.floor[v] + (surface.grid[v][2] - surface.floor[v]) * tree_crown_share;
            // Sloped sides take the orthophoto colour of the crown rim, not vertical streaks.
            low[0] += dir.dc * tree_side_slope_m;
            low[1] += dir.dr * tree_side_slope_m;
            return low;
        };
        const auto k = ps.size();
        const auto first = vertices.size();
        for (const auto p : ps) vertices.push_back(lower(p));
        for (const auto p : qv) vertices.push_back(lower(p));
        for (std::size_t i = 0; i < k; ++i) all.push_back({ps[i], first + i, first + k + i});
        for (std::size_t i = 0; i < k; ++i) all.push_back({ps[i], first + k + i, qv[i]});
    }
}

// Keep only the used vertices, in their order. The first tops triangles take the smooth
// grid normals, the sides their face normal.
Mesh compact_tree_mesh(const std::vector<Vec>& vertices, const std::vector<IndexTri>& all,
                       std::size_t tops, const std::vector<Vec>& smooth) {
    std::vector<std::int64_t> remap(vertices.size(), -1);
    for (const auto& t : all) {
        for (const auto v : t) remap[v] = 0;
    }
    Mesh mesh;
    for (std::size_t v = 0; v < vertices.size(); ++v) {
        if (remap[v] < 0) continue;
        remap[v] = static_cast<std::int64_t>(mesh.vertices.size());
        mesh.vertices.push_back({static_cast<float>(vertices[v][0]),
                                 static_cast<float>(vertices[v][1]),
                                 static_cast<float>(vertices[v][2])});
    }
    for (std::size_t t = 0; t < all.size(); ++t) {
        const auto& tri = all[t];
        const Triangle out{static_cast<std::uint32_t>(remap[tri[0]]),
                           static_cast<std::uint32_t>(remap[tri[1]]),
                           static_cast<std::uint32_t>(remap[tri[2]])};
        mesh.indices.push_back(out);
        Record r{};
        r[record_kind] = kind_terrain;
        const auto face =
            face_normal(mesh.vertices[out[0]], mesh.vertices[out[1]], mesh.vertices[out[2]]);
        set_normal(r, face);
        for (std::size_t k = 0; k < 3; ++k) {
            Vec n = t < tops ? smooth[tri[k]] : Vec{face[0], face[1], face[2]};
            if (n[2] < 0) n = {-n[0], -n[1], -n[2]};
            r[record_vertex_normals + k] = pack_normal(n[0], n[1]);
        }
        mesh.records.push_back(r);
    }
    return mesh;
}

}  // namespace

Mesh trees(std::span<const osm::Footprint> footprints, const raster::LocalFrame& origin, double z0,
           const GroundFn& ground, const raster::als::HeightGrid& heights,
           const LatLonBounds& bounds) {
    const auto samples = sample_trees(origin, heights, bounds);
    if (!samples) return {};
    const auto structure = roof_structure(*samples);
    const auto canopy =
        canopy_of(*samples, structure, building_mask(footprints, origin, samples->xs, samples->ys));
    const auto quads = canopy_quads(canopy.mask);
    if (std::ranges::none_of(quads.v, [](std::uint8_t v) { return v != 0; })) return {};

    const auto surface = tree_surface(*samples, canopy, origin, z0, ground);
    const auto smooth = grid_normals(surface.grid, samples->rows(), samples->cols());
    const auto qs = quads_of(quads);
    std::vector<Vec> vertices = surface.grid;
    auto all = tree_tops(qs);
    const auto tops = all.size();
    tree_sides(qs, quads, surface, vertices, all);
    return compact_tree_mesh(vertices, all, tops, smooth);
}

}  // namespace docksim::render::rt
