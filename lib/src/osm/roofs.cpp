#include "osm/roofs.hpp"

#include "osm/tags.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <ranges>
#include <set>

namespace docksim::osm {

namespace {

double cross(const Point& o, const Point& a, const Point& b) {
    return (a.first - o.first) * (b.second - o.second) -
           (a.second - o.second) * (b.first - o.first);
}

// Andrew's monotone chain, counter-clockwise, without the closing point.
std::vector<Point> convex_hull(std::span<const Point> points) {
    const std::set<Point> unique(points.begin(), points.end());
    std::vector<Point> pts(unique.begin(), unique.end());
    if (pts.size() < 3) return pts;
    std::vector<Point> lower;
    for (const auto& p : pts) {
        while (lower.size() >= 2 && cross(lower[lower.size() - 2], lower.back(), p) <= 0)
            lower.pop_back();
        lower.push_back(p);
    }
    std::vector<Point> upper;
    for (const auto& p : std::views::reverse(pts)) {
        while (upper.size() >= 2 && cross(upper[upper.size() - 2], upper.back(), p) <= 0)
            upper.pop_back();
        upper.push_back(p);
    }
    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
}

double auto_height(RoofShape shape, const Box& box) {
    const auto rad = [](double deg) { return deg * std::numbers::pi / 180.0; };
    if (shape == RoofShape::dome) return box.half_short;
    if (shape == RoofShape::skillion) return 2 * box.half_short * std::tan(rad(skillion_pitch_deg));
    return box.half_short * std::tan(rad(pitch_deg));
}

}  // namespace

std::optional<RoofShape> roof_shape(std::string_view value) {
    const auto v = lower(trim(value));
    if (v == "gabled" || v == "gambrel" || v == "saltbox") return RoofShape::gabled;
    if (v == "hipped" || v == "half-hipped" || v == "mansard" || v == "side_hipped")
        return RoofShape::hipped;
    if (v == "pyramidal") return RoofShape::pyramidal;
    if (v == "skillion" || v == "lean_to") return RoofShape::skillion;
    if (v == "dome" || v == "round" || v == "onion") return RoofShape::dome;
    return std::nullopt;
}

Box min_area_box(std::span<const Point> points) {
    const auto hull = convex_hull(points);
    bool found = false;
    double best_area = 0;
    double size_u = 0;
    double size_v = 0;
    Point best_u{1, 0};
    Point best_v{0, 1};
    Point centre{0, 0};
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const auto& p = hull[i];
        const auto& q = hull[(i + 1) % hull.size()];
        const double ex = q.first - p.first;
        const double ey = q.second - p.second;
        const double length = std::hypot(ex, ey);
        if (length == 0) continue;
        const Point u{ex / length, ey / length};
        const Point v{-u.second, u.first};
        double amin = std::numeric_limits<double>::infinity();
        double amax = -std::numeric_limits<double>::infinity();
        double bmin = std::numeric_limits<double>::infinity();
        double bmax = -std::numeric_limits<double>::infinity();
        for (const auto& h : hull) {
            const double a = h.first * u.first + h.second * u.second;
            const double b = h.first * v.first + h.second * v.second;
            amin = std::min(amin, a);
            amax = std::max(amax, a);
            bmin = std::min(bmin, b);
            bmax = std::max(bmax, b);
        }
        const double area = (amax - amin) * (bmax - bmin);
        if (!found || area < best_area) {
            found = true;
            best_area = area;
            size_u = amax - amin;
            size_v = bmax - bmin;
            best_u = u;
            best_v = v;
            const double ca = (amax + amin) / 2;
            const double cb = (bmax + bmin) / 2;
            centre = {u.first * ca + v.first * cb, u.second * ca + v.second * cb};
        }
    }
    if (!found) {
        double sx = 0;
        double sy = 0;
        for (const auto& p : points) {
            sx += p.first;
            sy += p.second;
        }
        const auto n = static_cast<double>(std::max<std::size_t>(points.size(), 1));
        return {
            .cx = sx / n, .cy = sy / n, .ux = 1.0, .uy = 0.0, .half_long = 0.0, .half_short = 0.0};
    }
    const Point axis = size_u >= size_v ? best_u : best_v;
    return {.cx = centre.first,
            .cy = centre.second,
            .ux = axis.first,
            .uy = axis.second,
            .half_long = std::max(size_u, size_v) / 2,
            .half_short = std::min(size_u, size_v) / 2};
}

std::pair<double, double> split_height(const Roof& roof, const Box& box, std::optional<double> top,
                                       std::optional<double> wall) {
    double roof_h = roof.height.value_or(0.0);
    if (roof_h == 0.0) roof_h = auto_height(roof.shape, box);
    if (top) {
        roof_h = std::min(roof_h, *top * max_roof_share);
        const double wall_h = std::max(*top - roof_h, min_wall_m);
        return {wall_h, std::max(*top - wall_h, 0.0)};
    }
    return {wall.value_or(0.0), roof_h};
}

double roof_height(const Roof& roof, const Box& box, double x, double y, std::optional<double> top,
                   std::optional<double> wall) {
    const auto [wall_h, roof_h] = split_height(roof, box, top, wall);
    const double dx = x - box.cx;
    const double dy = y - box.cy;
    double u = dx * box.ux + dy * box.uy;
    double v = -dx * box.uy + dy * box.ux;
    double hl = std::max(box.half_long, 1e-6);
    double hs = std::max(box.half_short, 1e-6);
    if (roof.across) {
        std::swap(u, v);
        std::swap(hl, hs);
    }
    double f = 0;
    switch (roof.shape) {
        case RoofShape::gabled:
            f = 1 - std::abs(v) / hs;
            break;
        case RoofShape::hipped:
            f = std::min(1 - std::abs(v) / hs, (hl - std::abs(u)) / hs);
            break;
        case RoofShape::pyramidal:
            f = 1 - std::max(std::abs(u) / hl, std::abs(v) / hs);
            break;
        case RoofShape::skillion:
            f = (v + hs) / (2 * hs);
            break;
        case RoofShape::dome:
            f = std::sqrt(std::max(0.0, 1 - (u / hl) * (u / hl) - (v / hs) * (v / hs)));
            break;
    }
    return wall_h + roof_h * std::clamp(f, 0.0, 1.0);
}

std::optional<std::vector<std::array<double, 3>>> roof_planes(const Roof& roof, const Box& box) {
    double hl = std::max(box.half_long, 1e-6);
    double hs = std::max(box.half_short, 1e-6);
    // u, v as linear functions of x, y: u = ux * x + uy * y + u0, v likewise.
    std::array<double, 3> u{box.ux, box.uy, -(box.cx * box.ux + box.cy * box.uy)};
    std::array<double, 3> v{-box.uy, box.ux, box.cx * box.uy - box.cy * box.ux};
    if (roof.across) {
        std::swap(u, v);
        std::swap(hl, hs);
    }
    const auto lin = [&](double cu, double cv, double c) {
        return std::array<double, 3>{cu * u[0] + cv * v[0], cu * u[1] + cv * v[1],
                                     cu * u[2] + cv * v[2] + c};
    };
    switch (roof.shape) {
        case RoofShape::gabled:
            return std::vector{lin(0, -1 / hs, 1), lin(0, 1 / hs, 1)};
        case RoofShape::hipped:
            return std::vector{lin(0, -1 / hs, 1), lin(0, 1 / hs, 1), lin(-1 / hs, 0, hl / hs),
                               lin(1 / hs, 0, hl / hs)};
        case RoofShape::pyramidal:
            return std::vector{lin(-1 / hl, 0, 1), lin(1 / hl, 0, 1), lin(0, -1 / hs, 1),
                               lin(0, 1 / hs, 1)};
        case RoofShape::skillion:
            return std::vector{lin(0, 1 / (2 * hs), 0.5)};
        case RoofShape::dome:
            return std::nullopt;
    }
    return std::nullopt;
}

}  // namespace docksim::osm
