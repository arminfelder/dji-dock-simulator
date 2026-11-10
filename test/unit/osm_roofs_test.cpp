#include "osm/footprints.hpp"
#include "osm/grid.hpp"
#include "osm/roofs.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

using namespace docksim;
using namespace docksim::osm;
using Catch::Approx;

namespace {

constexpr double lat0 = 47.0;
constexpr double lon0 = 11.0;
constexpr double m_lat = 111'195.0;
const double m_lon = m_lat * std::cos(lat0 * std::numbers::pi / 180.0);

std::vector<Point> rect(double width, double depth, double angle_deg = 0.0, double offset = 0.0) {
    const double a = angle_deg * std::numbers::pi / 180.0;
    const Point u{std::cos(a), std::sin(a)};
    const Point v{-std::sin(a), std::cos(a)};
    std::vector<Point> out;
    for (const auto& [x, y] : {std::pair{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}) {
        out.emplace_back(u.first * width / 2 * x + v.first * depth / 2 * y + offset,
                         u.second * width / 2 * x + v.second * depth / 2 * y + offset);
    }
    return out;
}

Ring square(double e, double n, double size) {
    Ring ring;
    for (const auto& [de, dn] : {std::pair{0.0, 0.0}, {size, 0.0}, {size, size}, {0.0, size}}) {
        ring.push_back({lat0 + (n + dn) / m_lat, lon0 + (e + de) / m_lon});
    }
    return ring;
}

}  // namespace

TEST_CASE("osm: min_area_box finds a rotated rectangle") {
    const auto box = min_area_box(rect(30, 10, 30, 100));
    CHECK(box.half_long == Approx(15));
    CHECK(box.half_short == Approx(5));
    CHECK(std::abs(box.ux * std::cos(std::numbers::pi / 6) +
                   box.uy * std::sin(std::numbers::pi / 6)) == Approx(1));
    CHECK(box.cx == Approx(100));
    CHECK(box.cy == Approx(100));
}

TEST_CASE("osm: gabled ridge and eaves") {
    const auto box = min_area_box(rect(20, 10));
    const Roof roof{.shape = RoofShape::gabled, .height = 4.0};
    CHECK(roof_height(roof, box, 0, 0, std::nullopt, 6.0) == Approx(10.0));
    CHECK(roof_height(roof, box, 0, 5, std::nullopt, 6.0) == Approx(6.0));
    CHECK(roof_height(roof, box, 9, 0, std::nullopt, 6.0) == Approx(10.0));
}

TEST_CASE("osm: hipped ends and pyramid peak") {
    const auto box = min_area_box(rect(20, 10));
    CHECK(roof_height({RoofShape::hipped, 4.0}, box, 10, 0, std::nullopt, 6.0) == Approx(6.0));
    CHECK(roof_height({RoofShape::pyramidal, 4.0}, box, 0, 0, std::nullopt, 6.0) == Approx(10.0));
}

TEST_CASE("osm: across turns the ridge") {
    const auto box = min_area_box(rect(20, 10));
    const Roof roof{.shape = RoofShape::gabled, .height = 4.0, .across = true};
    CHECK(roof_height(roof, box, 10, 0, std::nullopt, 6.0) == Approx(6.0));
    CHECK(roof_height(roof, box, 0, 5, std::nullopt, 6.0) == Approx(10.0));
}

TEST_CASE("osm: a roof never eats the walls") {
    const auto box = min_area_box(rect(40, 40));
    const Roof roof{.shape = RoofShape::gabled, .height = 30.0};
    CHECK(roof_height(roof, box, 0, 0, 10.0, std::nullopt) == Approx(10.0));
    CHECK(roof_height(roof, box, 0, 20, 10.0, std::nullopt) == Approx(10.0 * (1 - max_roof_share)));
}

TEST_CASE("osm: roof planes give the same surface") {
    const auto box = min_area_box(rect(20, 10, 25, 3));
    for (const auto shape :
         {RoofShape::gabled, RoofShape::hipped, RoofShape::pyramidal, RoofShape::skillion}) {
        const Roof roof{.shape = shape, .height = 4.0};
        const auto planes = roof_planes(roof, box);
        REQUIRE(planes);
        for (const auto& [x, y] : {std::pair{3.0, 3.0}, {5.0, 2.0}, {0.0, 4.0}}) {
            double f = INFINITY;
            for (const auto& [a, b, c] : *planes) f = std::min(f, a * x + b * y + c);
            CHECK(6.0 + 4.0 * std::clamp(f, 0.0, 1.0) ==
                  Approx(roof_height(roof, box, x, y, std::nullopt, 6.0)));
        }
    }
    CHECK_FALSE(roof_planes({RoofShape::dome}, box));
}

TEST_CASE("osm: tags give wall and roof") {
    const Json item = {
        {"id", 1},
        {"tags", {{"building", "house"}, {"building:levels", "2"}, {"roof:shape", "gabled"}}},
        {"outer",
         Json::array({Json::array({Json::array({lat0, lon0}), Json::array({lat0, lon0 + 1e-4}),
                                   Json::array({lat0 + 1e-4, lon0})})})},
        {"inner", Json::array()}};
    const auto fp = from_cache(item, {});
    CHECK(fp.roof == Roof{RoofShape::gabled, std::nullopt, false});
    REQUIRE(fp.wall);
    CHECK(*fp.wall == Approx(2 * 3.0 + roof_slab_m));
    CHECK_FALSE(fp.guessed);
    Json flat_item = item;
    flat_item["tags"] = {{"building", "yes"}, {"height", "12"}};
    const auto flat = from_cache(flat_item, {});
    CHECK_FALSE(flat.roof);
    CHECK(flat.height == 12.0);
    CHECK_FALSE(flat.guessed);
    flat_item["tags"] = {{"building", "yes"}};
    CHECK(from_cache(flat_item, {}).guessed);
}

TEST_CASE("osm: parts replace their outline only near") {
    const Footprint outline{.height = 30.0, .outer = {square(0, 0, 40)}};
    const Footprint part{.height = 30.0, .outer = {square(5, 5, 10)}, .part = true};
    const Footprint lone{.height = 10.0, .outer = {square(100, 100, 10)}};
    const std::vector items{outline, part, lone};
    const auto near = select_footprints(items, true);
    REQUIRE(near.size() == 2);
    CHECK(near[0].part);
    CHECK(near[1].height == 10.0);
    const auto far = select_footprints(items, false);
    REQUIRE(far.size() == 2);
    CHECK(far[0].height == 30.0);
    CHECK_FALSE(far[0].part);
}

TEST_CASE("osm: a rasterized gabled roof has a ridge") {
    Ring ring;
    for (const auto& [e, n] : {std::pair{-10.0, -5.0}, {10.0, -5.0}, {10.0, 5.0}, {-10.0, 5.0}}) {
        ring.push_back({lat0 + n / m_lat, lon0 + e / m_lon});
    }
    const std::vector house{Footprint{.height = 12.0,
                                      .outer = {ring},
                                      .roof = Roof{.shape = RoofShape::gabled, .height = 4.0},
                                      .wall = 8.0}};
    const auto near = rasterize(house, lat0, lon0, 30.0, 0.5, true);
    const auto far = rasterize(house, lat0, lon0, 30.0, 0.5, false);
    CHECK(near.height_at(lat0, lon0) == Approx(12.0).margin(0.3));
    CHECK(near.height_at(lat0 + 4.6 / m_lat, lon0) < 9.0);
    CHECK(far.height_at(lat0 + 4.6 / m_lat, lon0) == 12.0F);
}
