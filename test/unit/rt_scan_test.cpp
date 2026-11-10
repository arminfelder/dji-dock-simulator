#include "raster/als.hpp"
#include "render/rt/meshes.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <functional>
#include <limits>

using namespace docksim;
using namespace docksim::render::rt;
using Catch::Approx;

namespace {

const raster::LocalFrame origin{.lat0 = 48.2, .lon0 = 16.37};

// A 1 m EPSG:3035 grid over 120 x 120 m around origin; height(east, north) in
// metres. EPSG:3035 grid north is about 5 degrees off true north in Vienna, so
// each grid pixel maps back to local east/north through an affine fit.
raster::als::HeightGrid grid_of(const std::function<double(double, double)>& height) {
    const auto at = [](double east, double north) {
        return raster::als::laea(origin.lat0 + north / origin.m_per_deg_lat(),
                                 origin.lon0 + east / origin.m_per_deg_lon());
    };
    const auto [x0, y0] = at(0, 0);
    const auto [xe, ye] = at(100, 0);
    const auto [xn, yn] = at(0, 100);
    // Columns of to_laea: laea offset per metre east, per metre north.
    const double a = (xe - x0) / 100;
    const double c = (ye - y0) / 100;
    const double b = (xn - x0) / 100;
    const double d = (yn - y0) / 100;
    const double det = a * d - b * c;
    raster::Grid2D<float> heights(120, 120);
    const double xs0 = x0 - 60;
    const double ys0 = y0 + 60;
    for (int row = 0; row < 120; ++row) {
        for (int col = 0; col < 120; ++col) {
            const double gx = xs0 + col - x0;
            const double gy = ys0 - row - y0;
            const double east = (d * gx - b * gy) / det;
            const double north = (-c * gx + a * gy) / det;
            heights(col, row) = static_cast<float>(height(east, north));
        }
    }
    return {std::move(heights), xs0, ys0};
}

std::vector<Point2> square(double half) {
    return {{-half, -half}, {half, -half}, {half, half}, {-half, half}};
}

bool inside(double e, double n, double half) {
    return std::abs(e) <= half && std::abs(n) <= half;
}

}  // namespace

TEST_CASE("the scan picks a flat roof with the measured height") {
    const auto grid = grid_of([](double e, double n) { return inside(e, n, 15) ? 21.0 : 0.0; });
    const auto roof = measured_roof(square(14), {}, 0.0, grid, origin);
    REQUIRE(roof);
    CHECK(roof->flat());
    CHECK(roof->eaves == Approx(21.0).margin(0.1));
}

TEST_CASE("the scan picks a gabled roof") {
    // 30 x 14 m house, eaves 12 m, ridge 18 m along the long (east-west) side.
    const auto grid = grid_of([](double e, double n) {
        return std::abs(e) <= 15 && std::abs(n) <= 7 ? 12.0 + 6.0 * (1 - std::abs(n) / 7.0) : 0.0;
    });
    const std::vector<Point2> ring{{-15, -7}, {15, -7}, {15, 7}, {-15, 7}};
    const auto roof = measured_roof(ring, {}, 0.0, grid, origin);
    REQUIRE(roof);
    REQUIRE(roof->roof);
    CHECK(roof->roof->shape == osm::RoofShape::gabled);
    CHECK(roof->eaves + roof->rise == Approx(18.0).margin(1.0));
}

TEST_CASE("without a fitting shape the OSM roof stays") {
    // A 50 m tower with four 8 m deep pits: no standard roof fits.
    const auto grid = grid_of([](double e, double n) {
        double h = inside(e, n, 20) ? 50.0 : 0.0;
        for (const auto& [pe, pn] : {std::pair{-10, 10}, {10, 10}, {-10, -10}, {10, -10}}) {
            if (std::hypot(e - pe, n - pn) < 6) h = 30.0;
        }
        return h;
    });
    CHECK_FALSE(measured_roof(square(20), {}, 0.0, grid, origin));
}

TEST_CASE("without scan data the OSM roof stays") {
    const auto grid =
        grid_of([](double, double) { return std::numeric_limits<double>::quiet_NaN(); });
    CHECK_FALSE(measured_roof(square(10), {}, 0.0, grid, origin));
}

TEST_CASE("trees come from the scan outside buildings") {
    const auto latlon = [](double east, double north) {
        return osm::LatLon{.lat = origin.lat0 + north / origin.m_per_deg_lat(),
                           .lon = origin.lon0 + east / origin.m_per_deg_lon()};
    };
    // A 12 m round crown at (25, 25), a 15 m house at (-25, -25), one lamp post at
    // (0, -30), and a flat 8 m roof at (-25, 25) that OSM does not have.
    const auto grid = grid_of([](double e, double n) {
        const double r = std::hypot(e - 25, n - 25);
        // Leaves make a crown rough at 1 m.
        const double leaves = std::sin(2.5 * e) * std::cos(2.5 * n);
        const double crown = 12.0 - 0.1 * r * r - 0.8 * leaves * leaves;
        double h = r < 7 ? crown : 0.0;
        if (std::abs(e + 25) <= 6 && std::abs(n + 25) <= 6) h = 15.0;
        if (std::abs(e + 25) <= 10 && std::abs(n - 25) <= 10) h = 8.0;
        return std::hypot(e, n + 30) < 0.8 ? 8.0 : h;
    });
    osm::Footprint house;
    house.height = 15.0;
    house.outer = {{latlon(-31, -31), latlon(-19, -31), latlon(-19, -19), latlon(-31, -19)}};
    const auto sw = latlon(-50, -50);
    const auto ne = latlon(50, 50);
    const auto mesh =
        trees(std::span(&house, 1), origin, 100.0, [](double, double) { return 105.0; }, grid,
              {.south = sw.lat, .west = sw.lon, .north = ne.lat, .east = ne.lon});
    REQUIRE(mesh.triangles() > 0);
    float top = -1e9F;
    float bottom = 1e9F;
    for (const auto& v : mesh.vertices) {
        top = std::max(top, v[2]);
        bottom = std::min(bottom, v[2]);
    }
    CHECK(top == Approx(5.0 + 12.0).margin(1.5));
    // The sides end at the crown base, above the ground.
    CHECK(bottom > 5.0 + 3.0 * tree_crown_share - 0.1);
    bool sides = false;
    for (std::size_t t = 0; t < mesh.triangles(); ++t) {
        const auto& r = mesh.records[t];
        CHECK(r[record_kind] == kind_terrain);
        double cx = 0;
        double cy = 0;
        for (const auto i : mesh.indices[t]) {
            cx += mesh.vertices[i][0] / 3.0;
            cy += mesh.vertices[i][1] / 3.0;
        }
        CHECK(std::hypot(cx - 25, cy - 25) < 8);
        const float up = r[record_normal + 2];
        if (up < 0.7F) {
            sides = true;
            // Sides face away from the crown centre.
            CHECK((cx - 25) * r[record_normal] + (cy - 25) * r[record_normal + 1] > 0);
        } else {
            CHECK(up > 0.0F);
        }
    }
    CHECK(sides);
}
