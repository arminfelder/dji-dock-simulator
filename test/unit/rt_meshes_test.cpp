#include "osm/facades.hpp"
#include "raster/als.hpp"
#include "render/rt/meshes.hpp"
#include "render/rt/pieces.hpp"
#include "render/rt/slots.hpp"
#include "render/shading.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <cmath>
#include <set>

using namespace docksim;
using namespace docksim::render::rt;
using Catch::Approx;

namespace {

const raster::LocalFrame origin{.lat0 = 48.2, .lon0 = 16.37};

osm::LatLon latlon(double east, double north) {
    return {.lat = origin.lat0 + north / origin.m_per_deg_lat(),
            .lon = origin.lon0 + east / origin.m_per_deg_lon()};
}

osm::Ring rect(double e0, double n0, double e1, double n1) {
    return {latlon(e0, n0), latlon(e1, n0), latlon(e1, n1), latlon(e0, n1)};
}

double flat_ground(double /*unused*/, double /*unused*/) {
    return 105.0;
}

osm::Footprint footprint(double height, osm::Ring outer, std::vector<osm::Ring> inner = {}) {
    osm::Footprint f;
    f.height = height;
    f.outer = {std::move(outer)};
    f.inner = std::move(inner);
    return f;
}

double plan_area(const Mesh& mesh, float kind) {
    double area = 0;
    for (std::size_t t = 0; t < mesh.indices.size(); ++t) {
        if (mesh.records[t][record_kind] != kind) continue;
        const auto& a = mesh.vertices[mesh.indices[t][0]];
        const auto& b = mesh.vertices[mesh.indices[t][1]];
        const auto& c = mesh.vertices[mesh.indices[t][2]];
        area += 0.5 * std::abs((double{b[0]} - a[0]) * (double{c[1]} - a[1]) -
                               (double{b[1]} - a[1]) * (double{c[0]} - a[0]));
    }
    return area;
}

float min_z(const Mesh& m) {
    return std::ranges::min(m.vertices, {}, [](const Vertex& v) { return v[2]; })[2];
}

float max_z(const Mesh& m) {
    return std::ranges::max(m.vertices, {}, [](const Vertex& v) { return v[2]; })[2];
}

std::shared_ptr<render::geodata::Mosaic> dem_tile(int zoom, int tx, int ty, auto height) {
    auto m = std::make_shared<render::geodata::Mosaic>();
    m->zoom = zoom;
    m->x0 = std::int64_t{tx} * 256 - 2;
    m->y0 = std::int64_t{ty} * 256 - 2;
    m->values = raster::Grid2D<float>(260, 260);
    m->valid = raster::Grid2D<std::uint8_t>(260, 260, 1);
    for (int y = 0; y < 260; ++y) {
        for (int x = 0; x < 260; ++x) m->values(x, y) = height(x, y);
    }
    return m;
}

}  // namespace

TEST_CASE("a building with a courtyard has outward walls and a holed roof") {
    const auto item = footprint(20.0, rect(-10, -10, 10, 10), {rect(-2, -2, 2, 2)});
    const auto mesh = buildings(std::span(&item, 1), origin, 100.0, flat_ground, true);
    int walls = 0;
    for (std::size_t t = 0; t < mesh.triangles(); ++t) {
        const auto& r = mesh.records[t];
        if (r[record_kind] != kind_wall) continue;
        ++walls;
        // Wall normals point away from the building: out of the outline, into the courtyard.
        double cx = 0;
        double cy = 0;
        for (const auto v : mesh.indices[t]) {
            cx += mesh.vertices[v][0] / 3.0;
            cy += mesh.vertices[v][1] / 3.0;
        }
        const bool outer = std::max(std::abs(cx), std::abs(cy)) > 5;
        const double dot = cx * r[record_normal] + cy * r[record_normal + 1];
        CHECK((outer ? dot > 0 : dot < 0));
        CHECK(r[record_base] == Approx(5.0));
        CHECK(r[record_top] == Approx(25.0));
    }
    CHECK(walls == 16);
    CHECK(plan_area(mesh, kind_roof) == Approx(400 - 16).margin(1e-3));
    CHECK(min_z(mesh) == Approx(5.0 - bury_m));
}

TEST_CASE("a terrain tile faces up and has skirts") {
    const auto dem = dem_tile(15, 17909, 11360, [](int, int) { return 300.0F; });
    const auto mesh = terrain_tile(*dem, 15, 17909, 11360, 16, origin, 100.0);
    CHECK(mesh.triangles() == 2 * 16 * 16 + 2 * 4 * 16);
    for (std::size_t t = 0; t < 2 * 16 * 16; ++t)
        CHECK(mesh.records[t][record_normal + 2] > 0.999F);
    CHECK(max_z(mesh) == Approx(200.0));
    CHECK(min_z(mesh) < 200.0F);
}

TEST_CASE("terrain vertex normals follow the slope") {
    // Rises to the east.
    const auto dem = dem_tile(15, 17909, 11360,
                              [](int x, int) { return 300.0F + 0.5F * static_cast<float>(x); });
    const auto mesh = terrain_tile(*dem, 15, 17909, 11360, 16, origin, 100.0);
    for (std::size_t t = 0; t < 2 * 16 * 16; ++t) {
        const auto& r = mesh.records[t];
        // A plane: every vertex normal equals the face normal, tilted west (down-slope).
        CHECK(r[record_normal] < -0.01F);
        for (std::size_t k = 0; k < 3; ++k) {
            const auto [x, y] = unpack_normal(r[record_vertex_normals + k]);
            CHECK(x == Approx(r[record_normal]).margin(2e-3));
            CHECK(y == Approx(r[record_normal + 1]).margin(2e-3));
        }
    }
}

TEST_CASE("roof meshes follow the roof shape and close the walls") {
    const auto shape =
        GENERATE(osm::RoofShape::gabled, osm::RoofShape::hipped, osm::RoofShape::pyramidal,
                 osm::RoofShape::skillion, osm::RoofShape::dome);
    const osm::Roof roof{.shape = shape, .height = 6.0};
    auto item = footprint(16.0, rect(-15, -5, 15, 5));
    item.roof = roof;
    const auto mesh = buildings(std::span(&item, 1), origin, 100.0, flat_ground, true);
    // The roof covers the footprint once: its plan area is the rectangle.
    CHECK(plan_area(mesh, kind_roof) == Approx(300.0).epsilon(1e-4));
    const std::vector<osm::Point> corners{{-15, -5}, {15, -5}, {15, 5}, {-15, 5}};
    const auto box = osm::min_area_box(corners);
    const auto expected = [&](const Vertex& v) {
        return 5.0 + osm::roof_height(roof, box, v[0], v[1], 16.0, std::nullopt);
    };
    double roof_err = 0;
    double wall_err = 0;
    for (std::size_t t = 0; t < mesh.triangles(); ++t) {
        for (const auto i : mesh.indices[t]) {
            const auto& v = mesh.vertices[i];
            // Every roof vertex lies on the analytic roof; wall tops meet the roof edge.
            if (mesh.records[t][record_kind] == kind_roof)
                roof_err = std::max(roof_err, std::abs(v[2] - expected(v)));
            if (mesh.records[t][record_kind] == kind_wall && v[2] > 4.0F)
                wall_err = std::max(wall_err, std::abs(v[2] - expected(v)));
        }
    }
    CHECK(roof_err < 0.02);
    CHECK(wall_err < 0.02);
    CHECK(max_z(mesh) == Approx(21.0).margin(0.05));
}

TEST_CASE("roof height at the dock") {
    const auto tower = footprint(40.0, rect(-10, -10, 10, 10));
    // Ground 105, origin z0 100: roof at 5 + 40.
    CHECK(roof_height_at(std::span(&tower, 1), origin, 100.0, flat_ground, origin.lat0,
                         origin.lon0) == Approx(45.0));
    const auto beside = latlon(30, 0);
    CHECK(roof_height_at(std::span(&tower, 1), origin, 100.0, flat_ground, beside.lat,
                         beside.lon) == 0.0);
}

TEST_CASE("walls carry the storey height of the tagged levels") {
    auto item = footprint(16.0, rect(-5, -5, 5, 5));
    const auto storeys = [&] {
        std::set<float> out;
        const auto mesh = buildings(std::span(&item, 1), origin, 100.0, flat_ground, true);
        for (const auto& r : mesh.records) {
            if (r[record_kind] == kind_wall) out.insert(r[record_storey]);
        }
        return out;
    };
    CHECK(storeys() == std::set<float>{0.0F});
    item.levels = 5;
    CHECK(storeys() == std::set<float>{3.2F});
    // 16 m in 2 levels is no plausible storey: the window layout stays.
    item.levels = 2;
    CHECK(storeys() == std::set<float>{0.0F});
}

TEST_CASE("an entrance makes a door span and a shop makes a shopfront wall") {
    using Kind = osm::Feature::Kind;
    const auto item = footprint(12.0, rect(-10, -5, 10, 5));
    // Entrance on the south wall at east 3; shop inside, nearest to the south wall too.
    const std::vector<osm::Feature> features{{.kind = Kind::entrance, .at = latlon(3, -5)},
                                             {.kind = Kind::shop, .at = latlon(-4, -3)}};
    const auto mesh =
        buildings(std::span(&item, 1), origin, 100.0, flat_ground, true, nullptr, features);
    std::set<std::pair<float, float>> doors;
    std::set<std::pair<float, float>> shops;
    int south_plain = 0;
    for (const auto& r : mesh.records) {
        if (r[record_kind] != kind_wall) continue;
        const bool south = r[record_normal + 1] < -0.99F;
        if (r[record_feature] == render::shading::feature_door) {
            CHECK(south);
            doors.insert({r[record_feature_from], r[record_feature_to]});
        } else if (r[record_feature] == render::shading::feature_shop) {
            CHECK(south);
            shops.insert({r[record_feature_from], r[record_feature_to]});
        } else if (south) {
            ++south_plain;
        }
    }
    REQUIRE(doors.size() == 1);
    CHECK(doors.begin()->second - doors.begin()->first ==
          Approx(render::shading::door_width_m).margin(1e-4));
    // The shop takes the whole 20 m wall, the door cuts it in two.
    REQUIRE(shops.size() == 1);
    CHECK(shops.begin()->second - shops.begin()->first == Approx(20.0).margin(1e-3));
    CHECK(doors.begin()->first > shops.begin()->first);
    CHECK(doors.begin()->second < shops.begin()->second);
    CHECK(south_plain == 0);
}

TEST_CASE("features away from a building leave its walls plain") {
    const auto item = footprint(12.0, rect(-10, -5, 10, 5));
    const std::vector<osm::Feature> features{
        {.kind = osm::Feature::Kind::entrance, .at = latlon(3, -8)},
        {.kind = osm::Feature::Kind::shop, .at = latlon(30, 0)}};
    const auto mesh =
        buildings(std::span(&item, 1), origin, 100.0, flat_ground, true, nullptr, features);
    CHECK(std::ranges::none_of(mesh.records, [](const Record& r) {
        return r[record_kind] == kind_wall && r[record_feature] != 0.0F;
    }));
}

TEST_CASE("balconies sit on the floor lines of some walls, from the first floor up") {
    const int punched = osm::styles().id(osm::Facade{.windows = osm::Windows::punched});
    const auto block = [&](std::int64_t id, double height) {
        auto item = footprint(height, rect(-15, -6, 15, 6));
        item.id = id;
        item.style = punched;
        return item;
    };
    // 25 % of walls get balconies: some of 40 buildings with 4 walls must have them.
    std::size_t details = 0;
    for (std::int64_t id = 1; id <= 40; ++id) {
        const auto item = block(id, 15.0);
        const auto mesh =
            buildings(std::span(&item, 1), origin, 100.0, flat_ground, true, nullptr, {}, true);
        for (std::size_t t = 0; t < mesh.triangles(); ++t) {
            if (mesh.records[t][record_kind] != kind_detail) continue;
            ++details;
            for (const auto v : mesh.indices[t]) {
                // Slab and parapet heights above the base at 5 m: floor lines at 3 m steps.
                const double z = mesh.vertices[v][2] - 5.0;
                const double floor_z = std::round(z / 3.0) * 3.0;
                const bool on_floor = std::abs(z - floor_z) < 1e-3 ||
                                      std::abs(z - (floor_z - balcony_slab_m)) < 1e-3 ||
                                      std::abs(z - (floor_z + balcony_parapet_m)) < 1e-3;
                CHECK(on_floor);
                CHECK(z > 3.0 - balcony_slab_m - 1e-3);
            }
        }
        // The same building always gets the same balconies.
        CHECK(buildings(std::span(&item, 1), origin, 100.0, flat_ground, true, nullptr, {}, true)
                  .records == mesh.records);
    }
    CHECK(details > 0);
    CHECK(details % 12 == 0);
    // Two floors are too few; far cells (no balconies asked for) have none either.
    for (std::int64_t id = 1; id <= 40; ++id) {
        const auto low = block(id, 7.0);
        const auto tall = block(id, 15.0);
        const auto none = [](const Mesh& m) {
            return std::ranges::none_of(
                m.records, [](const Record& r) { return r[record_kind] == kind_detail; });
        };
        CHECK(none(
            buildings(std::span(&low, 1), origin, 100.0, flat_ground, true, nullptr, {}, true)));
        CHECK(none(buildings(std::span(&tall, 1), origin, 100.0, flat_ground, true)));
    }
}

TEST_CASE("a raised part starts at its min height") {
    auto slab = footprint(45.0, rect(-5, -5, 5, 5));
    slab.bottom = 44.0;
    const auto mesh = buildings(std::span(&slab, 1), origin, 100.0, flat_ground, true);
    CHECK(min_z(mesh) == Approx(5.0 + 44.0));
    CHECK(max_z(mesh) == Approx(5.0 + 45.0));
    CHECK(std::ranges::any_of(mesh.records,
                              [](const Record& r) { return r[record_normal + 2] < -0.99F; }));
}

TEST_CASE("curvature lowers far vertices and the facade heights with them") {
    auto item = footprint(10.0, rect(2000, 0, 2010, 10));
    auto mesh = buildings(std::span(&item, 1), origin, 100.0, flat_ground, true);
    const auto before = mesh;
    curve(mesh, origin);
    const double drop = 2000.0 * 2000.0 / (2 * 6371008.8);
    CHECK(mesh.vertices[0][2] == Approx(before.vertices[0][2] - drop).margin(0.02));
    for (std::size_t t = 0; t < mesh.triangles(); ++t) {
        CHECK(mesh.records[t][record_top] < before.records[t][record_top] - 0.25F);
    }
}

TEST_CASE("float16 normal packing round-trips") {
    const auto [x, y] = unpack_normal(pack_normal(-0.25, 0.7071));
    CHECK(x == Approx(-0.25).margin(1e-3));
    CHECK(y == Approx(0.7071).margin(1e-3));
}

TEST_CASE("piece keys order and compare") {
    const std::set<PieceKey> keys{
        TileKey{.z = 15, .tx = 1, .ty = 2}, CellKey{.cy = 4726, .cx = 1140, .near = true},
        TreesKey{.cy = 4726, .cx = 1140}, TileKey{.z = 15, .tx = 1, .ty = 2}};
    CHECK(keys.size() == 3);
}

TEST_CASE("slots reuse freed space and merge neighbours") {
    Slots slots(100);
    CHECK(slots.alloc(30) == 0U);
    CHECK(slots.alloc(30) == 30U);
    CHECK(slots.alloc(30) == 60U);
    CHECK_FALSE(slots.alloc(20));
    slots.release(30, 30);
    slots.release(0, 30);
    CHECK(slots.free().front() == std::pair<std::size_t, std::size_t>{0, 60});
    CHECK(slots.alloc(50) == 0U);
}

TEST_CASE("slots grow at the end") {
    Slots slots(10);
    CHECK(slots.alloc(10) == 0U);
    slots.grow(25);
    CHECK(slots.alloc(15) == 10U);
    CHECK(slots.capacity() == 25U);
}

#ifdef __STDCPP_FLOAT16_T__
#include <random>
#include <stdfloat>

// The packed normals use a hand-written binary16 conversion (Clang has no
// std::float16_t). Where the compiler has one, both must agree bit for bit.
TEST_CASE("normal packing matches std::float16_t") {
    std::mt19937 rng(1);
    std::uniform_int_distribution<std::uint32_t> any;
    std::uniform_real_distribution<float> unit(-1.5F, 1.5F);
    int mismatches = 0;
    for (int i = 0; i < 400000; ++i) {
        const float x = i % 2 == 0 ? std::bit_cast<float>(any(rng)) : unit(rng);
        if (x != x) continue;
        const auto ref = std::bit_cast<std::uint16_t>(static_cast<std::float16_t>(x));
        const auto packed = std::bit_cast<std::uint32_t>(docksim::render::rt::pack_normal(x, 0.0));
        if ((packed & 0xFFFFU) != ref) ++mismatches;
        const float back =
            docksim::render::rt::unpack_normal(std::bit_cast<float>(std::uint32_t{ref})).first;
        const float want = static_cast<float>(std::bit_cast<std::float16_t>(ref));
        if (back != want && !(back != back && want != want)) ++mismatches;
    }
    CHECK(mismatches == 0);
}
#endif
