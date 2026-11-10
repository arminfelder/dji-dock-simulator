#include "osm/cells.hpp"
#include "osm/facades.hpp"
#include "osm/features.hpp"
#include "osm/footprints.hpp"
#include "osm/grid.hpp"
#include "osm/overpass.hpp"
#include "osm/provider.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <format>
#include <fstream>
#include <random>
#include <set>
#include <sstream>
#include <stop_token>
#include <thread>
#include <tuple>

using namespace docksim;
using namespace docksim::osm;
using Catch::Approx;

namespace {

const Json way = {
    {"type", "way"},
    {"id", 7},
    {"tags", {{"building", "yes"}, {"building:levels", "2"}}},
    {"geometry", Json::array({{{"lat", 47.0}, {"lon", 11.0}}, {{"lat", 47.0001}, {"lon", 11.0}}})},
};

struct Scripted {
    std::deque<std::optional<PostReply>> replies;
    std::vector<std::string> urls;
    double now = 0;
    std::vector<double> slept;
};

OverpassClient client(Scripted& s, std::vector<std::string> urls = {"a", "b"}) {
    return {std::move(urls),
            [&s](const std::string& url, const std::string& query, std::string& problem,
                 const std::stop_token& /*stop*/) -> std::optional<PostReply> {
                CHECK(query.contains("building:part"));
                s.urls.push_back(url);
                auto reply = s.replies.front();
                s.replies.pop_front();
                if (!reply) problem = "ConnectionError";
                return reply;
            },
            [&s](double seconds, const std::stop_token& /*stop*/) {
                s.slept.push_back(seconds);
                s.now += seconds;
            },
            [&s] { return s.now; }};
}

PostReply ok(const Json& body) {
    return {.status = 200, .body = body.dump(), .headers = {}};
}

PostReply status(long code) {
    return {.status = code, .body = "", .headers = {}};
}

class TempDir {
public:
    TempDir()
        : path_(std::filesystem::temp_directory_path() /
                std::format("docksim-osm-{}", std::random_device{}())) {
        std::filesystem::create_directories(path_);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    TempDir(TempDir&&) = delete;
    TempDir& operator=(TempDir&&) = delete;
    ~TempDir() { std::filesystem::remove_all(path_); }
    [[nodiscard]] const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

class FakeSource final : public BuildingSource {
public:
    explicit FakeSource(std::optional<std::vector<Json>> raw) : raw_(std::move(raw)) {}
    std::optional<std::vector<Json>> buildings(double /*south*/, double /*west*/, double /*north*/,
                                               double /*east*/,
                                               const std::stop_token& /*stop*/) override {
        ++calls;
        return raw_;
    }
    int calls = 0;

private:
    std::optional<std::vector<Json>> raw_;
};

Json read_json(const std::filesystem::path& path) {
    std::ifstream in(path);
    return Json::parse(in);
}

std::vector<Json> one_way() {
    return {simplify(way)};
}

Ring square_ring(double lat, double lon, double d) {
    return {{lat - d, lon - d}, {lat - d, lon + d}, {lat + d, lon + d}, {lat + d, lon - d}};
}

}  // namespace

TEST_CASE("overpass: success simplifies the elements") {
    Scripted s;
    s.replies.emplace_back(ok({{"elements", Json::array({way})}}));
    auto c = client(s);
    const auto result = c.buildings(47, 11, 47.01, 11.01, {});
    REQUIRE(result);
    REQUIRE(result->size() == 1);
    CHECK((*result)[0] == simplify(way));
    CHECK((*result)[0]["outer"][0][1] == Json::array({47.0001, 11.0}));
}

TEST_CASE("overpass: a busy server backs off, then the next mirror is tried") {
    Scripted s;
    for (int i = 0; i < attempts; ++i) s.replies.emplace_back(status(504));
    s.replies.emplace_back(ok({{"elements", Json::array()}}));
    auto c = client(s);
    CHECK(c.buildings(47, 11, 47.01, 11.01, {}) == std::vector<Json>{});
    CHECK(s.urls == std::vector<std::string>{"a", "a", "a", "b"});
    std::vector<double> backoffs;
    for (const auto d : s.slept) {
        if (d >= busy_backoff_s) backoffs.push_back(d);
    }
    // No wait after the last attempt: the next mirror is asked at once.
    CHECK(backoffs == std::vector<double>{busy_backoff_s, 2 * busy_backoff_s});
}

TEST_CASE("overpass: requests keep the rate gap") {
    Scripted s;
    s.replies.emplace_back(status(403));
    s.replies.emplace_back(ok({{"elements", Json::array()}}));
    auto c = client(s);
    std::ignore = c.buildings(47, 11, 47.01, 11.01, {});
    REQUIRE(s.slept.size() == 1);
    CHECK(s.slept[0] == Approx(request_gap_s));
}

TEST_CASE("overpass: no server answers") {
    Scripted s;
    s.replies.emplace_back(status(403));
    s.replies.emplace_back(PostReply{.status = 200, .body = "not json", .headers = {}});
    auto c = client(s);
    CHECK_FALSE(c.buildings(47, 11, 47.01, 11.01, {}));
}

TEST_CASE("overpass: a transport error moves to the next mirror") {
    Scripted s;
    s.replies.emplace_back(std::nullopt);
    s.replies.emplace_back(ok({{"elements", Json::array()}}));
    auto c = client(s);
    CHECK(c.buildings(47, 11, 47.01, 11.01, {}));
    CHECK(s.urls == std::vector<std::string>{"a", "b"});
}

TEST_CASE("overpass: transient 5xx is retried on the same mirror") {
    Scripted s;
    s.replies.emplace_back(status(502));
    s.replies.emplace_back(ok({{"elements", Json::array()}}));
    auto c = client(s);
    CHECK(c.buildings(47, 11, 47.01, 11.01, {}) == std::vector<Json>{});
    CHECK(s.urls == std::vector<std::string>{"a", "a"});
}

TEST_CASE("overpass: a 429 waits for Retry-After") {
    Scripted s;
    s.replies.emplace_back(
        PostReply{.status = 429, .body = "", .headers = {{"retry-after", "30"}}});
    s.replies.emplace_back(ok({{"elements", Json::array()}}));
    auto c = client(s);
    CHECK(c.buildings(47, 11, 47.01, 11.01, {}));
    CHECK(std::ranges::contains(s.slept, 30.0));
}

TEST_CASE("overpass: failed mirrors cool down, then all are tried again") {
    Scripted s;
    s.replies.emplace_back(status(403));
    s.replies.emplace_back(status(403));
    s.replies.emplace_back(ok({{"elements", Json::array()}}));
    auto c = client(s);
    CHECK_FALSE(c.buildings(47, 11, 47.01, 11.01, {}));
    CHECK(c.buildings(47, 11, 47.01, 11.01, {}) == std::vector<Json>{});
}

TEST_CASE("overpass: the mirror that answered last is asked first") {
    Scripted s;
    s.replies.emplace_back(std::nullopt);
    s.replies.emplace_back(ok({{"elements", Json::array()}}));
    s.replies.emplace_back(ok({{"elements", Json::array()}}));
    auto c = client(s, {"a", "b", "c"});
    CHECK(c.buildings(47, 11, 47.01, 11.01, {}));
    // After the cooldown, a is up again, but b answered last.
    s.now += mirror_cooldown_s + 1;
    CHECK(c.buildings(47, 11, 47.01, 11.01, {}));
    CHECK(s.urls == std::vector<std::string>{"a", "b", "b"});
}

TEST_CASE("overpass: a stop request ends the backoff at once") {
    std::stop_source stop;
    OverpassClient c(
        {"a"},
        [](const std::string&, const std::string&, std::string&,
           const std::stop_token&) -> std::optional<PostReply> { return status(502); });
    const std::jthread stopper([&stop] {
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
        stop.request_stop();
    });
    const auto started = std::chrono::steady_clock::now();
    CHECK_THROWS_AS(c.buildings(47, 11, 47.01, 11.01, stop.get_token()), Stopped);
    CHECK(std::chrono::steady_clock::now() - started < std::chrono::seconds{2});
}

TEST_CASE("cells: a stopped fetch is not a failed cell") {
    struct StopSource final : BuildingSource {
        std::optional<std::vector<Json>> buildings(double /*south*/, double /*west*/,
                                                   double /*north*/, double /*east*/,
                                                   const std::stop_token& stop) override {
            ++calls;
            if (stop.stop_requested()) throw Stopped();
            return one_way();
        }
        int calls = 0;
    };
    const TempDir dir;
    auto source = std::make_shared<StopSource>();
    CellStore store(dir.path(), source, {});
    const std::stop_source stop;
    stop.request_stop();
    CHECK_THROWS_AS(store.footprints(4700, 1100, stop.get_token()), Stopped);
    CHECK(store.footprints(4700, 1100)->size() == 1);
    CHECK(source->calls == 2);
}

TEST_CASE("cells: callers that ask for the same cell at once share one fetch") {
    struct SlowSource final : BuildingSource {
        std::optional<std::vector<Json>> buildings(double /*south*/, double /*west*/,
                                                   double /*north*/, double /*east*/,
                                                   const std::stop_token& /*stop*/) override {
            ++calls;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return one_way();
        }
        std::atomic<int> calls = 0;
    };
    const TempDir dir;
    auto source = std::make_shared<SlowSource>();
    CellStore store(dir.path(), source, {});
    std::vector<Cell> cells(4);
    {
        std::vector<std::jthread> callers;
        for (auto& cell : cells)
            callers.emplace_back([&store, &cell] { cell = store.footprints(4700, 1100); });
    }
    CHECK(source->calls == 1);
    for (const auto& cell : cells) CHECK(cell == cells.front());
}

TEST_CASE("overpass: Retry-After") {
    const auto now = std::chrono::sys_days{std::chrono::year{2026} / 10 / 21} +
                     std::chrono::hours{7} + std::chrono::minutes{28};
    CHECK(retry_after(std::nullopt, now) == retry_after_default_s);
    CHECK(retry_after("30", now) == Approx(30.0));
    CHECK(retry_after("Wed, 21 Oct 2026 07:28:30 GMT", now) == Approx(30.0));
    CHECK(retry_after("garbage", now) == retry_after_default_s);
}

TEST_CASE("overpass: the query has the box with four decimals") {
    const auto q = format_query(47, 11, 47.01, 11.01);
    CHECK(q.contains(R"(way["building"](47.0000,11.0000,47.0100,11.0100);)"));
    CHECK(q.contains(R"(relation["building:part"](47.0000,11.0000,47.0100,11.0100);)"));
    CHECK(q.ends_with("out geom tags;"));
}

TEST_CASE("features: entrances and shops from Overpass nodes") {
    const auto entrance =
        simplify_feature(Json::parse(R"({"type": "node", "id": 1, "lat": 48.1, "lon": 16.2,
                                                           "tags": {"entrance": "main", "shop": "bakery"}})"));
    REQUIRE(entrance);
    CHECK(feature_from_cache(*entrance) ==
          Feature{.kind = Feature::Kind::entrance, .at = {.lat = 48.1, .lon = 16.2}});
    const auto shop = simplify_feature(Json::parse(R"({"type": "node", "lat": 48.1, "lon": 16.2,
                                                       "tags": {"amenity": "cafe"}})"));
    REQUIRE(shop);
    CHECK(feature_from_cache(*shop)->kind == Feature::Kind::shop);
    CHECK_FALSE(simplify_feature(Json::parse(R"({"type": "way", "id": 2})")));
    CHECK_FALSE(feature_from_cache(Json::parse(R"({"kind": "bench", "lat": 1, "lon": 2})")));
    CHECK(format_query(features_query, 48, 16, 48.01, 16.01)
              .contains(R"(node["entrance"]["entrance"!="no"](48.0000,16.0000,48.0100,16.0100))"));
}

TEST_CASE("cells: features are fetched once, cached, and retried after a failure") {
    struct Features final : BuildingSource {
        std::optional<std::vector<Json>> buildings(double /*south*/, double /*west*/,
                                                   double /*north*/, double /*east*/,
                                                   const std::stop_token& /*stop*/) override {
            return std::vector<Json>{};
        }
        std::optional<std::vector<Json>> features(double /*unused*/, double /*unused*/,
                                                  double /*unused*/, double /*unused*/,
                                                  const std::stop_token& /*unused*/) override {
            ++calls;
            if (fail) return std::nullopt;
            return std::vector<Json>{Json{{"kind", "shop"}, {"lat", 47.005}, {"lon", 11.005}}};
        }
        int calls = 0;
        bool fail = false;
    };
    const TempDir dir;
    auto failing = std::make_shared<Features>();
    failing->fail = true;
    CHECK(CellStore(dir.path(), failing, {}).features(4700, 1100)->empty());
    CHECK_FALSE(std::filesystem::exists(dir.path() / "osm-features" / "4700_1100.json"));
    auto source = std::make_shared<Features>();
    CellStore store(dir.path(), source, {});
    CHECK(store.features(4700, 1100)->size() == 1);
    CHECK(store.features(4700, 1100)->size() == 1);
    CHECK(source->calls == 1);
    // A second store reads the disk cache.
    auto unused = std::make_shared<Features>();
    CHECK(CellStore(dir.path(), unused, {}).features(4700, 1100)->size() == 1);
    CHECK(unused->calls == 0);
}

TEST_CASE("overpass: one area request gives buildings and features") {
    Scripted s;
    const Json shop = {{"type", "node"}, {"id", 9}, {"lat", 47.005}, {"lon", 11.005},
                       {"tags", {{"shop", "bakery"}}}};
    s.replies.emplace_back(ok({{"elements", Json::array({way, shop})}}));
    auto c = client(s);
    const auto result = c.area(47, 11, 47.01, 11.01, {});
    REQUIRE(result);
    CHECK(s.urls.size() == 1);
    REQUIRE(result->buildings.size() == 1);
    CHECK(result->buildings[0] == simplify(way));
    REQUIRE(result->features);
    REQUIRE(result->features->size() == 1);
    CHECK((*result->features)[0]["kind"] == "shop");
    const auto query = area_query();
    CHECK(query.starts_with(overpass_query));
    CHECK(query.contains(R"(node["shop"]({s},{w},{n},{e}))"));
    CHECK(query.ends_with(");out;"));
}

namespace {

// A square building of side 2 d degrees around lat, lon in cache form.
Json square(std::int64_t id, double lat, double lon, double d) {
    Json ring = Json::array();
    for (const auto& [a, b] : square_ring(lat, lon, d)) ring.push_back(Json::array({a, b}));
    return {{"id", id}, {"tags", {{"building", "yes"}}}, {"outer", {ring}}, {"inner", Json::array()}};
}

struct Box {
    double south, west, north, east;
};

// Answers area() with the buildings and features inside the box. Boxes that
// hold a cell in broken fail.
struct AreaSource final : BuildingSource {
    std::vector<Json> all_buildings;
    std::vector<Json> all_features;
    std::set<CellKey> broken;
    std::vector<Box> boxes;
    std::optional<std::vector<Json>> buildings(double /*south*/, double /*west*/, double /*north*/,
                                               double /*east*/,
                                               const std::stop_token& /*stop*/) override {
        ++single_calls;
        return std::vector<Json>{};
    }
    std::optional<std::vector<Json>> features(double /*south*/, double /*west*/, double /*north*/,
                                              double /*east*/,
                                              const std::stop_token& /*stop*/) override {
        ++single_calls;
        return std::vector<Json>{};
    }
    std::optional<Area> area(double south, double west, double north, double east,
                             const std::stop_token& /*stop*/) override {
        boxes.push_back({south, west, north, east});
        for (const auto& [cy, cx] : broken) {
            const double lat = (cy + 0.5) * cell_deg;
            const double lon = (cx + 0.5) * cell_deg;
            if (lat > south && lat < north && lon > west && lon < east) return std::nullopt;
        }
        Area out{.buildings = {}, .features = std::vector<Json>{}};
        for (const auto& b : all_buildings) {
            const auto& p = b["outer"][0][0];
            const double lat = p[0].get<double>();
            const double lon = p[1].get<double>();
            if (lat < north && lon < east && lat + 0.01 > south && lon + 0.01 > west)
                out.buildings.push_back(b);
        }
        for (const auto& f : all_features) {
            const double lat = f["lat"].get<double>();
            const double lon = f["lon"].get<double>();
            if (lat >= south && lat < north && lon >= west && lon < east)
                out.features->push_back(f);
        }
        return out;
    }
    int single_calls = 0;
};

}  // namespace

TEST_CASE("cells: prefetch fetches a ring in one request and splits it into cells") {
    const TempDir dir;
    auto source = std::make_shared<AreaSource>();
    source->all_buildings = {square(1, 47.005, 11.005, 0.001),
                             // Across the border of cells 4700_1101 and 4700_1102.
                             square(2, 47.005, 11.02, 0.001), square(3, 47.025, 11.035, 0.001)};
    source->all_features = {Json{{"kind", "shop"}, {"lat", 47.015}, {"lon", 11.015}}};
    CellStore store(dir.path(), source, {});
    store.prefetch(4700, 1100, 4702, 1103);
    REQUIRE(source->boxes.size() == 1);
    CHECK(source->boxes[0].south == Approx(47.00));
    CHECK(source->boxes[0].north == Approx(47.03));
    CHECK(source->boxes[0].east == Approx(11.04));
    for (int cy = 4700; cy <= 4702; ++cy) {
        for (int cx = 1100; cx <= 1103; ++cx) {
            CHECK(std::filesystem::exists(store.path(cy, cx)));
            CHECK(std::filesystem::exists(dir.path() / "osm-features" /
                                          std::format("{}_{}.json", cy, cx)));
        }
    }
    CHECK(store.footprints(4700, 1100)->size() == 1);
    CHECK(store.footprints(4700, 1101)->size() == 1);
    CHECK(store.footprints(4700, 1102)->size() == 1);
    CHECK(store.footprints(4702, 1103)->size() == 1);
    CHECK(store.footprints(4701, 1100)->empty());
    CHECK(store.features(4701, 1101)->size() == 1);
    CHECK(store.features(4700, 1100)->empty());
    CHECK(source->single_calls == 0);
    // Everything is on disk now: no second request.
    store.prefetch(4700, 1100, 4702, 1103);
    CHECK(source->boxes.size() == 1);
}

TEST_CASE("cells: prefetch splits a wide range into blocks") {
    const TempDir dir;
    auto source = std::make_shared<AreaSource>();
    CellStore store(dir.path(), source, {});
    store.prefetch(4700, 1100, 4700, 1100 + fetch_block_cells);
    CHECK(source->boxes.size() == 2);
}

TEST_CASE("cells: a failed block is halved until only the bad cell is missing") {
    const TempDir dir;
    auto source = std::make_shared<AreaSource>();
    source->all_buildings = {square(1, 47.005, 11.005, 0.001)};
    source->broken = {{4701, 1102}};
    CellStore store(dir.path(), source, {});
    store.prefetch(4700, 1100, 4701, 1103);
    CHECK(source->boxes.size() > 1);
    CHECK_FALSE(std::filesystem::exists(store.path(4701, 1102)));
    for (const auto& [cy, cx] : {CellKey{4700, 1100}, CellKey{4700, 1103}, CellKey{4701, 1103},
                                 CellKey{4701, 1101}}) {
        CHECK(std::filesystem::exists(store.path(cy, cx)));
    }
    CHECK(store.footprints(4700, 1100)->size() == 1);
    // The bad cell is marked failed: no per-cell request, no new block.
    const auto boxes = source->boxes.size();
    CHECK(store.footprints(4701, 1102)->empty());
    store.prefetch(4700, 1100, 4701, 1103);
    CHECK(source->boxes.size() == boxes);
    CHECK(source->single_calls == 0);
}

TEST_CASE("cells: cached features never wait; a background fetch fills them") {
    struct Slow final : BuildingSource {
        std::optional<std::vector<Json>> buildings(double /*south*/, double /*west*/,
                                                   double /*north*/, double /*east*/,
                                                   const std::stop_token& /*stop*/) override {
            return std::vector<Json>{};
        }
        std::optional<std::vector<Json>> features(double /*unused*/, double /*unused*/,
                                                  double /*unused*/, double /*unused*/,
                                                  const std::stop_token& /*unused*/) override {
            std::this_thread::sleep_for(std::chrono::milliseconds{50});
            return std::vector<Json>{Json{{"kind", "entrance"}, {"lat", 47.005}, {"lon", 11.005}}};
        }
    };
    const TempDir dir;
    CellStore store(dir.path(), std::make_shared<Slow>(), {});
    CHECK(store.cached_features(4700, 1100) == nullptr);
    FeatureCell found;
    for (int i = 0; i < 100 && !found; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
        found = store.cached_features(4700, 1100);
    }
    REQUIRE(found);
    CHECK(found->size() == 1);
}

TEST_CASE("footprints: heights from tags") {
    const auto [tags, height] = GENERATE(table<Json, double>({
        {Json{{"height", "22 m"}}, 22.0},
        {Json{{"height", "30 ft"}}, 30 * 0.3048},
        {Json{{"building:levels", "4"}}, 4 * 3.0 + 1.0},
        {Json{{"building:levels", "4"}, {"roof:levels", "1"}}, 5 * 3.0 + 1.0},
        {Json{{"building", "carport"}}, roof_only_m},
        {Json{{"building", "yes"}}, 8.0},
    }));
    CHECK(height_from_tags(tags, 8.0, 3.0) == Approx(height));
}

TEST_CASE("footprints: parse_metres with a comma") {
    CHECK(parse_metres("12,5") == 12.5);
    CHECK_FALSE(parse_metres("n/a"));
}

namespace {

Json geometry(std::initializer_list<std::pair<double, double>> points) {
    Json g = Json::array();
    for (const auto& [lat, lon] : points) g.push_back({{"lat", lat}, {"lon", lon}});
    return g;
}

Json member(const char* role, Json geom) {
    return {{"type", "way"}, {"role", role}, {"geometry", std::move(geom)}};
}

Json ring_of(std::initializer_list<std::pair<double, double>> points) {
    Json r = Json::array();
    for (const auto& [lat, lon] : points) r.push_back(Json::array({lat, lon}));
    return r;
}

}  // namespace

TEST_CASE("footprints: a relation gives outer and inner rings") {
    const Json relation = {
        {"type", "relation"},
        {"id", 9},
        {"members",
         Json::array({member("outer", geometry({{0, 0}, {0, 4}, {4, 4}, {4, 0}, {0, 0}})),
                      member("inner", geometry({{1, 1}, {1, 2}, {2, 2}, {1, 1}}))})},
    };
    const auto s = simplify(relation);
    CHECK(s["outer"] == Json::array({ring_of({{0, 0}, {0, 4}, {4, 4}, {4, 0}, {0, 0}})}));
    CHECK(s["inner"] == Json::array({ring_of({{1, 1}, {1, 2}, {2, 2}, {1, 1}})}));
    CHECK(s["tags"] == Json::object());
}

TEST_CASE("footprints: split multipolygon rings are joined, reversed pieces included") {
    // The outer ring in three ways; the middle one runs backwards. The inner ring in two ways.
    const Json relation = {
        {"type", "relation"},
        {"id", 10},
        {"members", Json::array({member("outer", geometry({{0, 0}, {0, 4}})),
                                 member("outer", geometry({{4, 0}, {4, 4}, {0, 4}})),
                                 member("outer", geometry({{4, 0}, {0, 0}})),
                                 member("inner", geometry({{1, 1}, {1, 2}, {2, 2}})),
                                 member("inner", geometry({{2, 2}, {1, 1}}))})},
    };
    const auto s = simplify(relation);
    REQUIRE(s["outer"].size() == 1);
    const auto& outer = s["outer"][0];
    CHECK(outer.size() == 5);
    CHECK(outer.front() == outer.back());
    for (const auto& corner : ring_of({{0, 0}, {0, 4}, {4, 4}, {4, 0}})) {
        CHECK(std::ranges::find(outer, corner) != outer.end());
    }
    REQUIRE(s["inner"].size() == 1);
    CHECK(s["inner"][0] == ring_of({{2, 2}, {1, 1}, {1, 2}, {2, 2}}));
}

TEST_CASE("footprints: a piece that never closes is dropped, closed rings stay") {
    const Json relation = {
        {"type", "relation"},
        {"id", 11},
        {"members", Json::array({member("outer", geometry({{0, 0}, {0, 1}, {1, 1}, {0, 0}})),
                                 member("outer", geometry({{5, 5}, {5, 6}})),
                                 member("outer", geometry({{7, 7}}))})},
    };
    const auto s = simplify(relation);
    CHECK(s["outer"] == Json::array({ring_of({{0, 0}, {0, 1}, {1, 1}, {0, 0}})}));
    CHECK(s["inner"] == Json::array());
}

TEST_CASE("cells: fetched once, then cached on disk in the Python layout") {
    const TempDir dir;
    auto fake = std::make_shared<FakeSource>(one_way());
    CellStore store(dir.path(), fake, {});
    const auto first = store.footprints(4700, 1100);
    CHECK(store.footprints(4700, 1100) == first);
    CHECK(fake->calls == 1);
    CHECK(read_json(dir.path() / osm::building_cache_dir / "4700_1100.json")[0]["id"] == 7);
    REQUIRE(first->size() == 1);
    CHECK((*first)[0].id == 7);
}

TEST_CASE("cells: a corrupt cache file is fetched again") {
    const TempDir dir;
    const auto path = dir.path() / osm::building_cache_dir / "4700_1100.json";
    std::filesystem::create_directories(path.parent_path());
    std::ofstream(path) << "{half a fi";
    auto fake = std::make_shared<FakeSource>(std::vector<Json>{});
    std::ignore = CellStore(dir.path(), fake, {}).footprints(4700, 1100);
    CHECK(fake->calls == 1);
    CHECK(read_json(path) == Json::array());
}

TEST_CASE("cells: a zero override removes a building") {
    const TempDir dir;
    CellStore store(dir.path(), std::make_shared<FakeSource>(one_way()), {.overrides = {{7, 0.0}}});
    CHECK((*store.footprints(4700, 1100))[0].height == 0.0);
}

TEST_CASE("cells: a stale cell is refreshed, and kept when no server answers") {
    const TempDir dir;
    const auto path = dir.path() / osm::building_cache_dir / "4700_1100.json";
    std::filesystem::create_directories(path.parent_path());
    std::ofstream(path) << Json(one_way()).dump();
    std::filesystem::last_write_time(path, std::filesystem::file_time_type::clock::now() -
                                               max_cell_age - std::chrono::seconds{10});

    auto failing = std::make_shared<FakeSource>(std::nullopt);
    const auto kept = CellStore(dir.path(), failing, {}).footprints(4700, 1100);
    CHECK(failing->calls == 1);
    CHECK(kept->size() == 1);

    auto fresh = std::make_shared<FakeSource>(std::vector<Json>{});
    CHECK(CellStore(dir.path(), fresh, {}).footprints(4700, 1100)->empty());
    CHECK(read_json(path) == Json::array());
}

TEST_CASE("cells: a failed cell is not asked for again at once") {
    const TempDir dir;
    auto fake = std::make_shared<FakeSource>(std::nullopt);
    CellStore store(dir.path(), fake, {});
    CHECK(store.footprints(4700, 1100)->empty());
    CHECK(store.footprints(4700, 1100)->empty());
    CHECK(fake->calls == 1);
}

TEST_CASE("cells: the extras add the flak tower pits") {
    const double lat = 48.225819;
    const double lon = 16.373287;
    const auto [cy, cx] = cell_of(lat, lon);
    const Json block = {
        {"id", 263795548},
        {"tags", {{"building:part", "yes"}, {"height", "51"}}},
        {"outer", Json::array({Json::array({Json::array({lat - 2e-4, lon - 3e-4}),
                                            Json::array({lat - 2e-4, lon + 3e-4}),
                                            Json::array({lat + 2e-4, lon})})})},
        {"inner", Json::array()},
    };
    const auto merged = with_extras({block}, cy, cx);
    CHECK(merged[0]["inner"].size() == 4);
    int floors = 0;
    for (const auto& m : merged) {
        if (m["id"].get<std::int64_t>() < 0) {
            ++floors;
            CHECK(m["tags"]["height"] == "48.5");
            CHECK_FALSE(m.contains("requires"));
        }
    }
    CHECK(floors == 4);
    // The new floor parts belong to the tower's cell only.
    CHECK(with_extras({block}, cy + 5, cx).size() == 1);
    // An old cache without the top block: no free-standing pit floors.
    Json other = block;
    other["id"] = 1;
    const auto alone = with_extras({other}, cy, cx);
    REQUIRE(alone.size() == 1);
    CHECK(alone[0] == other);
}

TEST_CASE("grid: the taller footprint wins and an inner ring is empty") {
    const double lat = 47.0;
    const double lon = 11.0;
    const double d = 0.0002;
    const Ring hole{
        {lat - d / 4, lon - d / 4}, {lat - d / 4, lon + d / 4}, {lat + d / 4, lon + d / 4}};
    const std::vector items{
        Footprint{.height = 20.0, .outer = {square_ring(lat, lon, d)}, .inner = {hole}, .style = 2},
        Footprint{.height = 5.0, .outer = {square_ring(lat, lon, d)}, .style = 1}};
    const auto grid = rasterize(items, lat, lon, 50.0, 0.5);
    CHECK(grid.top == 20.0F);
    CHECK(grid.height_at(lat + d / 2, lon) == Approx(20.0));
    CHECK(grid.style_at(lat + d / 2, lon) == 3);
    // The hole: centroid of the triangle.
    CHECK(grid.height_at(lat - d / 12, lon + d / 12) == 0.0F);
    CHECK(grid.style_at(lat - d / 12, lon + d / 12) == 0);
    CHECK(grid.contains(lat, lon, 10.0));
    CHECK_FALSE(grid.contains(lat + 0.001, lon, 0.0));
}

TEST_CASE("grid: wall cells carry the nearest outline edge") {
    const double lat = 47.0;
    const double lon = 11.0;
    const double d = 0.0002;
    const std::vector items{Footprint{.height = 10.0, .outer = {square_ring(lat, lon, d)}}};
    const auto grid = rasterize(items, lat, lon, 50.0, 0.5);
    CHECK(grid.edges.size() == 4);
    const double m_lat = raster::LocalFrame{.lat0 = lat, .lon0 = lon}.m_per_deg_lat();
    // 0.3 m inside the south wall: distance about 0.3 m, normal along north.
    const auto wall = grid.wall_at(lat - d + 0.3 / m_lat, lon);
    CHECK(std::abs(wall.distance) == Approx(0.3).margin(0.02));
    CHECK(std::abs(wall.north) == Approx(1.0).margin(1e-4));
    CHECK(wall.east == Approx(0.0).margin(1e-4));
    // Far inside: no edge.
    CHECK(grid.wall_at(lat, lon).distance == 0.0F);
    const auto packed = grid.packed_at(lat - d + 0.3 / m_lat, lon);
    CHECK((packed >> 8U) >= 1U);
    CHECK((packed & 0xFFU) == 1U);
}

TEST_CASE("grid: the frame map matches pixel()") {
    const std::vector items{Footprint{.height = 10.0, .outer = {square_ring(47.0, 11.0, 0.0002)}}};
    const auto grid = rasterize(items, 47.0, 11.0, 50.0, 0.5);
    const raster::LocalFrame frame{.lat0 = 47.0001, .lon0 = 11.0002};
    const auto m = grid.map(frame);
    for (const auto& [east, north] : {std::pair{0.0, 0.0}, {10.0, -7.0}, {-20.0, 15.0}}) {
        const auto [lat, lon] = frame.to_latlon(east, north);
        const auto [gx, gy] = grid.pixel(lat, lon);
        CHECK(m.gx0 + east * m.bx == Approx(gx).margin(1e-3));
        CHECK(m.gy0 - north * m.by == Approx(gy).margin(1e-3));
    }
}

TEST_CASE("grid: a plain part takes the windows and weathering of its outline") {
    const double lat = 47.0;
    const double lon = 11.0;
    const double d = 0.0002;
    const Ring small{
        {lat - d / 2, lon - d / 2}, {lat - d / 2, lon + d / 2}, {lat + d / 2, lon + d / 2}};
    auto& table = styles();
    const int bunker_style = table.id({.colour = {140, 138, 132},
                                       .windows = Windows::none,
                                       .weathering = 0.9,
                                       .material = "concrete"});
    const int classic = table.id({.colour = {150, 150, 146},
                                  .windows = Windows::classic,
                                  .weathering = 0.0,
                                  .material = "concrete"});
    const std::vector items{
        Footprint{.height = 40.0, .outer = {square_ring(lat, lon, d)}, .style = bunker_style},
        Footprint{.height = 44.0,
                  .outer = {small},
                  .style = classic,
                  .part = true,
                  .plain_part = true,
                  .own_look = true},
    };
    const auto chosen = select_footprints(items, true);
    REQUIRE(chosen.size() == 1);
    const auto look = table.facade(chosen[0].style);
    CHECK(look.windows == Windows::none);
    CHECK(look.weathering == 0.9);
    CHECK(look.colour == Colour{150, 150, 146});
}

TEST_CASE("grid: the near grid wins where it covers") {
    const auto flat = [](int size, float value) {
        auto g = std::make_shared<LocalGrid>();
        g->lat_c = 47.0;
        g->lon_c = 11.0;
        g->step_lat = 1e-5;
        g->step_lon = 1e-5;
        g->resolution_m = 1.0;
        g->height = raster::Grid2D<float>(size, size, value);
        g->style = raster::Grid2D<std::uint32_t>(size, size, 1U);
        g->top = value;
        return std::shared_ptr<const LocalGrid>(std::move(g));
    };
    const Buildings pair{.near = flat(10, 5.0F), .far = flat(100, 20.0F)};
    CHECK(pair.height_at(47.0, 11.0) == 5.0F);
    CHECK(pair.height_at(47.0003, 11.0) == 20.0F);
    CHECK(pair.top() == 20.0F);
}

TEST_CASE("provider: wait builds the near grid; the far ring loads in the background") {
    const TempDir dir;
    const double lat = 47.005;
    const double lon = 11.005;
    Json item = {{"id", 1},
                 {"tags", {{"building", "yes"}, {"height", "15"}}},
                 {"outer", Json::array()},
                 {"inner", Json::array()}};
    Json ring = Json::array();
    for (const auto& p : square_ring(lat, lon, 0.0001)) ring.push_back(Json::array({p.lat, p.lon}));
    item["outer"].push_back(ring);
    auto source = std::make_shared<FakeSource>(std::vector<Json>{item});
    auto cells = std::make_shared<CellStore>(dir.path(), source, FootprintOptions{});
    BuildingProvider provider(cells, 100.0, 300.0, 3.0);
    int ready = 0;
    std::mutex m;
    provider.set_on_ready([&](const std::shared_ptr<const LocalGrid>&) {
        const std::scoped_lock lock(m);
        ++ready;
    });
    const auto first = provider.grid(lat, lon, true);
    REQUIRE(first);
    REQUIRE(first->near);
    CHECK(first->height_at(lat, lon) == Approx(15.0));
    // The far ring: wait until its background load is done.
    std::shared_ptr<const LocalGrid> far;
    for (int i = 0; i < 500 && !far; ++i) {
        far = provider.grid(lat, lon, false)->far;
        if (!far) std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    REQUIRE(far);
    CHECK(far->resolution_m == 3.0);
    CHECK(provider.grid(lat, lon, false)->near == first->near);
    const std::scoped_lock lock(m);
    CHECK(ready == 2);
}

TEST_CASE("cells: prefetch leaves cells alone whose buildings are on disk") {
    const TempDir dir;
    auto source = std::make_shared<AreaSource>();
    CellStore store(dir.path(), source, {});
    std::filesystem::create_directories(store.path(4700, 1100).parent_path());
    std::ofstream(store.path(4700, 1100)) << "[]";
    store.prefetch(4700, 1100, 4700, 1100);
    CHECK(source->boxes.empty());
}
