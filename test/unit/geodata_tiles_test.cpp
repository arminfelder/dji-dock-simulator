#include "image/image.hpp"
#include "raster/mercator.hpp"
#include "render/geodata/imagery.hpp"
#include "render/geodata/terrain.hpp"
#include "render/geodata/tiles.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <stb_image_write.h>

#include <atomic>
#include <cmath>
#include <filesystem>
#include <random>
#include <thread>
#include <tuple>

using namespace docksim;
using namespace docksim::render::geodata;
using Catch::Approx;

namespace {

constexpr int tile = raster::tile_px;
constexpr double elevation_m = 560.25;

Bytes png(const image::RgbImage& img) {
    Bytes out;
    const auto sink = [](void* context, void* data, int size) {
        auto& o = *static_cast<Bytes*>(context);
        const auto* b = static_cast<const std::byte*>(data);
        o.insert(o.end(), b, b + size);
    };
    stbi_write_png_to_func(sink, &out, img.width(), img.height(), 3, img.bytes().data(),
                           img.width() * 3);
    return out;
}

// Parses .../{z}/{x}/{y}.png of a test URL.
std::array<int, 3> zxy(const std::string& url) {
    const auto rest = url.substr(url.find('/', url.find("://") + 3) + 1);
    std::array<int, 3> out{};
    std::size_t at = 0;
    for (auto& v : out) {
        const auto next = rest.find_first_of("/.", at);
        v = std::stoi(rest.substr(at, next - at));
        at = next + 1;
    }
    return out;
}

// Imagery tiles carry x and y in red and green; DEM tiles a constant Terrarium height.
class FakeServer {
public:
    Fetcher fetcher() {
        return [this](const std::string& url) -> FetchResult {
            ++requests;
            const auto [z, x, y] = zxy(url);
            if (y < 0 || missing_row == y) return {.status = 404, .body = {}};
            if (url.starts_with("https://dem.test")) {
                const double value = elevation_m + 32768.0;
                const auto r = static_cast<std::uint8_t>(std::floor(value / 256));
                const auto g = static_cast<std::uint8_t>(std::floor(value - r * 256.0));
                const auto b = static_cast<std::uint8_t>(std::floor((value - r * 256.0 - g) * 256));
                return {.status = 200, .body = png(image::RgbImage(tile, tile, {r, g, b}))};
            }
            return {.status = 200,
                    .body = png(image::RgbImage(
                        tile, tile,
                        {static_cast<std::uint8_t>(x % 256), static_cast<std::uint8_t>(y % 256),
                         static_cast<std::uint8_t>(z)}))};
        };
    }
    std::atomic<int> requests = 0;
    int missing_row = -1;
};

class TempDir {
public:
    TempDir() {
        std::random_device rd;
        path_ = std::filesystem::temp_directory_path() / std::format("docksim-tiles-{:08x}", rd());
        std::filesystem::create_directories(path_);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    TempDir(TempDir&&) = delete;
    TempDir& operator=(TempDir&&) = delete;
    ~TempDir() { std::filesystem::remove_all(path_); }
    [[nodiscard]] std::string str() const { return path_.string(); }
    [[nodiscard]] const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

constexpr int zoom = 17;
const char* imagery_url = "https://imagery.test/{z}/{x}/{y}.png";

Bounds box(double x, double y, double size = 300.0) {
    const double origin = 70'000.0 * tile;
    return {.x0 = origin + x, .y0 = origin + y, .x1 = origin + x + size, .y1 = origin + y + size};
}

void wait_idle(const MosaicCache& cache) {
    cache.wait_idle();
}

}  // namespace

TEST_CASE("a tile shrunk by 4 averages every source pixel, not only the middle two") {
    // Every fourth column is white. Bilinear reads columns 4x+1 and 4x+2 only and sees black.
    image::RgbImage lines(1024, 1024);
    for (int y = 0; y < lines.height(); ++y) {
        for (int x = 0; x < lines.width(); x += 4) lines.set(x, y, {255, 255, 255});
    }
    const auto small = image::resize(lines, 256, 256);
    REQUIRE(small.width() == 256);
    REQUIRE(small.height() == 256);
    for (const auto v : small.bytes()) CHECK((v == 63 || v == 64));
}

TEST_CASE("format_url fills z, x and y") {
    CHECK(format_url("https://a/{z}/{y}/{x}.jpeg", 3, 4, 5) == "https://a/3/5/4.jpeg");
}

TEST_CASE("tiles cache on disk in the layout of the Python simulator") {
    const TempDir dir;
    FakeServer server;
    TileSource source(imagery_url, dir.str(), server.fetcher());
    const auto t = source.get(17, 100, 200);
    REQUIRE(t);
    CHECK(t->pixel(5, 5)[0] == 100);
    CHECK(t->pixel(5, 5)[1] == 200);
    // sha1("https://imagery.test/{z}/{x}/{y}.png")[:12]
    CHECK(source.root().filename().string().size() == 12);
    CHECK(std::filesystem::exists(source.root() / "17" / "100" / "200.tile"));
    std::ignore = source.get(17, 100, 200);
    CHECK(server.requests == 1);

    // A fresh source reads the disk, not the server.
    TileSource again(imagery_url, dir.str(), server.fetcher());
    CHECK(again.get(17, 100, 200)->pixel(0, 0)[0] == 100);
    CHECK(server.requests == 1);
}

TEST_CASE("a missing tile leaves a marker") {
    const TempDir dir;
    FakeServer server;
    server.missing_row = 7;
    TileSource source(imagery_url, dir.str(), server.fetcher());
    CHECK_FALSE(source.get(5, 3, 7));
    CHECK(std::filesystem::exists(source.root() / "5" / "3" / "7.missing"));
    TileSource again(imagery_url, dir.str(), server.fetcher());
    CHECK_FALSE(again.get(5, 3, 7));
    CHECK(server.requests == 1);
}

TEST_CASE("a mosaic stitches tiles and marks missing ones invalid") {
    const TempDir dir;
    FakeServer server;
    server.missing_row = 11;
    TileSource source(imagery_url, dir.str(), server.fetcher());
    const auto m = build_mosaic(source, 5,
                                {.x0 = 10 * tile + 5.0,
                                 .y0 = 10 * tile + 5.0,
                                 .x1 = 11 * tile + 5.0,
                                 .y1 = 11 * tile + 5.0});
    CHECK(m.x0 == 10 * tile);
    CHECK(m.width() == 2 * tile);
    CHECK(m.height() == 2 * tile);
    CHECK(m.valid(0, 0) == 1);
    CHECK(m.valid(0, tile) == 0);
    CHECK(m.rgb[3 * static_cast<std::size_t>(tile + 1)] == 11);
}

TEST_CASE("prefetch: the first frame builds at once") {
    const TempDir dir;
    FakeServer server;
    MosaicCache cache(std::make_shared<TileSource>(imagery_url, dir.str(), server.fetcher()));
    const auto mosaic = cache.get(zoom, box(0, 0), false);
    CHECK(mosaic->covers(box(0, 0)));
    CHECK(cache.builds() == 1);
}

TEST_CASE("prefetch: a frame near the edge prefetches") {
    const TempDir dir;
    FakeServer server;
    MosaicCache cache(std::make_shared<TileSource>(imagery_url, dir.str(), server.fetcher()));
    const auto first = cache.get(zoom, box(0, 0), false);
    const auto right_edge = static_cast<double>(first->x0 + first->width());
    const double size = 300.0;
    const double origin = 70'000.0 * tile;
    const auto near_edge = box(right_edge - prefetch_margin / 2 - size - origin, 0, size);
    REQUIRE(first->covers(near_edge));
    CHECK(cache.get(zoom, near_edge, false) == first);
    wait_idle(cache);
    CHECK(cache.builds() == 2);
    const auto latest = cache.latest(zoom);
    REQUIRE(latest);
    CHECK(latest != first);
    CHECK(latest->covers(near_edge.grown(prefetch_margin)));
}

TEST_CASE("prefetch: an outrun prefetch keeps old data") {
    const TempDir dir;
    FakeServer server;
    MosaicCache cache(std::make_shared<TileSource>(imagery_url, dir.str(), server.fetcher()));
    const auto first = cache.get(zoom, box(0, 0), false);
    const auto far_away = box(10.0 * tile, 0);
    CHECK(cache.get(zoom, far_away, false) == first);
    wait_idle(cache);
    CHECK(cache.get(zoom, far_away, false)->covers(far_away));
}

TEST_CASE("prefetch: a waiting caller always gets coverage") {
    const TempDir dir;
    FakeServer server;
    MosaicCache cache(std::make_shared<TileSource>(imagery_url, dir.str(), server.fetcher()));
    std::ignore = cache.get(zoom, box(0, 0));
    const auto far_away = box(10.0 * tile, 0);
    CHECK(cache.get(zoom, far_away)->covers(far_away));
}

TEST_CASE("prefetch: built mosaics are handed over") {
    const TempDir dir;
    FakeServer server;
    MosaicCache cache(std::make_shared<TileSource>(imagery_url, dir.str(), server.fetcher()));
    std::vector<std::shared_ptr<const Mosaic>> seen;
    cache.set_on_built([&](const std::shared_ptr<const Mosaic>& m) { seen.push_back(m); });
    const auto mosaic = cache.get(zoom, box(0, 0));
    REQUIRE(seen.size() == 1);
    CHECK(seen.front() == mosaic);
}

TEST_CASE("prefetch: two cameras far apart keep their mosaics") {
    const TempDir dir;
    FakeServer server;
    MosaicCache cache(std::make_shared<TileSource>(imagery_url, dir.str(), server.fetcher()));
    const auto drone = cache.get(zoom, box(0, 0));
    const auto dock = cache.get(zoom, box(20.0 * tile, 0));
    CHECK_FALSE(dock->covers(box(0, 0)));
    const int builds = cache.builds();
    CHECK(cache.get(zoom, box(0, 0), false) == drone);
    CHECK(cache.get(zoom, box(20.0 * tile, 0), false) == dock);
    CHECK(cache.builds() == builds);
}

TEST_CASE("mosaic builds from several threads at once are safe") {
    const TempDir dir;
    FakeServer server;
    MosaicCache cache(std::make_shared<TileSource>(imagery_url, dir.str(), server.fetcher()));
    std::vector<std::jthread> threads;
    std::atomic<int> covered = 0;
    threads.reserve(6);
    for (int i = 0; i < 6; ++i) {
        threads.emplace_back([&, i] {
            for (int k = 0; k < 5; ++k) {
                const auto b = box(static_cast<double>((i + k) % 4) * tile, 0);
                const bool wait = k % 2 == 0;
                const auto m = cache.get(zoom, b, wait);
                // Without wait the cache may hand out older data that misses
                // the box while a prefetch runs; only a waiting call must cover it.
                if (m && (!wait || m->overlap(b) > 0)) ++covered;
            }
        });
    }
    threads.clear();
    wait_idle(cache);
    CHECK(covered == 30);
}

TEST_CASE("terrain decodes Terrarium and caches points") {
    const TempDir dir;
    FakeServer server;
    Terrain terrain("https://dem.test/{z}/{x}/{y}.png", 14, dir.str(), 15, 1500.0,
                    server.fetcher());
    CHECK(terrain.point(47.263, 11.401) == Approx(elevation_m).margin(0.01));
    const int used = server.requests;
    CHECK(terrain.point(47.263, 11.401) == Approx(elevation_m).margin(0.01));
    CHECK(server.requests == used);

    const auto relief = terrain.relief(47.263, 11.401, 3000.0);
    REQUIRE(relief.far);
    REQUIRE(relief.near);
    CHECK(relief.near->zoom == 15);
    CHECK(relief.near_radius == 1500.0);
    CHECK(relief.top() == Approx(elevation_m).margin(0.01));
    CHECK(relief.height(47.263, 11.401, 0, 0, 0.0F) == Approx(elevation_m).margin(0.01));

    const raster::LocalFrame frame{.lat0 = 47.263, .lon0 = 11.401};
    const auto kernel = to_relief(relief, frame, 500.0F);
    REQUIRE(kernel.far.mosaic);
    REQUIRE(kernel.near.mosaic);
    CHECK(kernel.near_radius == 1500.0F);
    // The kernel map puts the camera inside the mosaic.
    const auto [x, y] = kernel.near.map.apply(0.0F, 0.0F);
    CHECK(x > 0.0F);
    CHECK(y > 0.0F);
    CHECK(x < static_cast<float>(kernel.near.mosaic->values.width()));
    CHECK(raster::bilinear(kernel.near.mosaic->values, x, y).value_or(0) ==
          Approx(elevation_m).margin(0.01));
    // The view shares the mosaic; a frame copies no pixels.
    CHECK(kernel.near.mosaic == relief.near);
}

TEST_CASE("the near DEM fades into the far DEM") {
    const auto flat = [](float value, int z) {
        auto m = std::make_shared<Mosaic>();
        m->zoom = z;
        m->valid = raster::Grid2D<std::uint8_t>(1024, 1024, 1);
        m->values = raster::Grid2D<float>(1024, 1024, value);
        m->top = value;
        return std::shared_ptr<const Mosaic>(m);
    };
    const ReliefMosaics relief{
        .far = flat(100.0F, 2), .near = flat(110.0F, 2), .near_radius = 1000.0};
    CHECK(relief.height(0, 0, 0, 0, 0.0F) == 110.0F);
    const float mid = relief.height(0, 0, near_blend_start * 1000.0 + 100.0, 0, 0.0F);
    CHECK(mid > 100.0F);
    CHECK(mid < 110.0F);
    CHECK(relief.height(0, 0, 2000.0, 0, 0.0F) == 100.0F);
}

TEST_CASE("imagery levels cover the view, sharpest first") {
    const TempDir dir;
    FakeServer server;
    Imagery imagery(imagery_url, 19, dir.str(), server.fetcher());
    const raster::LocalFrame frame{.lat0 = 47.263, .lon0 = 11.401};
    const auto [s, w] = frame.to_latlon(-800, -800);
    const auto [n, e] = frame.to_latlon(800, 800);
    const auto mosaics =
        imagery.levels({.south = s, .west = w, .north = n, .east = e}, frame.lat0, frame.lon0, 0.3);
    REQUIRE(!mosaics.empty());
    CHECK(mosaics.size() <= static_cast<std::size_t>(level_count));
    for (std::size_t i = 1; i < mosaics.size(); ++i) CHECK(mosaics[i]->zoom < mosaics[i - 1]->zoom);
    CHECK(texel_metres(frame.lat0, 0) == Approx(raster::tile_metres(frame.lat0, 0) / tile));

    const auto levels = to_levels(mosaics, frame);
    REQUIRE(levels.size() == mosaics.size());
    // The red channel holds the tile column at the level's zoom.
    const auto rgb = sample_levels(levels, 0.0F, 0.0F, levels.front().zoom + 1.0F);
    const auto [gx, gy] = raster::global_pixel(frame.lat0, frame.lon0, mosaics.front()->zoom);
    CHECK(rgb[0] == Approx(static_cast<double>(static_cast<int>(gx / tile) % 256)).margin(1.0));
    CHECK(rgb[2] == Approx(mosaics.front()->zoom).margin(0.5));
    // Far outside every level: the no-data colour.
    CHECK(sample_levels(levels, 1e7F, 0.0F, 0.0F) == no_data_rgb);
}
