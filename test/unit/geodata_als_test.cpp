#include "raster/als.hpp"
#include "raster/mercator.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <miniz.h>

#include <bit>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <random>

using namespace docksim;
using Catch::Approx;

namespace {

// The TIFF floating point predictor, as the BEV files use it.
std::vector<std::uint8_t> predict_float(const std::vector<float>& values, int w, int h) {
    std::vector<std::uint8_t> out(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4);
    const auto uw = static_cast<std::size_t>(w);
    for (int y = 0; y < h; ++y) {
        auto* row = out.data() + static_cast<std::size_t>(y) * uw * 4;
        for (std::size_t x = 0; x < uw; ++x) {
            const auto bits =
                std::bit_cast<std::uint32_t>(values[static_cast<std::size_t>(y) * uw + x]);
            for (std::size_t k = 0; k < 4; ++k)
                row[k * uw + x] = static_cast<std::uint8_t>(bits >> (24U - 8U * k));
        }
        std::uint8_t prev = 0;
        for (std::size_t i = 0; i < uw * 4; ++i) {
            const auto cur = row[i];
            row[i] = static_cast<std::uint8_t>(cur - prev);
            prev = cur;
        }
    }
    return out;
}

template <class T>
void put(raster::als::Bytes& out, std::size_t at, T v) {
    if (out.size() < at + sizeof(T)) out.resize(at + sizeof(T));
    std::memcpy(out.data() + at, &v, sizeof(T));
}

// A 512 x 512 px BigTIFF (2 x 2 blocks) whose pixel (c, r) holds f(c, r).
// Its top-left corner is at EPSG:3035 (ox, oy).
raster::als::Bytes bigtiff(double ox, double oy, float (*f)(int, int)) {
    constexpr int size = 512;
    constexpr int blocks = 4;
    raster::als::Bytes file;
    file.resize(16);
    std::memcpy(file.data(), "II+\0", 4);
    put<std::uint16_t>(file, 4, 8);
    put<std::uint16_t>(file, 6, 0);
    put<std::uint64_t>(file, 8, 16);
    constexpr std::size_t ifd = 16;
    constexpr std::size_t tables = ifd + 8 + 4 * 20 + 8;
    constexpr std::size_t offsets_at = tables;
    constexpr std::size_t counts_at = tables + 8 * blocks;
    constexpr std::size_t geo_at = counts_at + 8 * blocks;
    std::size_t data_at = geo_at + 48;
    put<std::uint64_t>(file, ifd, 4);
    const auto entry = [&](std::size_t k, std::uint16_t tag, std::uint16_t type,
                           std::uint64_t count, std::uint64_t v) {
        const std::size_t e = ifd + 8 + 20 * k;
        put(file, e, tag);
        put(file, e + 2, type);
        put(file, e + 4, count);
        put(file, e + 12, v);
    };
    entry(0, 256, 16, 1, size);
    entry(1, 324, 16, blocks, offsets_at);
    entry(2, 325, 16, blocks, counts_at);
    entry(3, 33922, 12, 6, geo_at);
    for (int k = 0; k < 6; ++k)
        put<double>(file, geo_at + 8 * static_cast<std::size_t>(k),
                    k == 3   ? ox
                    : k == 4 ? oy
                             : 0.0);
    for (int b = 0; b < blocks; ++b) {
        std::vector<float> values(256 * 256);
        for (int r = 0; r < 256; ++r) {
            for (int c = 0; c < 256; ++c)
                values[static_cast<std::size_t>(r * 256 + c)] =
                    f((b % 2) * 256 + c, (b / 2) * 256 + r);
        }
        const auto plain = predict_float(values, 256, 256);
        mz_ulong len = mz_compressBound(plain.size());
        std::vector<unsigned char> packed(len);
        REQUIRE(mz_compress(packed.data(), &len, plain.data(), plain.size()) == MZ_OK);
        put<std::uint64_t>(file, offsets_at + 8 * static_cast<std::size_t>(b), data_at);
        put<std::uint64_t>(file, counts_at + 8 * static_cast<std::size_t>(b), len);
        file.resize(data_at + len);
        std::memcpy(file.data() + data_at, packed.data(), len);
        data_at += len;
    }
    return file;
}

std::filesystem::path temp_dir(std::string_view name) {
    std::random_device rd;
    auto dir =
        std::filesystem::temp_directory_path() / std::format("docksim-{}-{:08x}", name, rd());
    std::filesystem::create_directories(dir);
    return dir;
}

}  // namespace

TEST_CASE("laea matches the EPSG example") {
    // EPSG guidance note 7-2, ETRS89 / LAEA Europe: lat 50, lon 5.
    const auto [x, y] = raster::als::laea(50.0, 5.0);
    CHECK(x == Approx(3962799.45).margin(0.01));
    CHECK(y == Approx(2999718.85).margin(0.01));
}

TEST_CASE("float predictor round trip") {
    std::mt19937 rng(1);
    std::normal_distribution<float> dist(170.0F, 30.0F);
    std::vector<float> values(8 * 16);
    for (auto& v : values) v = dist(rng);
    CHECK(raster::als::unpredict_float(predict_float(values, 16, 8), 16, 8) == values);
}

TEST_CASE("pixel map matches exact mercator") {
    const double lat0 = GENERATE(0.0, 47.263, 64.1);
    const int zoom = GENERATE(14, 19);
    const raster::LocalFrame frame{.lat0 = lat0, .lon0 = 11.401};
    // Mosaics start near the camera, so cx and cy stay small in float32.
    const auto [gx, gy] = raster::global_pixel(lat0, 11.401, zoom);
    const double x0 = std::floor(gx) - 1234.0;
    const double y0 = std::floor(gy) - 5678.0;
    const auto m = frame.pixel_map(zoom, x0, y0);
    double worst_x = 0;
    double worst_y = 0;
    for (int i = 0; i <= 40; ++i) {
        for (int j = 0; j <= 40; ++j) {
            const double east = -5000 + 250.0 * i;
            const double north = -5000 + 250.0 * j;
            const auto [lat, lon] = frame.to_latlon(east, north);
            const auto [px, py] = raster::global_pixel(lat, lon, zoom);
            // The map is float32 by design; compare with the precision a kernel gets.
            const auto [ax, ay] = m.apply(static_cast<float>(east), static_cast<float>(north));
            worst_x = std::max(worst_x, std::abs(ax - (px - x0 - 0.5)));
            worst_y = std::max(worst_y, std::abs(ay - (py - y0 - 0.5)));
        }
    }
    INFO("lat0 " << lat0 << " zoom " << zoom);
    // The Python version, in float64, allows 1e-6 in x and 1e-2 in y.
    CHECK(worst_x < 0.01);
    CHECK(worst_y < 0.02);
}

TEST_CASE("ALS layer reads a window from a BigTIFF by ranges and caches it") {
    const double ox = 4822000.0;
    const double oy = 2637000.0;
    const auto file = bigtiff(ox, oy, [](int c, int r) {
        return c == 300 && r == 300
                   ? -9999.0F
                   : 100.0F + static_cast<float>(c) * 0.01F + static_cast<float>(r);
    });
    int requests = 0;
    std::string seen_url;
    const raster::als::RangeFetcher fetch = [&](const std::string& url, std::uint64_t start,
                                                std::uint64_t end) {
        ++requests;
        seen_url = url;
        const auto last = std::min<std::uint64_t>(end, file.size() - 1);
        return std::optional(
            raster::als::Bytes(file.begin() + static_cast<std::ptrdiff_t>(start),
                               file.begin() + static_cast<std::ptrdiff_t>(last) + 1));
    };
    const auto dir = temp_dir("als");
    // The file named N2600000E4800000 covers easting 4800000.. but its tie point says ox, oy.
    raster::als::Layer layer("DSM", dir.string(), "https://als.test/{kind}/N{n}E{e}.tif", fetch);
    const auto win = layer.window(ox + 290.2, oy - 310.0, ox + 310.0, oy - 290.5);
    CHECK(seen_url.starts_with("https://als.test/DSM/N2600000E4800000.tif"));
    // Pixel (0, 0) of the window is column 290, row 290 of the file.
    CHECK(win.x0 == ox + 290.5);
    CHECK(win.y0 == oy - 290.5);
    CHECK(win.heights(0, 0) == Approx(100.0 + 2.90 + 290.0));
    CHECK(std::isnan(win.heights(10, 10)));
    CHECK(win.heights(15, 12) == Approx(100.0 + 3.05 + 302.0));
    const int used = requests;
    CHECK(std::filesystem::exists(dir / "als" / "dsm" / "N2600000E4800000.idx.bin"));

    // A new layer reads index and blocks from disk without the network.
    raster::als::Layer offline(
        "DSM", dir.string(), "https://als.test/{kind}/N{n}E{e}.tif",
        [](const std::string&, std::uint64_t, std::uint64_t) -> std::optional<raster::als::Bytes> {
            throw std::runtime_error("network used");
        });
    const auto again = offline.window(ox + 290.2, oy - 310.0, ox + 310.0, oy - 290.5);
    CHECK(again.heights(15, 12) == win.heights(15, 12));
    CHECK(used == 3 + 1);
    std::filesystem::remove_all(dir);
}

TEST_CASE("a missing ALS file gives NaN") {
    const auto dir = temp_dir("als-missing");
    raster::als::Layer layer("DTM", dir.string(), "https://als.test/{kind}/N{n}E{e}.tif",
                             [](const std::string&, std::uint64_t, std::uint64_t) {
                                 return std::optional<raster::als::Bytes>{};
                             });
    const auto win = layer.window(10.0, 10.0, 20.0, 20.0);
    CHECK(std::ranges::all_of(win.heights.data(), [](float v) { return std::isnan(v); }));
    std::filesystem::remove_all(dir);
}

TEST_CASE("height grid samples bilinear and takes peaks") {
    const raster::LocalFrame origin{.lat0 = 48.2258, .lon0 = 16.3733};
    const auto [x, y] = raster::als::laea(origin.lat0, origin.lon0);
    raster::Grid2D<float> h(20, 20, 0.0F);
    h(10, 10) = 9.0F;
    h(3, 3) = std::numeric_limits<float>::quiet_NaN();
    const raster::als::HeightGrid grid(h, std::floor(x) - 9.5, std::floor(y) + 9.5);
    const auto& peaks = grid.peaks();
    CHECK(peaks(9, 9) == 9.0F);
    CHECK(peaks(11, 11) == 9.0F);
    CHECK(peaks(13, 13) == 0.0F);
    CHECK(peaks(3, 3) == 0.0F);
    // Far outside the grid.
    CHECK(std::isnan(grid.sample(origin.lat0 + 1.0, origin.lon0)));
    const float centre = grid.sample(origin.lat0, origin.lon0);
    CHECK(centre >= 0.0F);
    CHECK(centre <= 9.0F);
    CHECK(grid.sample(origin.lat0, origin.lon0, true) >= centre);
}

TEST_CASE("a BigTIFF with broken tile tables is refused, not read past its end") {
    const double ox = 4822000.0;
    const double oy = 2637000.0;
    const auto good = bigtiff(ox, oy, [](int, int) { return 120.0F; });
    // IFD at 16: 8 bytes of entry count, then 20 bytes per entry; count at +4, value at +12.
    constexpr std::size_t counts_entry = 16 + 8 + 20 * 2;
    constexpr std::size_t offsets_entry = 16 + 8 + 20 * 1;
    const auto broken = GENERATE(as<std::string>{}, "short counts", "huge offset");
    auto file = good;
    if (broken == "short counts") put<std::uint64_t>(file, counts_entry + 4, 3);
    if (broken == "huge offset")
        put<std::uint64_t>(file, offsets_entry + 12, ~std::uint64_t{0} - 8);
    INFO(broken);
    const raster::als::RangeFetcher fetch =
        [&](const std::string&, std::uint64_t start,
            std::uint64_t end) -> std::optional<raster::als::Bytes> {
        if (start >= file.size()) return std::nullopt;
        const auto last = std::min<std::uint64_t>(end, file.size() - 1);
        return raster::als::Bytes(file.begin() + static_cast<std::ptrdiff_t>(start),
                                  file.begin() + static_cast<std::ptrdiff_t>(last) + 1);
    };
    const auto dir = temp_dir("als-broken");
    raster::als::Layer layer("DSM", dir.string(), "https://als.test/{kind}/N{n}E{e}.tif", fetch);
    // Block 3 (bottom right) is the one the short table has no count for.
    const auto win = layer.window(ox + 300.0, oy - 310.0, ox + 310.0, oy - 300.0);
    CHECK(std::isnan(win.heights(0, 0)));
    std::filesystem::remove_all(dir);
}
