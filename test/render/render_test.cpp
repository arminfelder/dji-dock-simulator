// Pixel snapshots of the CPU renderer, rendered offline from scene.cpp. A change
// in the look must be reviewed: set DOCKSIM_UPDATE_RENDER=1, check the PNGs in
// test/render/snapshots, commit them.

#include "image/image.hpp"
#include "scene.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>

using namespace docksim;

namespace {

constexpr int width = 96;
constexpr int height = 72;
// uint8 levels. Optimisation levels round floats differently; a ray that
// grazes a building edge then flips between wall and ground. So a frame
// matches when nearly every value is within tolerance and the mean stays low.
constexpr int tolerance = 2;
constexpr double max_outlier_share = 0.005;
constexpr double max_mean = 0.5;

const std::filesystem::path snapshots =
    std::filesystem::path(DOCKSIM_SOURCE_DIR) / "test/render/snapshots";

struct Case {
    const char* name;
    // Metres north of home, height, heading, gimbal pitch, UTC hour.
    double north;
    double height;
    double heading;
    double pitch;
    double hour;
};

std::vector<std::byte> read(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::vector<char> chars{std::istreambuf_iterator<char>(in), {}};
    std::vector<std::byte> out(chars.size());
    std::ranges::transform(chars, out.begin(), [](char c) { return static_cast<std::byte>(c); });
    return out;
}

void write(const std::filesystem::path& path, std::span<const std::byte> data) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream(path, std::ios::binary)
        .write(reinterpret_cast<const char*>(data.data()),
               static_cast<std::streamsize>(data.size()));
}

void check_close(const image::RgbImage& frame, const image::RgbImage& expected) {
    REQUIRE(expected.width() == frame.width());
    REQUIRE(expected.height() == frame.height());
    std::size_t outliers = 0;
    double sum = 0;
    int worst = 0;
    for (std::size_t i = 0; i < frame.bytes().size(); ++i) {
        const int d =
            std::abs(static_cast<int>(frame.bytes()[i]) - static_cast<int>(expected.bytes()[i]));
        worst = std::max(worst, d);
        sum += d;
        if (d > tolerance) ++outliers;
    }
    const auto n = static_cast<double>(frame.bytes().size());
    INFO("max diff " << worst << ", mean " << sum / n << ", outliers " << outliers);
    CHECK(static_cast<double>(outliers) / n <= max_outlier_share);
    CHECK(sum / n <= max_mean);
}

}  // namespace

TEST_CASE("CPU render matches its snapshot", "[render]") {
    const auto c = GENERATE(Case{"nadir_noon", 0.0, 60.0, 0.0, -90.0, 10.0},
                            Case{"oblique_afternoon", 0.0, 80.0, 45.0, -20.0, 15.5},
                            Case{"low_sun_west", 0.0, 40.0, 270.0, -10.0, 18.5},
                            Case{"night", 0.0, 60.0, 0.0, -45.0, 22.5},
                            // Walls, windows and cast shadows of the scene buildings.
                            Case{"facades_morning", -150.0, 30.0, 0.0, -12.0, 6.0},
                            Case{"nadir_long_shadows", 0.0, 180.0, 0.0, -90.0, 5.0});
    INFO(c.name);
    test::OfflineScene scene;
    const auto frame = scene.renderer().render(
        test::pose(c.north, c.height, c.heading, c.pitch, c.hour), width, height, 84.0);
    REQUIRE(frame.width() == width);
    REQUIRE(frame.height() == height);

    const auto path = snapshots / (std::string(c.name) + ".png");
    if (std::getenv("DOCKSIM_UPDATE_RENDER") != nullptr || !std::filesystem::exists(path)) {
        write(path, image::encode_png(frame));
        if (std::getenv("DOCKSIM_UPDATE_RENDER") == nullptr)
            FAIL(path.string() << " was missing and is now written; check it");
        return;
    }
    check_close(frame, image::decode(read(path)));
}

TEST_CASE("supersampling and the coarse march keep the frame size", "[render]") {
    test::OfflineScene scene;
    render::FrameOptions options;
    options.supersample = 2;
    options.march_scale = 1;
    const auto frame =
        scene.renderer().render(test::pose(0.0, 60.0, 0.0, -45.0, 10.0), 32, 24, 84.0, options);
    CHECK(frame.width() == 32);
    CHECK(frame.height() == 24);
}

#ifdef DOCKSIM_WITH_VULKAN_RT
#include "render/rt/vk.hpp"

TEST_CASE("RT render matches its snapshot", "[render][rt]") {
    const auto c = GENERATE(Case{"nadir_noon", 0.0, 60.0, 0.0, -90.0, 10.0},
                            Case{"oblique_afternoon", 0.0, 80.0, 45.0, -20.0, 15.5},
                            Case{"low_sun_west", 0.0, 40.0, 270.0, -10.0, 18.5},
                            Case{"night", 0.0, 60.0, 0.0, -45.0, 22.5},
                            Case{"facades_morning", -150.0, 30.0, 0.0, -12.0, 6.0},
                            Case{"nadir_long_shadows", 0.0, 180.0, 0.0, -90.0, 5.0});
    INFO(c.name);
    std::unique_ptr<test::OfflineScene> scene;
    try {
        scene = std::make_unique<test::OfflineScene>(200.0, true);
    } catch (const render::rt::vk::NoDevice&) {
        SKIP("no Vulkan device with ray queries");
    }
    const auto frame = scene->renderer().render(
        test::pose(c.north, c.height, c.heading, c.pitch, c.hour), width, height, 84.0);
    REQUIRE(frame.width() == width);
    const auto path = snapshots / ("rt_" + std::string(c.name) + ".png");
    if (std::getenv("DOCKSIM_UPDATE_RENDER") != nullptr || !std::filesystem::exists(path)) {
        write(path, image::encode_png(frame));
        if (std::getenv("DOCKSIM_UPDATE_RENDER") == nullptr)
            FAIL(path.string() << " was missing and is now written; check it");
        return;
    }
    check_close(frame, image::decode(read(path)));
}
#endif
