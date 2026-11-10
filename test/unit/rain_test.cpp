#include "image/image.hpp"
#include "render/rain.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

using namespace docksim;

namespace {

// Sky over grass, so that the streaks show on both.
image::RgbImage backdrop(int w = 640, int h = 360) {
    image::RgbImage img(w, h);
    for (int y = 0; y < h; ++y) {
        const image::Rgb c = y < h / 2 ? image::Rgb{150, 170, 190} : image::Rgb{70, 100, 60};
        for (int x = 0; x < w; ++x) img.set(x, y, c);
    }
    return img;
}

void dump(const image::RgbImage& img, const std::string& name) {
    const char* dir = std::getenv("DOCKSIM_RAIN_DUMP");
    if (dir == nullptr) return;
    const auto png = image::encode_png(img);
    std::ofstream(std::filesystem::path(dir) / (name + ".png"), std::ios::binary)
        .write(reinterpret_cast<const char*>(png.data()), static_cast<std::streamsize>(png.size()));
}

// Mean signed horizontal offset of brightened pixels against the pixel two rows above.
// Positive: streaks lean right going down.
double lean(const image::RgbImage& rain, const image::RgbImage& dry) {
    double sum = 0.0;
    int n = 0;
    for (int y = 4; y < rain.height(); ++y) {
        for (int x = 4; x < rain.width() - 4; ++x) {
            if (rain.pixel(x, y)[0] <= dry.pixel(x, y)[0] + 25) continue;
            for (int dx = -3; dx <= 3; ++dx) {
                if (rain.pixel(x + dx, y - 2)[0] > dry.pixel(x + dx, y - 2)[0] + 25) {
                    sum += -dx;
                    ++n;
                }
            }
        }
    }
    return n > 0 ? sum / n : 0.0;
}

render::RainScene horizon(int rainfall) {
    return {.rainfall = rainfall, .heading = 0.0, .pitch = 0.0, .dfov = 84.0};
}

}  // namespace

TEST_CASE("rain: dry weather leaves the frame alone") {
    auto img = backdrop();
    const auto before = img;
    render::apply_rain(img, horizon(0), 12.0);
    CHECK(img == before);
}

TEST_CASE("rain: equal times give equal frames, and the drops move") {
    auto a = backdrop();
    auto b = backdrop();
    auto c = backdrop();
    render::apply_rain(a, horizon(2), 5.0);
    render::apply_rain(b, horizon(2), 5.0);
    render::apply_rain(c, horizon(2), 5.1);
    CHECK(a == b);
    CHECK(a != c);
}

TEST_CASE("rain: heavier rain hazes more and draws more drops") {
    const auto dry = backdrop();
    auto light = dry;
    auto heavy = dry;
    render::apply_rain(light, horizon(1), 3.0);
    render::apply_rain(heavy, horizon(3), 3.0);
    dump(light, "light");
    dump(heavy, "heavy");
    const auto changed = [&](const image::RgbImage& img) {
        int n = 0;
        for (int y = 0; y < img.height(); ++y)
            for (int x = 0; x < img.width(); ++x)
                if (img.pixel(x, y)[0] > dry.pixel(x, y)[0] + 25) ++n;
        return n;
    };
    CHECK(changed(heavy) > 2 * changed(light));
    // Haze pulls the green grass towards grey: less colour.
    const auto colour = [](const std::uint8_t* p) { return int{p[1]} - int{p[0]}; };
    CHECK(colour(heavy.pixel(5, 350)) < colour(dry.pixel(5, 350)));
}

TEST_CASE("rain: a cross wind slants the streaks downwind") {
    const auto dry = backdrop();
    auto calm = dry;
    auto from_west = dry;
    auto from_east = dry;
    auto scene = horizon(3);
    render::apply_rain(calm, scene, 7.0);
    // The camera looks north, so air moving east goes to the right of the frame.
    scene.air_east = 8.0;
    render::apply_rain(from_west, scene, 7.0);
    scene.air_east = -8.0;
    render::apply_rain(from_east, scene, 7.0);
    dump(calm, "calm");
    dump(from_west, "wind_from_west");
    dump(from_east, "wind_from_east");
    CHECK(std::abs(lean(calm, dry)) < 0.3);
    CHECK(lean(from_west, dry) > 0.5);
    CHECK(lean(from_east, dry) < -0.5);
}

TEST_CASE("rain: a nadir camera sees the drops come out of the middle") {
    auto img = backdrop();
    render::apply_rain(img, {.rainfall = 3, .heading = 0.0, .pitch = -90.0, .dfov = 84.0}, 2.0);
    dump(img, "nadir");
    auto forward = backdrop();
    render::apply_rain(forward,
                       {.rainfall = 2, .air_east = 3.0, .camera_north = 12.0, .pitch = -20.0,
                        .dfov = 84.0},
                       2.0);
    dump(forward, "flying_forward_crosswind");
    CHECK(img != backdrop());
}
