#include "live/feed.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <thread>

using namespace docksim;

TEST_CASE("feed stats report once per interval", "[live]") {
    live::FeedStats stats(0.0);
    for (int i = 0; i < 30; ++i) stats.add_render(0.010);
    stats.add_render(0.050);
    stats.sent = 750;
    CHECK_FALSE(stats.report(live::stats_interval_s - 1));
    const auto line = stats.report(live::stats_interval_s);
    REQUIRE(line);
    CHECK(*line == "rendered 1.0 fps (avg 11.3 ms, max 50.0 ms), sent 25.0 fps");
    CHECK(stats.renders == 0);
    CHECK(stats.sent == 0);
}

namespace {

render::Pose pose() {
    return {.position = geo::point(47.263, 11.401),
            .height = units::above_home(50),
            .home = geo::point(47.263, 11.401),
            .heading = 90 * units::degree,
            .gimbal_pitch = -45 * units::degree,
            .time = std::chrono::system_clock::now()};
}

}  // namespace

TEST_CASE("a feed renders the current view with its overlay", "[live]") {
    std::atomic<int> renders = 0;
    std::atomic<bool> moving = true;
    live::Feed feed(
        {.name = "SN/80-0-0/normal-0", .width = 160, .height = 120, .fps = 50, .overlay = true},
        [&](const render::Pose&, int w, int h, double, const render::FrameOptions& o) {
            CHECK_FALSE(o.wait_for_data);
            ++renders;
            return image::RgbImage(w, h, {40, 120, 40});
        },
        [&](const std::string& lens) {
            return sim::View{
                .pose = pose(), .dfov = 84.0, .lens = lens, .idle_text = "", .still = !moving};
        });
    feed.start();
    for (int i = 0; i < 100 && feed.rendered() < 3; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    const auto frame = feed.latest();
    REQUIRE(frame);
    CHECK(frame->width() == 160);
    // The overlay draws black boxes and white text in the top left.
    const auto* corner = frame->pixel(8, 10);
    CHECK((corner[0] < 20 && corner[1] < 20));
    CHECK(std::ranges::any_of(frame->bytes(), [](auto b) { return b > 200; }));
    CHECK(feed.rendered() >= 3);

    // A still view is not rendered again within the refresh interval.
    moving = false;
    std::this_thread::sleep_for(std::chrono::milliseconds{60});
    const int before = renders;
    std::this_thread::sleep_for(std::chrono::milliseconds{150});
    CHECK(renders == before);
    feed.stop();
}

TEST_CASE("a feed without a pose shows dark noise", "[live]") {
    live::Feed feed(
        {.name = "dock", .width = 64, .height = 48, .fps = 50, .overlay = false},
        [](const render::Pose&, int w, int h, double, const render::FrameOptions&) {
            return image::RgbImage(w, h, {255, 0, 0});
        },
        [](const std::string&) {
            return sim::View{.pose = std::nullopt, .idle_text = "AIRCRAFT IN DOCK"};
        });
    feed.start();
    for (int i = 0; i < 100 && feed.rendered() < 1; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    feed.stop();
    const auto frame = feed.latest();
    CHECK(std::ranges::all_of(frame->bytes(), [](auto b) { return b >= 8 && b < 20; }));
}

TEST_CASE("the thermal look is grey and uses the full range", "[live]") {
    image::RgbImage img(32, 32);
    for (int y = 0; y < 32; ++y) {
        for (int x = 0; x < 32; ++x)
            img.set(x, y, {static_cast<std::uint8_t>(x * 8), static_cast<std::uint8_t>(y * 8), 50});
    }
    const auto t = live::thermal(img);
    for (int i = 0; i < 32 * 32; ++i) {
        const auto* p = t.pixel(i % 32, i / 32);
        CHECK((p[0] == p[1] && p[1] == p[2]));
    }
    const auto [lo, hi] = std::ranges::minmax(t.bytes());
    CHECK(lo == 0);
    CHECK(hi == 255);
}
