// The ray tracing scene shows pieces as they finish.

#ifdef DOCKSIM_WITH_VULKAN_RT

#include "osm/cells.hpp"
#include "osm/overpass.hpp"
#include "render/rt/scene.hpp"
#include "render/rt/vk.hpp"
#include "scene.hpp"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <future>
#include <random>
#include <thread>

using namespace docksim;
using namespace docksim::test;

namespace {

// Holds every request until released.
class HeldSource final : public osm::BuildingSource {
public:
    std::optional<std::vector<Json>> buildings(double /*south*/, double /*west*/, double /*north*/,
                                               double /*east*/,
                                               const std::stop_token& /*stop*/) override {
        released_.wait();
        return std::vector<Json>{};
    }
    void release() { gate_.set_value(); }

private:
    std::promise<void> gate_;
    std::shared_future<void> released_ = gate_.get_future().share();
};

struct Release {
    HeldSource& source;
    explicit Release(HeldSource& s) : source(s) {}
    Release(const Release&) = delete;
    Release& operator=(const Release&) = delete;
    Release(Release&&) = delete;
    Release& operator=(Release&&) = delete;
    ~Release() { source.release(); }
};

}  // namespace

TEST_CASE("buildings show while another building cell still loads", "[rt]") {
    constexpr const char* lavapipe = "/usr/share/vulkan/icd.d/lvp_icd.json";
    if (std::getenv("VK_ICD_FILENAMES") == nullptr && std::getenv("DOCKSIM_TEST_GPU") == nullptr &&
        std::filesystem::exists(lavapipe)) {
        ::setenv("VK_ICD_FILENAMES", lavapipe, 1);
    }
    std::shared_ptr<render::rt::vk::Context> ctx;
    try {
        ctx = render::rt::vk::Context::open(std::nullopt, true);
    } catch (const render::rt::vk::NoDevice&) {
        SKIP("no Vulkan device with ray queries");
    }

    std::random_device rd;
    const auto cache =
        std::filesystem::temp_directory_path() / std::format("docksim-rt-scene-{:08x}", rd());
    constexpr double radius = 300.0;
    write_cells(cache, radius);
    const auto cell = [](double v) { return static_cast<int>(std::floor(v / osm::cell_deg)); };
    // Both cells are near (within scan_radius plus half a cell), with or without a scan.
    const render::rt::CellKey home{.cy = cell(home_lat), .cx = cell(home_lon), .near = true};
    // Home sits 75 m east of the cell edge; the cell to the west is wanted too.
    const render::rt::CellKey held{.cy = home.cy, .cx = home.cx - 1, .near = true};
    REQUIRE(std::filesystem::remove(cache / osm::building_cache_dir /
                                    std::format("{}_{}.json", held.cy, held.cx)));

    auto source = std::make_shared<HeldSource>();
    {
        render::rt::Scene scene(
            ctx, {.terrain = std::make_shared<render::geodata::Terrain>(
                      "https://dem.test/{z}/{x}/{y}.png", 14, cache.string(), 0, 0.0, fake_get),
                  .cells = std::make_shared<osm::CellStore>(cache, source, osm::FootprintOptions{}),
                  .scan = nullptr,
                  .building_radius = radius,
                  .far_radius = radius,
                  .scan_radius = 0.0});
        const Release release{*source};
        scene.set_origin(home_lat, home_lon, static_cast<float>(elevation(home_lat, home_lon)));

        const auto wanted = scene.wanted(home_lat, home_lon, 2000.0);
        REQUIRE(std::ranges::find(wanted, render::rt::PieceKey{held}) != wanted.end());
        std::shared_ptr<const render::rt::SceneSet> set;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{120};
        while (std::chrono::steady_clock::now() < deadline) {
            set = scene.frame(home_lat, home_lon, 2000.0, false);
            if (set && set->keys.contains(home)) break;
            std::this_thread::sleep_for(std::chrono::milliseconds{5});
        }
        REQUIRE(set);
        CHECK(set->keys.contains(home));
        CHECK_FALSE(set->keys.contains(held));
    }
    std::filesystem::remove_all(cache);
}

#endif
