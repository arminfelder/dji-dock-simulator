#pragma once

#include "render/camera.hpp"
#include "render/geodata/tiles.hpp"
#include "render/renderer.hpp"

#include <filesystem>
#include <memory>
#include <string>

// An offline scene for render tests: synthetic orthophoto and DEM tiles from an
// injected fetcher, and OSM cells written into the cache in the layout the
// renderer reads. Nothing touches the network.
namespace docksim::test {

inline constexpr double home_lat = 47.263;
inline constexpr double home_lon = 11.401;

// A valley with a ridge to the north-east of home.
[[nodiscard]] double elevation(double lat, double lon);

// Synthetic tiles for https://imagery.test/{z}/{x}/{y}.png and https://dem.test/{z}/{x}/{y}.png.
[[nodiscard]] render::geodata::FetchResult fake_get(const std::string& url);

// Every OSM cell within radius_m of home, in cache/osm-buildings. All buildings go into the home
// cell.
void write_cells(const std::filesystem::path& cache, double radius_m);

class OfflineScene {
public:
    // building_radius_m: the near building ring. The far ring stays off.
    // ray_tracing: render through Vulkan (lavapipe in tests). Throws
    // render::rt::vk::NoDevice without a ray query device.
    explicit OfflineScene(double building_radius_m = 200.0, bool ray_tracing = false);
    OfflineScene(const OfflineScene&) = delete;
    OfflineScene& operator=(const OfflineScene&) = delete;
    OfflineScene(OfflineScene&&) = delete;
    OfflineScene& operator=(OfflineScene&&) = delete;
    ~OfflineScene();

    [[nodiscard]] render::Renderer& renderer() { return *renderer_; }
    [[nodiscard]] const std::filesystem::path& cache() const { return cache_; }

private:
    std::filesystem::path cache_;
    std::unique_ptr<render::Renderer> renderer_;
};

// UTC on 2026-06-21 at a fractional hour.
[[nodiscard]] sim::WallTime at(double hour);

[[nodiscard]] render::Pose pose(double north_m, double height, double heading, double pitch,
                                double hour);

}  // namespace docksim::test
