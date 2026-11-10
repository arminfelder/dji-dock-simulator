#pragma once

#include "image/image.hpp"
#include "osm/provider.hpp"
#include "render/camera.hpp"
#include "render/geodata/imagery.hpp"
#include "render/geodata/terrain.hpp"

#include <docksim/config.hpp>

#include <asio/thread_pool.hpp>

#ifdef DOCKSIM_WITH_VULKAN_RT
namespace docksim::render::rt {
class FrameRenderer;
}
#endif

#include <map>
#include <memory>
#include <mutex>
#include <string>

// The renderer: one camera frame from orthophoto, DEM and OSM buildings.
namespace docksim::render {

struct FrameOptions {
    // False never blocks on Overpass or tile fetches: it renders with the data
    // at hand while the next data loads. The live feed uses it.
    bool wait_for_data = true;
    // Rays per pixel side, averaged.
    int supersample = 1;
    // Full-resolution rays start just before the nearest hit of a march on a
    // grid this many times coarser. 1 marches every pixel from the camera.
    int march_scale = 4;
    // Seconds per phase, added up; null for none.
    std::map<std::string, double>* profile = nullptr;
};

// One renderer per process: the feeds and the photos share its caches.
class Renderer {
public:
    struct Sources {
        std::shared_ptr<geodata::Imagery> imagery;
        std::shared_ptr<geodata::Terrain> terrain;
        // Null: no buildings.
        std::shared_ptr<osm::BuildingProvider> buildings;
        bool shadows = true;
        // Tests compare frames without the sensor noise.
        bool sensor_noise = true;
        // Render threads.
        int threads = 8;
#ifdef DOCKSIM_WITH_VULKAN_RT
        // Hardware ray tracing. Null: the CPU kernels.
        std::shared_ptr<rt::FrameRenderer> rt;
#endif
    };

    explicit Renderer(Sources sources);
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    Renderer(Renderer&&) = delete;
    Renderer& operator=(Renderer&&) = delete;
    ~Renderer();

    // From the media section: tile URLs, cache, buildings, shadows.
    [[nodiscard]] static std::unique_ptr<Renderer> from_config(const MediaConfig& media,
                                                               int threads = 8);

    // One frame, width x height pixels, dfov the diagonal field of view in degrees.
    // Safe to call from several threads; frames of one renderer take turns.
    [[nodiscard]] image::RgbImage render(const Pose& pose, int width, int height, double dfov_deg,
                                         const FrameOptions& options = {});

    [[nodiscard]] bool gpu() const noexcept;

private:
    [[nodiscard]] image::RgbImage render_rt(const Pose& pose, int width, int height,
                                            double dfov_deg, const FrameOptions& options);

    Sources sources_;
    asio::thread_pool pool_;
    std::mutex frame_mutex_;
};

}  // namespace docksim::render
