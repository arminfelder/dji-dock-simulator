#pragma once

#include "render/camera.hpp"
#include "render/geodata/tiles.hpp"
#include "render/rt/scene.hpp"
#include "render/rt/vk.hpp"
#include "render/sun.hpp"

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <span>
#include <vector>

// One camera frame by hardware ray tracing: trace, pick texture levels on the
// host, shade, average. Only a subsampled distance buffer and the final image
// cross to the host; a full distance readback cost about 58 ms per frame.
namespace docksim::render::rt {

// The host reads every stride-th distance per direction to pick texture levels.
inline constexpr int level_stride = 8;

struct Job {
    // Unit rays in camera axes, row-major, width * height.
    std::shared_ptr<const std::vector<Vec3>> rays;
    int width = 0;
    int height = 0;
    double dfov = 84.0;
    Rotation rot{};
    // Metres east, north, up of the scene origin.
    Vec3 camera;
    double latitude = 0;
    double longitude = 0;
    float reach = 0;
    SunPosition sun_position{};
    Vec3 sun;
    bool shadows = true;
    std::uint32_t seed = 0;
    bool noise = true;
    float pixel_angle = 0;
    float texel0 = 0;
    int supersample = 1;
    bool wait = true;
};

// Distances of every stride-th ray per direction (infinity for a miss), cols x
// rows, row-major, and the stride. Returns the imagery mosaics, finest first.
using LevelPicker = std::function<std::vector<std::shared_ptr<const geodata::Mosaic>>(
    std::span<const float> distance, int cols, int rows, int stride)>;

class FrameRenderer {
public:
    FrameRenderer(std::shared_ptr<vk::Context> ctx, std::shared_ptr<Scene> scene);
    FrameRenderer(const FrameRenderer&) = delete;
    FrameRenderer& operator=(const FrameRenderer&) = delete;
    FrameRenderer(FrameRenderer&&) = delete;
    FrameRenderer& operator=(FrameRenderer&&) = delete;
    ~FrameRenderer();

    // Output size is job size / supersample.
    [[nodiscard]] image::RgbImage frame(const Job& job, const LevelPicker& levels_for);

    [[nodiscard]] Scene& scene() { return *scene_; }
    [[nodiscard]] const std::string& device_name() const { return ctx_->name(); }

private:
    struct Programs;
    // The caches hand out shared_ptr copies: a frame keeps its buffers while it waits for
    // imagery without the lock, and another frame may change the cache meanwhile.
    [[nodiscard]] std::shared_ptr<const vk::Buffer> rays_buffer(const Job& job);
    [[nodiscard]] std::shared_ptr<vk::Buffer> level_buffer(
        const std::shared_ptr<const geodata::Mosaic>& mosaic);
    [[nodiscard]] std::shared_ptr<const vk::Buffer> styles_buffer(int& count);

    std::shared_ptr<vk::Context> ctx_;
    std::shared_ptr<Scene> scene_;
    std::unique_ptr<Programs> programs_;
    std::mutex mutex_;
    vk::Buffer atlas_;
    vk::Buffer normals_;
    vk::Buffer mat_info_;
    vk::Buffer layouts_;
    vk::Buffer empty_level_;
    std::shared_ptr<const SceneSet> empty_scene_;
    std::uint64_t styles_version_ = ~std::uint64_t{0};
    int style_count_ = 0;
    std::shared_ptr<const vk::Buffer> styles_;
    // Keyed by the ray set itself; camera_rays hands out one shared set per size and lens.
    std::map<const std::vector<Vec3>*,
             std::pair<std::weak_ptr<const std::vector<Vec3>>, std::shared_ptr<const vk::Buffer>>>
        rays_;
    std::map<const geodata::Mosaic*,
             std::pair<std::weak_ptr<const geodata::Mosaic>, std::shared_ptr<vk::Buffer>>>
        levels_;
};

}  // namespace docksim::render::rt
