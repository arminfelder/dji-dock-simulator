#pragma once

#include "render/camera.hpp"
#include "render/inputs.hpp"
#include "render/materials.hpp"
#include "render/shading.hpp"
#include "render/vec.hpp"

#include <cstdint>
#include <span>
#include <vector>

// The heightfield renderer on the CPU: a C++ translation of the OpenCL kernels
// march, shade and downsample (render.cl of the Python version). One call per
// pixel; the frame driver splits pixels over a thread pool.
namespace docksim::render::cpu {

// Steps per ray, geometric from 0.5 m to the reach, and bisection steps after
// the first step below the surface.
inline constexpr int march_steps = 640;
inline constexpr int refine_steps = 12;
// Shadow rays towards the sun.
inline constexpr float shadow_range_m = 3000.0F;
inline constexpr int shadow_steps = 160;
// Shadow LoD: half the steps beyond shadow_full_m, none beyond shadow_max_m,
// where a shadow edge is smaller than a pixel.
inline constexpr float shadow_full_m = 1500.0F;
inline constexpr float shadow_max_m = 4000.0F;
// The footprint of a pixel grows with 1/cos of the incidence angle; capped at
// 1/min_cos, as anisotropic filtering would.
inline constexpr float min_cos = 0.2F;
// Shadow rays start this far off the surface, so that they do not hit it at once.
inline constexpr float shadow_lift_m = 0.3F;
// Finite-difference step of the surface normal. It matches the building grid.
inline constexpr float normal_step_m = 0.5F;
// The facade style and roof height are read this far inside the wall.
inline constexpr float inside_m = 0.6F;
// Wall hits move at most this far onto their outline plane, and only where the
// ray meets the plane at more than min_toward (cosine).
inline constexpr float max_snap_m = 1.5F;
inline constexpr float min_toward = 0.05F;
// An outline edge counts for a hit only within about 45 degrees of the grid normal.
inline constexpr float min_agree = 0.7F;
// The relief blend starts at this share of the near radius.
inline constexpr float near_blend_start = 0.8F;

// One facade style: base colour, window layout index, weathering share, and
// the photo material index (-1 for none).
struct StyleRow {
    Vec3 colour;
    int layout = 0;
    float weathering = 0;
    int material = -1;
};

struct Scene {
    Relief relief;
    BuildingView buildings;
    // Highest surface (terrain plus roofs) above the camera base, metres.
    float top = 0;
};

struct Surface {
    // Distance along the ray; infinity for sky.
    float distance{};
    Vec3 normal{0, 0, 1};
    // 1 sunlit, 0 in shadow.
    float lit = 1;
    // Roof height of the building at the hit, metres above terrain.
    float roof = 0;
    // Facade style id + 1, 0 for none.
    float style = 0;
    // Terrain height under the hit.
    float terrain = 0;
};

struct MarchParams {
    float camera_up{};
    float reach{};
    Vec3 sun;
    bool shadows{};
};

// The first surface along ray (east-north-up, unit length) from the camera.
// start skips empty space when a lower bound of the hit distance is known.
[[nodiscard]] Surface march(const Scene& scene, const MarchParams& p, const Vec3& ray,
                            float start = 0.5F);

// Heights at one point: terrain alone and terrain plus buildings.
[[nodiscard]] float terrain_at(const Relief& relief, float east, float north);
[[nodiscard]] float building_at(const BuildingView& b, float east, float north);

struct ShadeParams {
    float camera_up;
    float reach;
    float anchor_e;
    float anchor_n;
    float pixel_angle;
    // Ground size of one tile pixel at zoom 0.
    float texel0;
    std::span<const LevelView> levels;
    std::span<const StyleRow> styles;
    const materials::Atlas* atlas;
    Vec3 sun;
    shading::Light light;
    shading::Sky sky;
    shading::Exposure exposure;
    // Noise sigma after supersampling compensation; 0 turns the noise off.
    float sigma;
    std::uint32_t seed;
    int width;
    int height;
};

// Colour of pixel i (row-major index into the frame of width * height).
[[nodiscard]] std::array<std::uint8_t, 3> shade(const ShadeParams& p, int i, const Vec3& ray,
                                                const Surface& s);

// Average each s x s block of src (src_width wide) into one pixel of dst.
void downsample(std::span<const std::uint8_t> src, int src_width, int s,
                std::span<std::uint8_t> dst, int out_width, int out_height);

}  // namespace docksim::render::cpu
