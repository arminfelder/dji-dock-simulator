#pragma once

#include "render/vec.hpp"

#include <array>

// Light model, facade detail, sky, haze and camera response. The CPU kernels
// and the ray tracing shader (through a generated constants file) read these.
namespace docksim::render::shading {

// Light model relative to flat ground: ambient plus direct sun.
inline constexpr float ambient = 0.45F;
inline constexpr float direct = 0.55F;
inline constexpr Vec3 glass{64.0F, 74.0F, 86.0F};

// Window layout per facade Windows value: floor height and bay width in
// metres, the glass cell as fractions of the floor (lo, hi) and of the bay
// (lo, hi), and the depth of the window recess in metres (the ray tracer draws it).
struct WindowLayout {
    float floor_m;
    float bay_m;
    float floor_lo;
    float floor_hi;
    float bay_lo;
    float bay_hi;
    float recess_m;
};
inline constexpr std::array<WindowLayout, 7> window_layouts{{
    {.floor_m = 3.0F,
     .bay_m = 2.6F,
     .floor_lo = 0.0F,
     .floor_hi = 0.0F,
     .bay_lo = 0.0F,
     .bay_hi = 0.0F,
     .recess_m = 0.0F},
    {.floor_m = 3.0F,
     .bay_m = 2.6F,
     .floor_lo = 0.35F,
     .floor_hi = 0.8F,
     .bay_lo = 0.3F,
     .bay_hi = 0.75F,
     .recess_m = 0.15F},
    {.floor_m = 3.5F,
     .bay_m = 1.5F,
     .floor_lo = 0.3F,
     .floor_hi = 0.8F,
     .bay_lo = 0.06F,
     .bay_hi = 1.0F,
     .recess_m = 0.08F},
    {.floor_m = 3.5F,
     .bay_m = 1.5F,
     .floor_lo = 0.05F,
     .floor_hi = 0.97F,
     .bay_lo = 0.04F,
     .bay_hi = 1.0F,
     .recess_m = 0.02F},
    {.floor_m = 6.0F,
     .bay_m = 6.0F,
     .floor_lo = 0.65F,
     .floor_hi = 0.85F,
     .bay_lo = 0.1F,
     .bay_hi = 0.9F,
     .recess_m = 0.1F},
    {.floor_m = 4.0F,
     .bay_m = 3.2F,
     .floor_lo = 0.25F,
     .floor_hi = 0.8F,
     .bay_lo = 0.38F,
     .bay_hi = 0.62F,
     .recess_m = 0.3F},
    {.floor_m = 3.8F,
     .bay_m = 2.9F,
     .floor_lo = 0.28F,
     .floor_hi = 0.78F,
     .bay_lo = 0.34F,
     .bay_hi = 0.66F,
     .recess_m = 0.25F},
}};
// Layout indices (osm facade Windows values) with extra detail.
inline constexpr int windows_historic = 5;
// 19th century city block: stucco trim, cornices, ground floor grooves.
inline constexpr int windows_classic = 6;

// Ray tracer only. Plaster inside a window recess gets less sky light.
inline constexpr float reveal_shade = 0.85F;
// Ambient occlusion: ao_rays rays up to ao_range_m into the half-space above a
// surface; the blocked share, weighted by nearness, dims the sky light by up to
// ao_strength. It fades out from ao_fade_m to ao_max_m.
inline constexpr int ao_rays = 6;
inline constexpr float ao_range_m = 10.0F;
inline constexpr float ao_strength = 0.85F;
inline constexpr float ao_fade_m = 600.0F;
inline constexpr float ao_max_m = 1000.0F;
// Below roof_detail_far_m per pixel a roof material adds its structure to the
// photo brightness, fully below roof_detail_near_m.
inline constexpr float roof_detail_near_m = 0.02F;
inline constexpr float roof_detail_far_m = 0.06F;
inline constexpr float roof_detail = 0.8F;
// Stochastic tiling of materials against a visible repeat grid.
inline constexpr float tile_noise = 0.35F;
inline constexpr float tile_variants = 8.0F;
// Hex-tiling of isotropic materials (Mikkelsen 2022): triangle grid edge in tiles,
// exponent that sharpens the blend, weight of texel brightness in the blend.
inline constexpr float hex_cell = 0.5F;
inline constexpr float hex_sharpness = 7.0F;
inline constexpr float hex_contrast = 0.6F;
// Hex-tiling keeps only texture detail finer than this mip level (5: 16 texels per tile).
inline constexpr int hex_lowpass_level = 5;
// Macro variation of wall brightness: two octaves of world-space noise, metres,
// and the largest change as a share of the colour.
inline constexpr float macro_m = 25.0F;
inline constexpr float macro_fine_m = 8.0F;
inline constexpr float macro_strength = 0.25F;
// Distance blend of isotropic wall materials: a second sample at distance_scale
// times the tile size takes over between these footprints per pixel, metres.
inline constexpr float distance_scale = 8.0F;
inline constexpr float distance_near_m = 0.03F;
inline constexpr float distance_far_m = 0.12F;
// Mip levels added to the coarse sample.
inline constexpr float distance_lod_bias = 2.0F;
// 3D cornices of classic facades: floor bands stick out cornice_depth_m, the
// eaves cornice eaves_m high, eaves_depth_m out.
inline constexpr float cornice_depth_m = 0.15F;
inline constexpr float eaves_m = 0.6F;
inline constexpr float eaves_depth_m = 0.45F;
// Window reflections: traced up to reflect_range_m, and only where a pixel is
// smaller than reflect_max_footprint_m.
inline constexpr float reflect_range_m = 600.0F;
inline constexpr float reflect_max_footprint_m = 1.5F;
// Material normal maps: full relief below bump_near_m per pixel, none above bump_far_m.
inline constexpr float bump_near_m = 0.01F;
inline constexpr float bump_far_m = 0.05F;
// The photo normal maps are shallow; their tilt is scaled by this, so raking sun shows the surface.
inline constexpr float bump_strength = 2.5F;
// Ground outside the imagery coverage.
inline constexpr Vec3 no_data{96.0F, 104.0F, 84.0F};

// Windows start this high above the ground, metres.
inline constexpr float window_bottom_m = 1.2F;
// Ground floor features from OSM (osm/features.hpp), kind codes in the wall record.
inline constexpr float feature_door = 1.0F;
inline constexpr float feature_shop = 2.0F;
// Door: width, height and depth of the opening, metres; leaf colour.
inline constexpr float door_width_m = 1.4F;
inline constexpr float door_height_m = 2.6F;
inline constexpr float door_depth_m = 0.3F;
inline constexpr Vec3 door_leaf{82.0F, 60.0F, 44.0F};
// Shopfront per bay: pillars at each side, sill, sign band under the floor line,
// glass depth, metres. Ground floor windows give way to it.
inline constexpr float shop_pillar_m = 0.3F;
inline constexpr float shop_sill_m = 0.45F;
inline constexpr float shop_sign_m = 0.6F;
inline constexpr float shop_depth_m = 0.15F;
// A door must be this close to a wall, metres; entrance nodes sit on the outline.
inline constexpr float entrance_snap_m = 1.0F;
// A shopfront needs a wall at least this long, metres.
inline constexpr float shop_wall_m = 3.0F;
// Weathering darkens the wall by at most these shares: rain streaks, stains of
// blotch_m size, splash dirt up to splash_m high.
// Each column of streak_m holds at most one streak. Its half width is a share of the
// column. It starts at the wall top and runs down a share of the wall height.
inline constexpr float streak_m = 0.5F;
inline constexpr float streak_dark = 0.4F;
inline constexpr float streak_half_min = 0.08F;
inline constexpr float streak_half_max = 0.32F;
inline constexpr float streak_length_min = 0.15F;
inline constexpr float blotch_m = 3.0F;
inline constexpr float blotch_dark = 0.25F;
inline constexpr float splash_m = 1.0F;
inline constexpr float splash_dark = 0.15F;
// Shown at a distance: mean streak cover of a column (strength hash squared, 1/3,
// times the profile area of one half width), mean streak length, mean stain (value noise).
inline constexpr float streak_mean = (streak_half_min + streak_half_max) / 2.0F / 3.0F;
inline constexpr float streak_length_mean = (streak_length_min + 1.0F) / 2.0F;
inline constexpr float blotch_mean = 0.5F;
// Classic facade detail, metres: frame around each window, the window cross
// (transom at that share of the window height), cornice band per floor,
// grooves of the ground floor.
inline constexpr Vec3 trim{236.0F, 232.0F, 220.0F};
inline constexpr float frame_m = 0.14F;
inline constexpr float mullion_m = 0.11F;
inline constexpr float transom = 0.7F;
inline constexpr float cornice_m = 0.3F;
inline constexpr float cornice_mix = 0.7F;
inline constexpr float ledge_m = 0.12F;
inline constexpr float ledge_dark = 0.35F;
// Thin trim shows from this many pixels per trim width, fully from twice that.
inline constexpr float trim_pixels = 0.7F;
inline constexpr float ground_shade = 0.9F;
inline constexpr float groove_m = 0.5F;
inline constexpr float groove_line_m = 0.06F;
inline constexpr float groove_dark = 0.25F;
// Window sill and hood, metres: height, overhang to each side of the frame,
// shadow under the sill.
inline constexpr float sill_m = 0.08F;
inline constexpr float hood_m = 0.22F;
inline constexpr float over_m = 0.12F;
inline constexpr float sill_shadow_m = 0.1F;
inline constexpr float sill_dark = 0.45F;
// Window glass: room brightness, curtains, Schlick sky reflection, reveal shadow.
inline constexpr float room_dark = 0.5F;
inline constexpr float room_spread = 0.8F;
inline constexpr Vec3 curtain{196.0F, 186.0F, 166.0F};
inline constexpr float curtain_share = 0.7F;
inline constexpr float curtain_mix = 0.55F;
inline constexpr float reflect_min = 0.12F;
inline constexpr float reveal_m = 0.12F;
inline constexpr float reveal_top = 0.5F;
inline constexpr float reveal_side = 0.7F;
// Band limit: below fade_lo pixels per floor the facade is its average colour,
// above fade_hi the window grid is fully drawn. Stops flicker at a distance.
inline constexpr float window_fade_lo = 1.5F;
inline constexpr float window_fade_hi = 3.0F;
// A pixel is wall when its surface is steeper than wall_normal_z and it lies
// roof_band_m below the roof line. On the 0.5 m grid a roof edge is a steep
// ramp; without the height test those pixels would speckle the roof outline.
inline constexpr float roof_band_m = 0.2F;
inline constexpr float wall_normal_z = 0.55F;
inline constexpr float window_top_gap_m = 0.8F;
inline constexpr float min_building_m = 0.3F;
// Haze reaches 1 - 1/e at this distance, and at most haze_max.
inline constexpr float haze_distance_m = 9000.0F;
inline constexpr float haze_max = 0.85F;
inline constexpr float vignette_strength = 0.25F;

struct Light {
    // 0 at night, 1 with the sun 10 degrees up. Scales the slope and shadow effect.
    float day;
    // Light on flat sunlit ground, the reference that keeps the orthophoto colour.
    float flat;
};

struct Sky {
    Vec3 horizon;
    Vec3 zenith;
    // 1 with the sun at the horizon: sunset tint and glow.
    float low{};
    float sin_az{};
    float cos_az{};
};

struct Exposure {
    // Exposure times white balance, per channel.
    Vec3 gain;
    // Sensor noise sigma in 8-bit levels.
    float sigma{};
};

[[nodiscard]] Light light_params(double sun_el, const Vec3& sun);
[[nodiscard]] Sky sky_params(double sun_el, double sun_az);
[[nodiscard]] Exposure exposure_params(double sun_el);

}  // namespace docksim::render::shading
