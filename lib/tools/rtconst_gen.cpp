// Writes the rtconst Slang module: the C++ constants that render.slang reads.
// One source of truth for the CPU kernels and the ray tracing shaders.

#include "render/cpu/kernel.hpp"
#include "render/materials.hpp"
#include "render/shading.hpp"

#include <algorithm>
#include <format>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>

namespace {

using namespace docksim::render;

// Mesh record kinds; render/rt/mesh.hpp has the same values.
constexpr float kind_terrain = 0.0F;
constexpr float kind_wall = 1.0F;
constexpr float kind_roof = 2.0F;
constexpr float kind_detail = 3.0F;

std::string f(std::string_view name, float v) {
    return std::format("export static const float {} = {:.9g};\n", name, v);
}

std::string f3(std::string_view name, const Vec3& v) {
    return std::format("export static const float3 {} = float3({:.9g}, {:.9g}, {:.9g});\n", name,
                       v.x, v.y, v.z);
}

std::string i(std::string_view name, int v) {
    return std::format("export static const int {} = {};\n", name, v);
}

int material(std::string_view key) {
    return static_cast<int>(std::ranges::find(materials::keys, key) - materials::keys.begin());
}

}  // namespace

int main(int argc, char** argv) {
    namespace s = shading;
    std::string out;
    out += f("STREAK_M", s::streak_m) + f("STREAK_MEAN", s::streak_mean) +
           f("STREAK_DARK", s::streak_dark) +
           f("STREAK_HALF_MIN", s::streak_half_min) + f("STREAK_HALF_MAX", s::streak_half_max) +
           f("STREAK_LENGTH_MIN", s::streak_length_min) +
           f("STREAK_LENGTH_MEAN", s::streak_length_mean);
    out += f("BLOTCH_M", s::blotch_m) + f("BLOTCH_MEAN", s::blotch_mean) +
           f("BLOTCH_DARK", s::blotch_dark);
    out += f("SPLASH_M", s::splash_m) + f("SPLASH_DARK", s::splash_dark);
    out += f("CURTAIN_SHARE", s::curtain_share) + f("CURTAIN_MIX", s::curtain_mix);
    out += f("ROOM_DARK", s::room_dark) + f("ROOM_SPREAD", s::room_spread) +
           f("REFLECT_MIN", s::reflect_min);
    out += f("REVEAL_M", s::reveal_m) + f("REVEAL_TOP", s::reveal_top) +
           f("REVEAL_SIDE", s::reveal_side);
    out += f("TRIM_PIXELS", s::trim_pixels) + f("GROUND_SHADE", s::ground_shade);
    out += f("GROOVE_M", s::groove_m) + f("GROOVE_LINE_M", s::groove_line_m) +
           f("GROOVE_DARK", s::groove_dark);
    out += f("CORNICE_M", s::cornice_m) + f("CORNICE_MIX", s::cornice_mix);
    out += f("LEDGE_M", s::ledge_m) + f("LEDGE_DARK", s::ledge_dark) + f("FRAME_M", s::frame_m);
    out += f("OVER_M", s::over_m) + f("SILL_M", s::sill_m) + f("SILL_SHADOW_M", s::sill_shadow_m);
    out += f("SILL_DARK", s::sill_dark) + f("HOOD_M", s::hood_m) + f("TRANSOM", s::transom);
    out += f("MULLION_M", s::mullion_m) + f("WINDOW_BOTTOM_M", s::window_bottom_m);
    out += f("FEATURE_DOOR", s::feature_door) + f("FEATURE_SHOP", s::feature_shop);
    out += f("DOOR_HEIGHT_M", s::door_height_m) + f("DOOR_DEPTH_M", s::door_depth_m) +
           f3("DOOR_LEAF", s::door_leaf);
    out += f("SHOP_PILLAR_M", s::shop_pillar_m) + f("SHOP_SILL_M", s::shop_sill_m);
    out += f("SHOP_SIGN_M", s::shop_sign_m) + f("SHOP_DEPTH_M", s::shop_depth_m);
    out += f("WINDOW_TOP_GAP_M", s::window_top_gap_m) + f("WINDOW_FADE_LO", s::window_fade_lo);
    out +=
        f("WINDOW_FADE_HI", s::window_fade_hi) + f("AMBIENT", s::ambient) + f("DIRECT", s::direct);
    out += f("HAZE_DISTANCE_M", s::haze_distance_m) + f("HAZE_MAX", s::haze_max);
    out += f("VIGNETTE_STRENGTH", s::vignette_strength) + f("REVEAL_SHADE", s::reveal_shade);
    out += f("AO_RANGE_M", s::ao_range_m) + f("AO_STRENGTH", s::ao_strength) +
           f("AO_FADE_M", s::ao_fade_m);
    out += f("AO_MAX_M", s::ao_max_m) + f("ROOF_DETAIL_NEAR_M", s::roof_detail_near_m);
    out += f("ROOF_DETAIL_FAR_M", s::roof_detail_far_m) + f("ROOF_DETAIL", s::roof_detail);
    out += f("TILE_NOISE", s::tile_noise) + f("TILE_VARIANTS", s::tile_variants);
    out += f("HEX_CELL", s::hex_cell) + f("HEX_SHARPNESS", s::hex_sharpness) +
           f("HEX_CONTRAST", s::hex_contrast);
    out += f("CORNICE_DEPTH_M", s::cornice_depth_m) + f("EAVES_M", s::eaves_m) +
           f("EAVES_DEPTH_M", s::eaves_depth_m);
    out += f("REFLECT_RANGE_M", s::reflect_range_m) +
           f("REFLECT_MAX_FOOTPRINT_M", s::reflect_max_footprint_m);
    out += f("BUMP_NEAR_M", s::bump_near_m) + f("BUMP_FAR_M", s::bump_far_m) +
           f("BUMP_STRENGTH", s::bump_strength);
    out += f("MIN_COS", cpu::min_cos) + f("SHADOW_MAX_M", cpu::shadow_max_m);
    out += f("SHADOW_RANGE_M", cpu::shadow_range_m) + f("SHADOW_LIFT_M", cpu::shadow_lift_m);
    out += f("KIND_WALL", kind_wall) + f("KIND_TERRAIN", kind_terrain) + f("KIND_ROOF", kind_roof);
    out += f("KIND_DETAIL", kind_detail);
    out += f3("GLASS", s::glass) + f3("CURTAIN", s::curtain) + f3("TRIM", s::trim) +
           f3("NO_DATA", s::no_data);
    out += i("MAT_SIZE", materials::size) + i("MAT_LEVELS", materials::levels) +
           i("MAT_BLOCK", materials::block);
    out += i("WINDOWS_CLASSIC", s::windows_classic) + i("WINDOWS_HISTORIC", s::windows_historic);
    out += i("LAYOUT_COLUMNS", 7) + i("AO_RAYS", s::ao_rays);
    out += i("MAT_ROOF_TILES", material("roof_tiles")) + i("MAT_CONCRETE", material("concrete"));
    out += i("MAT_PLASTER_OLD", material("plaster_old")) + i("MAT_PLASTER", material("plaster"));
    out += i("HEX_LOWPASS_LEVEL", s::hex_lowpass_level);
    out += f("MACRO_M", s::macro_m) + f("MACRO_FINE_M", s::macro_fine_m) +
           f("MACRO_STRENGTH", s::macro_strength);
    out += f("DISTANCE_SCALE", s::distance_scale) + f("DISTANCE_NEAR_M", s::distance_near_m) +
           f("DISTANCE_FAR_M", s::distance_far_m);
    out += f("DISTANCE_LOD_BIAS", s::distance_lod_bias);

    if (argc < 2) {
        std::cout << out;
        return 0;
    }
    std::ofstream file(argv[1]);
    file << out;
    return file ? 0 : 1;
}
