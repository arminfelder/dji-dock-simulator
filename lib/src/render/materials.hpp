#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <vector>

// Photo wall materials with mip levels, for the facades. A pixel gets the
// texel colour divided by the material mean, times the facade colour; at a
// distance the factor tends to 1, and the wall keeps the colour of its style.
namespace docksim::render::materials {

inline constexpr int size = 512;
inline constexpr int levels = 8;
// Texels of all mip levels of one material.
inline constexpr int block = [] {
    int sum = 0;
    for (int k = 0; k < levels; ++k) sum += (size >> k) * (size >> k);
    return sum;
}();

// The order is the material index that facade styles refer to.
inline constexpr std::array<std::string_view, 8> keys{
    "plaster_old", "plaster", "concrete", "brick", "stone", "metal", "wood", "roof_tiles"};

struct Info {
    float size_m;
    float mean_r;
    float mean_g;
    float mean_b;
};

struct Atlas {
    // materials * block RGB8 texels: the mip levels of each material, level 0 first.
    std::vector<std::uint8_t> texels;
    std::vector<Info> info;
};

// Diffuse colour maps. Built once from the embedded JPEGs.
[[nodiscard]] const Atlas& atlas();
// Tangent-space normal maps (OpenGL convention), same layout. info means are unused.
[[nodiscard]] const Atlas& normal_atlas();

}  // namespace docksim::render::materials
