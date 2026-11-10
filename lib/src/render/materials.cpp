#include "render/materials.hpp"

#include "image/image.hpp"

#include <algorithm>
#include <cmath>
#include <span>
#include <stdexcept>

namespace docksim::render::materials {

namespace {

constexpr unsigned char plaster_old[] = {
#embed "../../resources/materials/painted_plaster_wall.jpg"
};
constexpr unsigned char plaster_old_nor[] = {
#embed "../../resources/materials/painted_plaster_wall_nor.jpg"
};
constexpr unsigned char plaster[] = {
#embed "../../resources/materials/plastered_wall_02.jpg"
};
constexpr unsigned char plaster_nor[] = {
#embed "../../resources/materials/plastered_wall_02_nor.jpg"
};
constexpr unsigned char concrete[] = {
#embed "../../resources/materials/concrete_wall_006.jpg"
};
constexpr unsigned char concrete_nor[] = {
#embed "../../resources/materials/concrete_wall_006_nor.jpg"
};
constexpr unsigned char brick[] = {
#embed "../../resources/materials/brick_wall_09.jpg"
};
constexpr unsigned char brick_nor[] = {
#embed "../../resources/materials/brick_wall_09_nor.jpg"
};
constexpr unsigned char stone[] = {
#embed "../../resources/materials/sandstone_blocks_04.jpg"
};
constexpr unsigned char stone_nor[] = {
#embed "../../resources/materials/sandstone_blocks_04_nor.jpg"
};
constexpr unsigned char metal[] = {
#embed "../../resources/materials/corrugated_iron_03.jpg"
};
constexpr unsigned char metal_nor[] = {
#embed "../../resources/materials/corrugated_iron_03_nor.jpg"
};
constexpr unsigned char wood[] = {
#embed "../../resources/materials/weathered_plank_siding.jpg"
};
constexpr unsigned char wood_nor[] = {
#embed "../../resources/materials/weathered_plank_siding_nor.jpg"
};
constexpr unsigned char roof_tiles[] = {
#embed "../../resources/materials/clay_roof_tiles_02.jpg"
};
constexpr unsigned char roof_tiles_nor[] = {
#embed "../../resources/materials/clay_roof_tiles_02_nor.jpg"
};

struct Source {
    std::span<const unsigned char> diffuse;
    std::span<const unsigned char> normal;
    // Tile size in metres. See resources/materials/SOURCES.md.
    float size_m;
};

const std::array<Source, keys.size()> sources{{
    {.diffuse = plaster_old, .normal = plaster_old_nor, .size_m = 2.0F},
    {.diffuse = plaster, .normal = plaster_nor, .size_m = 2.23F},
    {.diffuse = concrete, .normal = concrete_nor, .size_m = 2.0F},
    {.diffuse = brick, .normal = brick_nor, .size_m = 2.01F},
    {.diffuse = stone, .normal = stone_nor, .size_m = 3.0F},
    {.diffuse = metal, .normal = metal_nor, .size_m = 2.0F},
    {.diffuse = wood, .normal = wood_nor, .size_m = 1.57F},
    {.diffuse = roof_tiles, .normal = roof_tiles_nor, .size_m = 2.5F},
}};

Atlas build(bool normals) {
    Atlas out;
    out.texels.reserve(static_cast<std::size_t>(block) * keys.size() * 3);
    for (const auto& source : sources) {
        const auto bytes = normals ? source.normal : source.diffuse;
        const auto img = image::decode(std::as_bytes(bytes));
        if (img.width() != size || img.height() != size) {
            throw std::runtime_error("material texture must be 512 x 512");
        }
        std::vector<float> level(img.bytes().begin(), img.bytes().end());
        double mean[3] = {0, 0, 0};
        for (std::size_t i = 0; i < level.size(); ++i) mean[i % 3] += level[i];
        const double count = static_cast<double>(size) * size;
        for (int k = 0, n = size; k < levels; ++k, n /= 2) {
            for (const float v : level)
                out.texels.push_back(
                    static_cast<std::uint8_t>(std::clamp(std::rint(v), 0.0F, 255.0F)));
            if (n == 1) break;
            // Box filter to the next mip level.
            const int h = n / 2;
            std::vector<float> next(static_cast<std::size_t>(h) * static_cast<std::size_t>(h) * 3);
            for (int y = 0; y < h; ++y) {
                for (int x = 0; x < h; ++x) {
                    for (int c = 0; c < 3; ++c) {
                        const auto at = [&](int xx, int yy) {
                            return level[(static_cast<std::size_t>(yy) *
                                              static_cast<std::size_t>(n) +
                                          static_cast<std::size_t>(xx)) *
                                             3 +
                                         static_cast<std::size_t>(c)];
                        };
                        next[(static_cast<std::size_t>(y) * static_cast<std::size_t>(h) +
                              static_cast<std::size_t>(x)) *
                                 3 +
                             static_cast<std::size_t>(c)] =
                            (at(2 * x, 2 * y) + at(2 * x + 1, 2 * y) + at(2 * x, 2 * y + 1) +
                             at(2 * x + 1, 2 * y + 1)) /
                            4.0F;
                    }
                }
            }
            level = std::move(next);
        }
        out.info.push_back({source.size_m, static_cast<float>(mean[0] / count),
                            static_cast<float>(mean[1] / count),
                            static_cast<float>(mean[2] / count)});
    }
    return out;
}

}  // namespace

const Atlas& atlas() {
    static const Atlas a = build(false);
    return a;
}

const Atlas& normal_atlas() {
    static const Atlas a = build(true);
    return a;
}

}  // namespace docksim::render::materials
