#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

// Triangle meshes for the ray tracer. Positions are float32 metres east, north
// and up of a fixed origin: the home point and its ground height z0. Every
// triangle carries a record of record_size floats that render.slang reads at a hit.
namespace docksim::render::rt {

inline constexpr std::size_t record_size = 12;
// Record columns.
inline constexpr std::size_t record_kind = 0;
inline constexpr std::size_t record_style = 1;
// Face normal x, y, z in columns 2, 3, 4.
inline constexpr std::size_t record_normal = 2;
// Wall pattern offset along the wall.
inline constexpr std::size_t record_along = 5;
// Wall foot and eaves, metres up.
inline constexpr std::size_t record_base = 6;
inline constexpr std::size_t record_top = 7;
// Terrain: x, y of the three vertex normals as two float16 in each float32,
// columns 8, 9, 10. The shader rebuilds z, so they must point up.
inline constexpr std::size_t record_vertex_normals = 8;
// Wall: ground floor feature in the columns of the terrain normals. Kind
// (shading::feature_door or feature_shop, 0 none) and its span along the wall.
inline constexpr std::size_t record_feature = 8;
inline constexpr std::size_t record_feature_from = 9;
inline constexpr std::size_t record_feature_to = 10;
// Wall: storey height from building:levels, metres; 0 keeps the window layout floor.
inline constexpr std::size_t record_storey = 11;

inline constexpr float kind_terrain = 0.0F;
inline constexpr float kind_wall = 1.0F;
inline constexpr float kind_roof = 2.0F;
// Balcony and similar parts: plain colour in the feature columns 8, 9, 10 (0..255).
inline constexpr float kind_detail = 3.0F;

using Vertex = std::array<float, 3>;
using Triangle = std::array<std::uint32_t, 3>;
using Record = std::array<float, record_size>;

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<Triangle> indices;
    // One per triangle.
    std::vector<Record> records;

    [[nodiscard]] std::size_t triangles() const noexcept { return indices.size(); }
    [[nodiscard]] bool empty() const noexcept { return indices.empty(); }

    // Append other, with its indices moved past this mesh's vertices.
    void append(const Mesh& other);
};

}  // namespace docksim::render::rt
