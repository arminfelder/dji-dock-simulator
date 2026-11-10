#pragma once

#include <cstddef>
#include <span>
#include <string_view>

// The SPIR-V and Slang reflection of each entry point of render.slang,
// compiled at build time (cmake/Shaders.cmake) and embedded.
namespace docksim::render::rt::shaders {

struct Shader {
    std::span<const std::byte> spirv;
    std::string_view reflection;
};

[[nodiscard]] Shader trace();
[[nodiscard]] Shader shade();
[[nodiscard]] Shader subsample();
[[nodiscard]] Shader downsample();

}  // namespace docksim::render::rt::shaders
