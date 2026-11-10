#include "render/rt/shaders.hpp"

// The files come from the build tree; the target passes --embed-dir for them.
namespace docksim::render::rt::shaders {

namespace {

alignas(4) constexpr unsigned char trace_spv[] = {
#embed "trace.spv"
};
constexpr char trace_json[] = {
#embed "trace.json"
};
alignas(4) constexpr unsigned char shade_spv[] = {
#embed "shade.spv"
};
constexpr char shade_json[] = {
#embed "shade.json"
};
alignas(4) constexpr unsigned char subsample_spv[] = {
#embed "subsample.spv"
};
constexpr char subsample_json[] = {
#embed "subsample.json"
};
alignas(4) constexpr unsigned char downsample_spv[] = {
#embed "downsample.spv"
};
constexpr char downsample_json[] = {
#embed "downsample.json"
};

Shader make(std::span<const unsigned char> spirv, std::span<const char> json) {
    return {.spirv = std::as_bytes(spirv),
            .reflection = std::string_view(json.data(), json.size())};
}

}  // namespace

Shader trace() {
    return make(trace_spv, trace_json);
}
Shader shade() {
    return make(shade_spv, shade_json);
}
Shader subsample() {
    return make(subsample_spv, subsample_json);
}
Shader downsample() {
    return make(downsample_spv, downsample_json);
}

}  // namespace docksim::render::rt::shaders
