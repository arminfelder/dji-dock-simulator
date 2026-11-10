// The Vulkan layer on any device with ray queries. On a machine without a GPU
// the tests use Mesa lavapipe; without any usable device they skip.

#include "render/rt/shaders.hpp"
#include "render/rt/vk.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>

using namespace docksim::render;
using namespace docksim::render::rt;
using Catch::Approx;

namespace {

std::shared_ptr<vk::Context> open_any() {
    constexpr const char* lavapipe = "/usr/share/vulkan/icd.d/lvp_icd.json";
    if (std::getenv("VK_ICD_FILENAMES") == nullptr && std::filesystem::exists(lavapipe) &&
        std::getenv("DOCKSIM_TEST_GPU") == nullptr) {
        ::setenv("VK_ICD_FILENAMES", lavapipe, 1);
    }
    static const std::shared_ptr<vk::Context> ctx = [] {
        try {
            return vk::Context::open(std::nullopt, true);
        } catch (const vk::NoDevice&) {
            return std::shared_ptr<vk::Context>{};
        }
    }();
    return ctx;
}

template <class T>
std::span<const std::byte> bytes(const std::vector<T>& v) {
    return std::as_bytes(std::span(v));
}

}  // namespace

TEST_CASE("a ray hits one triangle through BLAS, TLAS and the trace shader", "[rt]") {
    auto ctx = open_any();
    if (!ctx) SKIP("no Vulkan device with ray queries");

    // A large triangle 10 m below the camera, in the ground plane.
    const std::vector<float> vertices{-100, -100, 0, 100, -100, 0, 0, 100, 0};
    const std::vector<std::uint32_t> indices{0, 1, 2};
    const auto blas = ctx->build_blas(vertices, indices);
    const std::array instances{vk::Instance{.blas = &blas, .custom_index = 7}};
    const auto tlas = ctx->build_tlas(instances);

    // Two rays in camera axes: straight ahead (forward = down here) and one that misses.
    const std::vector<float> base{0, 0, 1, 0, -1, 0};
    const auto base_buf = ctx->storage(bytes(base));
    const vk::Buffer out_t(*ctx, 2 * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, vk::Memory::device);
    const vk::Buffer out_id(*ctx, 2 * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, vk::Memory::device);
    const vk::Buffer out_bary(*ctx, 4 * 4, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, vk::Memory::device);

    const auto shader = shaders::trace();
    const vk::Program trace(*ctx, shader.spirv, shader.reflection);
    vk::Program::Call call(trace);
    // Rows map camera right, down, forward to east-north-up: forward points down.
    call.bind("tlas", tlas)
        .bind("base", base_buf)
        .bind("out_t", out_t)
        .bind("out_id", out_id)
        .bind("out_bary", out_bary)
        .push("rot0", Vec3{1, 0, 0})
        .push("rot1", Vec3{0, 1, 0})
        .push("rot2", Vec3{0, 0, -1})
        .push("cam", Vec3{0, 0, 10})
        .push("reach", 1000.0F)
        .push("count", std::uint32_t{2});
    ctx->run([&](VkCommandBuffer cmd) { trace.dispatch(cmd, call, 2); });

    std::array<float, 2> t{};
    std::array<std::uint32_t, 2> id{};
    ctx->download(out_t, std::as_writable_bytes(std::span(t)));
    ctx->download(out_id, std::as_writable_bytes(std::span(id)));
    CHECK(t[0] == Approx(10.0F));
    CHECK(id[0] == 7U);
    // The second ray runs along the plane at height 10 and misses.
    CHECK(t[1] == -1.0F);
    CHECK(id[1] == 0xFFFFFFFFU);
}

TEST_CASE("reflection gives the push constants and bindings of every kernel", "[rt]") {
    auto ctx = open_any();
    if (!ctx) SKIP("no Vulkan device with ray queries");
    for (const auto& shader :
         {shaders::trace(), shaders::shade(), shaders::subsample(), shaders::downsample()}) {
        CHECK_NOTHROW(vk::Program(*ctx, shader.spirv, shader.reflection));
    }
}

TEST_CASE("a value whose size differs from the shader field is refused", "[rt]") {
    auto ctx = open_any();
    if (!ctx) SKIP("no Vulkan device with ray queries");
    const auto shader = shaders::trace();
    const vk::Program trace(*ctx, shader.spirv, shader.reflection);
    vk::Program::Call call(trace);
    CHECK_NOTHROW(call.push("reach", 1000.0F));
    CHECK_THROWS_AS(call.push("reach", Vec3{1, 2, 3}), std::invalid_argument);
    CHECK_THROWS_AS(call.push("cam", 1.0F), std::invalid_argument);
    CHECK_THROWS_AS(call.push("no_such_field", 1.0F), std::invalid_argument);
}

TEST_CASE("copies past the end of a buffer are refused", "[rt]") {
    auto ctx = open_any();
    if (!ctx) SKIP("no Vulkan device with ray queries");
    vk::Buffer device(*ctx, 16, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, vk::Memory::device);
    vk::Buffer mapped(*ctx, 16, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, vk::Memory::upload);
    const std::array<std::byte, 8> data{};
    std::array<std::byte, 8> out{};
    CHECK_NOTHROW(ctx->upload(device, data, 8));
    CHECK_THROWS_AS(ctx->upload(device, data, 9), std::out_of_range);
    CHECK_THROWS_AS(ctx->upload(mapped, data, 12), std::out_of_range);
    CHECK_NOTHROW(ctx->download(device, out, 8));
    CHECK_THROWS_AS(ctx->download(device, out, 9), std::out_of_range);

    const vk::Buffer moved = std::move(device);
    CHECK(moved.handle() != VK_NULL_HANDLE);
    // NOLINTNEXTLINE(bugprone-use-after-move): the test checks the moved-from state.
    CHECK(device.handle() == VK_NULL_HANDLE);
}
