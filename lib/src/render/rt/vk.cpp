#include "render/rt/vk.hpp"

#include "log/log.hpp"
#include "util/json.hpp"

// VMA and Vulkan-Hpp take their configuration as macros.
// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#define VMA_STATIC_VULKAN_FUNCTIONS 0
#define VMA_DYNAMIC_VULKAN_FUNCTIONS 1
#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>

// volk loads the C entry points. The RAII classes load their own through vk::raii::Context.
#define VULKAN_HPP_DISPATCH_LOADER_DYNAMIC 1
// vk::raii::Context takes the vkGetInstanceProcAddr of volk; Vulkan-Hpp must not open the library
// itself.
#define VULKAN_HPP_ENABLE_DYNAMIC_LOADER_TOOL 0
// NOLINTEND(cppcoreguidelines-macro-usage)
#define VULKAN_HPP_NO_STRUCT_CONSTRUCTORS
#include <vulkan/vulkan_raii.hpp>

VULKAN_HPP_DEFAULT_DISPATCH_LOADER_DYNAMIC_STORAGE

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <format>

namespace docksim::render::rt::vk {

namespace {

void check(VkResult r, std::string_view what) {
    if (r != VK_SUCCESS)
        throw std::runtime_error(std::format("{} failed: VkResult {}", what, static_cast<int>(r)));
}

// Scratch addresses of an acceleration structure build must be aligned to this.
constexpr VkDeviceAddress scratch_align = 256;

std::string lower(std::string_view s) {
    std::string out(s);
    std::ranges::transform(out, out.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

constexpr std::array required_extensions{
    VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
    VK_KHR_RAY_QUERY_EXTENSION_NAME,
    VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
    VK_KHR_PUSH_DESCRIPTOR_EXTENSION_NAME,
};

bool has_extensions(const ::vk::raii::PhysicalDevice& device) {
    const auto props = device.enumerateDeviceExtensionProperties();
    return std::ranges::all_of(required_extensions, [&](const char* name) {
        return std::ranges::any_of(
            props, [&](const auto& p) { return std::strcmp(p.extensionName, name) == 0; });
    });
}

// Lower is better.
enum class Rank : std::uint8_t { discrete, integrated, other, software };

Rank rank(::vk::PhysicalDeviceType type) {
    switch (type) {
        case ::vk::PhysicalDeviceType::eDiscreteGpu:
            return Rank::discrete;
        case ::vk::PhysicalDeviceType::eIntegratedGpu:
            return Rank::integrated;
        case ::vk::PhysicalDeviceType::eCpu:
            return Rank::software;
        default:
            return Rank::other;
    }
}

VkTransformMatrixKHR identity() {
    VkTransformMatrixKHR m{};
    m.matrix[0][0] = 1.0F;
    m.matrix[1][1] = 1.0F;
    m.matrix[2][2] = 1.0F;
    return m;
}

// Name of the first OpEntryPoint in a SPIR-V module.
std::string entry_point_name(std::span<const std::byte> spirv) {
    const auto words = spirv.size() / 4;
    std::vector<std::uint32_t> w(words);
    std::memcpy(w.data(), spirv.data(), words * 4);
    for (std::size_t i = 5; i < words;) {
        const std::uint32_t op = w[i] & 0xFFFFU;
        const std::uint32_t count = w[i] >> 16U;
        if (count == 0) break;
        // OpEntryPoint: execution model, function id, then the name string.
        if (op == 15 && i + 3 < words) {
            const auto* text = reinterpret_cast<const char*>(&w[i + 3]);
            return {text, ::strnlen(text, std::size_t{count - 3} * 4)};
        }
        i += count;
    }
    throw std::runtime_error("SPIR-V module has no entry point");
}

}  // namespace

// -- Buffer -------------------------------------------------------------------

void BufferFree::operator()(VmaAllocation_T* allocation) const noexcept {
    vmaDestroyBuffer(allocator, buffer, allocation);
}

Buffer::Buffer(Context& ctx, std::size_t bytes, VkBufferUsageFlags usage, Memory memory)
    : ctx_(&ctx), size_(bytes) {
    VkBufferCreateInfo info{.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    info.size = std::max<std::size_t>(bytes, 16);
    info.usage = usage | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    VmaAllocationCreateInfo alloc{};
    alloc.usage = VMA_MEMORY_USAGE_AUTO;
    if (memory == Memory::upload) {
        alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                      VMA_ALLOCATION_CREATE_MAPPED_BIT;
    } else if (memory == Memory::readback) {
        alloc.flags =
            VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
    }
    VkBuffer buffer = VK_NULL_HANDLE;
    VmaAllocation allocation = nullptr;
    VmaAllocationInfo result{};
    check(vmaCreateBuffer(ctx.allocator(), &info, &alloc, &buffer, &allocation, &result),
          "vmaCreateBuffer");
    allocation_ = {allocation, BufferFree{.allocator = ctx.allocator(), .buffer = buffer}};
    mapped_ = static_cast<std::byte*>(result.pMappedData);
}

void Buffer::flush() const {
    if (allocation_) vmaFlushAllocation(ctx_->allocator(), allocation_.get(), 0, VK_WHOLE_SIZE);
}

void Buffer::invalidate() const {
    if (allocation_)
        vmaInvalidateAllocation(ctx_->allocator(), allocation_.get(), 0, VK_WHOLE_SIZE);
}

VkDeviceAddress Buffer::address() const {
    const VkBufferDeviceAddressInfo info{.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO,
                                         .buffer = handle()};
    return vkGetBufferDeviceAddress(ctx_->device(), &info);
}

// -- Accel --------------------------------------------------------------------

struct Accel::Handle {
    ::vk::raii::AccelerationStructureKHR accel;
};

Accel::Accel() = default;
Accel::Accel(Accel&& other) noexcept = default;
Accel& Accel::operator=(Accel&& other) noexcept = default;
Accel::~Accel() = default;

VkAccelerationStructureKHR Accel::handle() const noexcept {
    return handle_ ? static_cast<VkAccelerationStructureKHR>(*handle_->accel) : VK_NULL_HANDLE;
}

// -- Context ------------------------------------------------------------------

// Members are destroyed in reverse order: the fence and pool, then the allocator, the device, the
// instance.
struct Context::Impl {
    explicit Impl(PFN_vkGetInstanceProcAddr get_proc) : context(get_proc) {}

    struct AllocatorFree {
        void operator()(VmaAllocator_T* a) const noexcept { vmaDestroyAllocator(a); }
    };

    ::vk::raii::Context context;
    ::vk::raii::Instance instance{nullptr};
    ::vk::raii::PhysicalDevice physical{nullptr};
    ::vk::raii::Device device{nullptr};
    std::unique_ptr<VmaAllocator_T, AllocatorFree> allocator;
    ::vk::raii::Queue queue{nullptr};
    ::vk::raii::CommandPool pool{nullptr};
    ::vk::raii::Fence fence{nullptr};
};

Context::Context() = default;

std::shared_ptr<Context> Context::open(const std::optional<std::string>& name_filter,
                                       bool allow_cpu) {
    static std::once_flag loaded;
    static VkResult loader = VK_ERROR_INITIALIZATION_FAILED;
    std::call_once(loaded, [] { loader = volkInitialize(); });
    if (loader != VK_SUCCESS) throw NoDevice("no Vulkan loader (libvulkan.so.1)");

    std::shared_ptr<Context> ctx(new Context());
    ctx->impl_ = std::make_unique<Impl>(vkGetInstanceProcAddr);
    auto& m = *ctx->impl_;
    const ::vk::ApplicationInfo app{.pApplicationName = "dock-sim",
                                    .apiVersion = VK_API_VERSION_1_3};
    try {
        m.instance =
            ::vk::raii::Instance(m.context, ::vk::InstanceCreateInfo{.pApplicationInfo = &app});
    } catch (const ::vk::SystemError& e) {
        throw NoDevice(std::format("vkCreateInstance failed: {}", e.what()));
    }
    volkLoadInstanceOnly(*m.instance);

    std::optional<Rank> best;
    for (auto& d : m.instance.enumeratePhysicalDevices()) {
        const auto props = d.getProperties();
        const Rank r = rank(props.deviceType);
        if (r == Rank::software && !allow_cpu) continue;
        const std::string name = props.deviceName;
        if (name_filter && lower(name).find(lower(*name_filter)) == std::string::npos) continue;
        if (props.apiVersion < VK_API_VERSION_1_3 || !has_extensions(d)) continue;
        if (!best || r < *best) {
            best = r;
            m.physical = std::move(d);
            ctx->name_ = name;
            ctx->software_ = r == Rank::software;
        }
    }
    if (!best) {
        throw NoDevice(
            name_filter ? std::format("no Vulkan device with ray queries matching {}", *name_filter)
                        : std::string("no Vulkan device with ray queries"));
    }

    const auto families = m.physical.getQueueFamilyProperties();
    const auto family_it = std::ranges::find_if(families, [](const auto& f) {
        return static_cast<bool>(f.queueFlags & ::vk::QueueFlagBits::eCompute);
    });
    if (family_it == families.end())
        throw NoDevice(std::format("{} has no compute queue", ctx->name_));
    const auto family = static_cast<std::uint32_t>(family_it - families.begin());

    const float priority = 1.0F;
    const ::vk::DeviceQueueCreateInfo queue{
        .queueFamilyIndex = family, .queueCount = 1, .pQueuePriorities = &priority};
    const ::vk::StructureChain<::vk::DeviceCreateInfo, ::vk::PhysicalDeviceVulkan12Features,
                               ::vk::PhysicalDeviceAccelerationStructureFeaturesKHR,
                               ::vk::PhysicalDeviceRayQueryFeaturesKHR>
        chain{
            ::vk::DeviceCreateInfo{
                .queueCreateInfoCount = 1,
                .pQueueCreateInfos = &queue,
                .enabledExtensionCount = static_cast<std::uint32_t>(required_extensions.size()),
                .ppEnabledExtensionNames = required_extensions.data()},
            ::vk::PhysicalDeviceVulkan12Features{.scalarBlockLayout = VK_TRUE,
                                                 .bufferDeviceAddress = VK_TRUE},
            ::vk::PhysicalDeviceAccelerationStructureFeaturesKHR{.accelerationStructure = VK_TRUE},
            ::vk::PhysicalDeviceRayQueryFeaturesKHR{.rayQuery = VK_TRUE}};
    m.device = ::vk::raii::Device(m.physical, chain.get<::vk::DeviceCreateInfo>());
    volkLoadDevice(*m.device);
    m.queue = ::vk::raii::Queue(m.device, family, 0);

    VmaVulkanFunctions functions{};
    functions.vkGetInstanceProcAddr = vkGetInstanceProcAddr;
    functions.vkGetDeviceProcAddr = vkGetDeviceProcAddr;
    VmaAllocatorCreateInfo alloc{};
    alloc.flags = VMA_ALLOCATOR_CREATE_BUFFER_DEVICE_ADDRESS_BIT;
    alloc.physicalDevice = *m.physical;
    alloc.device = *m.device;
    alloc.instance = *m.instance;
    alloc.vulkanApiVersion = VK_API_VERSION_1_3;
    alloc.pVulkanFunctions = &functions;
    VmaAllocator allocator = nullptr;
    check(vmaCreateAllocator(&alloc, &allocator), "vmaCreateAllocator");
    m.allocator.reset(allocator);

    m.pool = ::vk::raii::CommandPool(
        m.device,
        ::vk::CommandPoolCreateInfo{.flags = ::vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
                                    .queueFamilyIndex = family});
    m.fence = ::vk::raii::Fence(m.device, ::vk::FenceCreateInfo{});
    log::get("dock_sim.render")->info("ray tracing on {}", ctx->name_);
    return ctx;
}

Context::~Context() {
    // vk::raii::Device::waitIdle throws on a lost device, and a destructor must not throw.
    // The C call only returns the error, which does not matter during teardown.
    if (impl_ && *impl_->device != VK_NULL_HANDLE) vkDeviceWaitIdle(*impl_->device);
}

VkDevice Context::device() const noexcept {
    return *impl_->device;
}

VmaAllocator_T* Context::allocator() const noexcept {
    return impl_->allocator.get();
}

void Context::run(const std::function<void(VkCommandBuffer)>& record) {
    const std::scoped_lock lock(mutex_);
    auto& m = *impl_;
    const ::vk::raii::CommandBuffer cmd = std::move(
        ::vk::raii::CommandBuffers(
            m.device, ::vk::CommandBufferAllocateInfo{.commandPool = *m.pool,
                                                      .level = ::vk::CommandBufferLevel::ePrimary,
                                                      .commandBufferCount = 1})
            .front());
    cmd.begin(
        ::vk::CommandBufferBeginInfo{.flags = ::vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
    record(static_cast<VkCommandBuffer>(*cmd));
    cmd.end();
    m.device.resetFences(*m.fence);
    const ::vk::CommandBuffer raw = *cmd;
    m.queue.submit(::vk::SubmitInfo{.commandBufferCount = 1, .pCommandBuffers = &raw}, *m.fence);
    if (m.device.waitForFences(*m.fence, VK_TRUE, UINT64_MAX) != ::vk::Result::eSuccess) {
        throw std::runtime_error("vkWaitForFences did not finish");
    }
}

void Context::upload(Buffer& dst, std::span<const std::byte> data, std::size_t offset) {
    if (data.empty()) return;
    if (offset > dst.size() || dst.size() - offset < data.size()) {
        throw std::out_of_range(std::format("upload of {} bytes at {} into a buffer of {}",
                                            data.size(), offset, dst.size()));
    }
    if (dst.mapped() != nullptr) {
        std::memcpy(dst.mapped() + offset, data.data(), data.size());
        dst.flush();
        return;
    }
    Buffer staging(*this, data.size(), VK_BUFFER_USAGE_TRANSFER_SRC_BIT, Memory::upload);
    if (staging.mapped() == nullptr)
        throw std::runtime_error("upload staging buffer is not mapped");
    std::memcpy(staging.mapped(), data.data(), data.size());
    staging.flush();
    run([&](VkCommandBuffer cmd) {
        const VkBufferCopy region{.srcOffset = 0, .dstOffset = offset, .size = data.size()};
        vkCmdCopyBuffer(cmd, staging.handle(), dst.handle(), 1, &region);
    });
}

void Context::download(const Buffer& src, std::span<std::byte> out, std::size_t offset) {
    if (out.empty()) return;
    if (offset > src.size() || src.size() - offset < out.size()) {
        throw std::out_of_range(std::format("download of {} bytes at {} from a buffer of {}",
                                            out.size(), offset, src.size()));
    }
    Buffer staging(*this, out.size(), VK_BUFFER_USAGE_TRANSFER_DST_BIT, Memory::readback);
    run([&](VkCommandBuffer cmd) {
        const VkBufferCopy region{.srcOffset = offset, .dstOffset = 0, .size = out.size()};
        vkCmdCopyBuffer(cmd, src.handle(), staging.handle(), 1, &region);
    });
    staging.invalidate();
    std::memcpy(out.data(), staging.mapped(), out.size());
}

void Context::copy(const Buffer& src, Buffer& dst, std::size_t bytes) {
    if (bytes > src.size() || bytes > dst.size())
        throw std::out_of_range("copy past the end of a buffer");
    run([&](VkCommandBuffer cmd) {
        const VkBufferCopy region{.srcOffset = 0, .dstOffset = 0, .size = bytes};
        vkCmdCopyBuffer(cmd, src.handle(), dst.handle(), 1, &region);
    });
}

Buffer Context::storage(std::span<const std::byte> data, VkBufferUsageFlags extra) {
    Buffer b(*this, data.size(), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | extra, Memory::device);
    upload(b, data);
    return b;
}

Accel Context::build_accel(VkAccelerationStructureTypeKHR type,
                           const VkAccelerationStructureGeometryKHR& geometry,
                           std::uint32_t primitives) {
    VkAccelerationStructureBuildGeometryInfoKHR build{
        .sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR,
        .type = type,
        .flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR,
        .mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR,
        .geometryCount = 1,
        .pGeometries = &geometry};
    VkAccelerationStructureBuildSizesInfoKHR sizes{
        .sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR};
    vkGetAccelerationStructureBuildSizesKHR(
        device(), VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &build, &primitives, &sizes);

    Accel out;
    out.storage_ = Buffer(*this, sizes.accelerationStructureSize,
                          VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR, Memory::device);
    out.handle_ = std::make_unique<Accel::Handle>(::vk::raii::AccelerationStructureKHR(
        impl_->device, ::vk::AccelerationStructureCreateInfoKHR{
                           .buffer = out.storage_.handle(),
                           .size = sizes.accelerationStructureSize,
                           .type = static_cast<::vk::AccelerationStructureTypeKHR>(type)}));
    const Buffer scratch(*this, sizes.buildScratchSize + scratch_align,
                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, Memory::device);
    build.dstAccelerationStructure = out.handle();
    build.scratchData.deviceAddress =
        (scratch.address() + scratch_align - 1) & ~(scratch_align - 1);
    const VkAccelerationStructureBuildRangeInfoKHR range{.primitiveCount = primitives};
    const VkAccelerationStructureBuildRangeInfoKHR* ranges = &range;
    run([&](VkCommandBuffer cmd) { vkCmdBuildAccelerationStructuresKHR(cmd, 1, &build, &ranges); });

    const VkAccelerationStructureDeviceAddressInfoKHR info{
        .sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR,
        .accelerationStructure = out.handle()};
    out.address_ = vkGetAccelerationStructureDeviceAddressKHR(device(), &info);
    return out;
}

Accel Context::build_blas(std::span<const float> vertices, std::span<const std::uint32_t> indices) {
    constexpr VkBufferUsageFlags input =
        VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;
    const Buffer vbuf = storage(std::as_bytes(vertices), input);
    const Buffer ibuf = storage(std::as_bytes(indices), input);
    VkAccelerationStructureGeometryKHR geometry{
        .sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR,
        .geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR,
        .flags = VK_GEOMETRY_OPAQUE_BIT_KHR};
    geometry.geometry.triangles = {
        .sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR,
        .vertexFormat = VK_FORMAT_R32G32B32_SFLOAT,
        .vertexData = {.deviceAddress = vbuf.address()},
        .vertexStride = 12,
        .maxVertex = static_cast<std::uint32_t>(vertices.size() / 3 - 1),
        .indexType = VK_INDEX_TYPE_UINT32,
        .indexData = {.deviceAddress = ibuf.address()}};
    return build_accel(VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR, geometry,
                       static_cast<std::uint32_t>(indices.size() / 3));
}

Accel Context::build_tlas(std::span<const Instance> instances) {
    if (instances.empty())
        throw std::invalid_argument("a top-level structure needs at least one instance");
    std::vector<VkAccelerationStructureInstanceKHR> list;
    list.reserve(instances.size());
    for (const auto& inst : instances) {
        VkAccelerationStructureInstanceKHR i{};
        i.transform = identity();
        i.instanceCustomIndex = inst.custom_index & 0xFFFFFFU;
        i.mask = 0xFF;
        i.accelerationStructureReference = inst.blas->address();
        list.push_back(i);
    }
    const Buffer ibuf =
        storage(std::as_bytes(std::span(list)),
                VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR);
    VkAccelerationStructureGeometryKHR geometry{
        .sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR,
        .geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR};
    geometry.geometry.instances = {
        .sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR,
        .arrayOfPointers = VK_FALSE,
        .data = {.deviceAddress = ibuf.address()}};
    return build_accel(VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR, geometry,
                       static_cast<std::uint32_t>(list.size()));
}

// -- Program ------------------------------------------------------------------

// Members are destroyed in reverse order: pipeline, layouts, then the module.
struct Program::Pipeline {
    ::vk::raii::ShaderModule module{nullptr};
    ::vk::raii::DescriptorSetLayout set_layout{nullptr};
    ::vk::raii::PipelineLayout layout{nullptr};
    ::vk::raii::Pipeline pipeline{nullptr};
};

Program::Program(Context& ctx, std::span<const std::byte> spirv, std::string_view reflection_json)
    : ctx_(&ctx) {
    const Json reflection = Json::parse(reflection_json);
    const Json& entry = reflection.at("entryPoints").at(0);
    if (entry.contains("threadGroupSize"))
        group_x_ = entry["threadGroupSize"][0].get<std::uint32_t>();

    std::vector<VkDescriptorSetLayoutBinding> bindings;
    const auto add_slot = [&](const std::string& name, std::uint32_t index, VkDescriptorType type) {
        slots_[name] = {.binding = index, .type = type};
        bindings.push_back({.binding = index,
                            .descriptorType = type,
                            .descriptorCount = 1,
                            .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT});
    };
    for (const auto& p : entry.at("parameters")) {
        if (!p.contains("binding")) continue;
        const auto& b = p["binding"];
        const auto& type = p.at("type");
        const auto name = p.at("name").get<std::string>();
        const auto kind = b.at("kind").get<std::string>();
        if (kind == "uniform") {
            const auto offset = b.at("offset").get<std::uint32_t>();
            const auto size = b.at("size").get<std::uint32_t>();
            push_fields_[name] = {.offset = offset, .size = size};
            push_size_ = std::max(push_size_, offset + size);
            continue;
        }
        if (kind != "descriptorTableSlot") continue;
        const auto index = b.at("index").get<std::uint32_t>();
        const auto tkind = type.at("kind").get<std::string>();
        if (tkind == "resource") {
            const auto shape = type.at("baseShape").get<std::string>();
            add_slot(name, index,
                     shape == "accelerationStructure"
                         ? VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR
                         : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
        } else if (tkind == "constantBuffer") {
            add_slot(name, index, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
            Block block{.binding = index, .size = 0, .fields = {}};
            for (const auto& f : type.at("elementType").at("fields")) {
                const auto offset = f.at("binding").at("offset").get<std::uint32_t>();
                const auto size = f.at("binding").at("size").get<std::uint32_t>();
                block.fields[f.at("name").get<std::string>()] = {.offset = offset, .size = size};
                block.size = std::max(block.size, offset + size);
            }
            blocks_[name] = std::move(block);
        } else if (tkind == "struct") {
            // A struct of buffers takes one slot per field, in order.
            std::uint32_t k = 0;
            for (const auto& f : type.at("fields")) {
                add_slot(std::format("{}.{}", name, f.at("name").get<std::string>()), index + k++,
                         VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
            }
        }
    }
    if (push_size_ > 128) {
        log::get("dock_sim.render")
            ->debug("push constants of {} bytes; the guaranteed minimum is 128", push_size_);
    }

    const auto& device = ctx.impl_->device;
    pipeline_ = std::make_unique<Pipeline>();
    auto& p = *pipeline_;
    p.module = ::vk::raii::ShaderModule(
        device, ::vk::ShaderModuleCreateInfo{
                    .codeSize = spirv.size(),
                    .pCode = reinterpret_cast<const std::uint32_t*>(spirv.data())});
    std::vector<::vk::DescriptorSetLayoutBinding> vk_bindings;
    vk_bindings.reserve(bindings.size());
    for (const auto& b : bindings) {
        vk_bindings.push_back(
            {.binding = b.binding,
             .descriptorType = static_cast<::vk::DescriptorType>(b.descriptorType),
             .descriptorCount = 1,
             .stageFlags = ::vk::ShaderStageFlagBits::eCompute});
    }
    p.set_layout = ::vk::raii::DescriptorSetLayout(
        device, ::vk::DescriptorSetLayoutCreateInfo{
                    .flags = ::vk::DescriptorSetLayoutCreateFlagBits::ePushDescriptorKHR,
                    .bindingCount = static_cast<std::uint32_t>(vk_bindings.size()),
                    .pBindings = vk_bindings.data()});
    const ::vk::PushConstantRange range{
        .stageFlags = ::vk::ShaderStageFlagBits::eCompute, .offset = 0, .size = push_size_};
    const ::vk::DescriptorSetLayout set_layout = *p.set_layout;
    p.layout = ::vk::raii::PipelineLayout(
        device, ::vk::PipelineLayoutCreateInfo{.setLayoutCount = 1,
                                               .pSetLayouts = &set_layout,
                                               .pushConstantRangeCount = push_size_ > 0 ? 1U : 0U,
                                               .pPushConstantRanges = &range});
    const auto entry_name = entry_point_name(spirv);
    p.pipeline = ::vk::raii::Pipeline(
        device, nullptr,
        ::vk::ComputePipelineCreateInfo{.stage = {.stage = ::vk::ShaderStageFlagBits::eCompute,
                                                  .module = *p.module,
                                                  .pName = entry_name.c_str()},
                                        .layout = *p.layout});
}

Program::~Program() = default;

const Program::Slot& Program::slot(std::string_view name) const {
    const auto it = slots_.find(name);
    if (it == slots_.end())
        throw std::invalid_argument(std::format("shader has no binding {}", name));
    return it->second;
}

Program::Call::Call(const Program& p) : program_(&p), push_(p.push_size_) {
    for (const auto& [name, block] : p.blocks_) blocks_[name] = std::vector<std::byte>(block.size);
}

Program::Call& Program::Call::bind(std::string_view name, const Buffer& buffer) {
    buffers_[program_->slot(name).binding] = {
        .buffer = buffer.handle(), .offset = 0, .range = VK_WHOLE_SIZE};
    return *this;
}

Program::Call& Program::Call::bind(std::string_view name, const Accel& accel) {
    accels_[program_->slot(name).binding] = accel.handle();
    return *this;
}

void Program::Call::put(std::vector<std::byte>& target, std::size_t offset, const void* data,
                        std::size_t size) {
    if (offset > target.size() || target.size() - offset < size)
        throw std::out_of_range("shader parameter outside its block");
    std::memcpy(target.data() + offset, data, size);
}

namespace {

template <class T>
void push_value(const auto& fields, std::string_view name, std::vector<std::byte>& target,
                const T& value, const auto& put) {
    const auto it = fields.find(name);
    if (it == fields.end())
        throw std::invalid_argument(std::format("shader has no parameter {}", name));
    if (it->second.size != sizeof(T)) {
        throw std::invalid_argument(std::format("shader parameter {} has {} bytes, the value {}",
                                                name, it->second.size, sizeof(T)));
    }
    put(target, it->second.offset, &value, sizeof(T));
}

}  // namespace

Program::Call& Program::Call::push(std::string_view name, float v) {
    push_value(program_->push_fields_, name, push_, v, [](auto&&... a) { put(a...); });
    return *this;
}
Program::Call& Program::Call::push(std::string_view name, std::int32_t v) {
    push_value(program_->push_fields_, name, push_, v, [](auto&&... a) { put(a...); });
    return *this;
}
Program::Call& Program::Call::push(std::string_view name, std::uint32_t v) {
    push_value(program_->push_fields_, name, push_, v, [](auto&&... a) { put(a...); });
    return *this;
}
Program::Call& Program::Call::push(std::string_view name, const Vec3& v) {
    push_value(program_->push_fields_, name, push_, v, [](auto&&... a) { put(a...); });
    return *this;
}

namespace {

template <class T>
void uniform_value(const auto& blocks, auto& data, std::string_view block, std::string_view field,
                   const T& value, const auto& put) {
    const auto b = blocks.find(block);
    if (b == blocks.end())
        throw std::invalid_argument(std::format("shader has no constant buffer {}", block));
    const auto f = b->second.fields.find(field);
    if (f == b->second.fields.end())
        throw std::invalid_argument(std::format("{} has no field {}", block, field));
    if (f->second.size != sizeof(T)) {
        throw std::invalid_argument(std::format("shader field {}.{} has {} bytes, the value {}",
                                                block, field, f->second.size, sizeof(T)));
    }
    put(data.find(block)->second, f->second.offset, &value, sizeof(T));
}

}  // namespace

Program::Call& Program::Call::uniform(std::string_view block, std::string_view field, float v) {
    uniform_value(program_->blocks_, blocks_, block, field, v, [](auto&&... a) { put(a...); });
    return *this;
}
Program::Call& Program::Call::uniform(std::string_view block, std::string_view field,
                                      std::int32_t v) {
    uniform_value(program_->blocks_, blocks_, block, field, v, [](auto&&... a) { put(a...); });
    return *this;
}
Program::Call& Program::Call::uniform(std::string_view block, std::string_view field,
                                      std::uint32_t v) {
    uniform_value(program_->blocks_, blocks_, block, field, v, [](auto&&... a) { put(a...); });
    return *this;
}
Program::Call& Program::Call::uniform(std::string_view block, std::string_view field,
                                      const Vec3& v) {
    uniform_value(program_->blocks_, blocks_, block, field, v, [](auto&&... a) { put(a...); });
    return *this;
}

void Program::dispatch(VkCommandBuffer cmd, Call& call, std::uint32_t threads) const {
    // Constant buffers: one small host-visible buffer per block, owned by the call.
    for (const auto& [name, data] : call.blocks_) {
        auto& buffer = call.owned_.emplace_back(*ctx_, data.size(),
                                                VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, Memory::upload);
        std::memcpy(buffer.mapped(), data.data(), data.size());
        buffer.flush();
        call.buffers_[blocks_.find(name)->second.binding] = {
            .buffer = buffer.handle(), .offset = 0, .range = VK_WHOLE_SIZE};
    }
    std::vector<VkWriteDescriptorSet> writes;
    std::vector<VkWriteDescriptorSetAccelerationStructureKHR> accel_writes;
    accel_writes.reserve(call.accels_.size());
    for (const auto& [name, s] : slots_) {
        VkWriteDescriptorSet w{.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                               .dstBinding = s.binding,
                               .descriptorCount = 1,
                               .descriptorType = s.type};
        if (s.type == VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR) {
            const auto it = call.accels_.find(s.binding);
            if (it == call.accels_.end())
                throw std::invalid_argument(std::format("binding {} is not set", name));
            accel_writes.push_back(
                {.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR,
                 .accelerationStructureCount = 1,
                 .pAccelerationStructures = &it->second});
            w.pNext = &accel_writes.back();
        } else {
            const auto it = call.buffers_.find(s.binding);
            if (it == call.buffers_.end())
                throw std::invalid_argument(std::format("binding {} is not set", name));
            w.pBufferInfo = &it->second;
        }
        writes.push_back(w);
    }
    // NOLINTNEXTLINE(misc-misplaced-const): the handle is const; Vulkan handles are pointers.
    const VkPipelineLayout layout = *pipeline_->layout;
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, *pipeline_->pipeline);
    vkCmdPushDescriptorSetKHR(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, layout, 0,
                              static_cast<std::uint32_t>(writes.size()), writes.data());
    if (push_size_ > 0) {
        vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, push_size_,
                           call.push_.data());
    }
    vkCmdDispatch(cmd, (threads + group_x_ - 1) / group_x_, 1, 1);
    // Later dispatches and copies in the same command buffer read these results.
    const VkMemoryBarrier barrier{
        .sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER,
        .srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT,
        .dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_TRANSFER_READ_BIT};
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT, 0,
                         1, &barrier, 0, nullptr, 0, nullptr);
}

}  // namespace docksim::render::rt::vk
