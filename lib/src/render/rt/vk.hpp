#pragma once

#include "render/vec.hpp"

#include <volk.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

struct VmaAllocator_T;
struct VmaAllocation_T;

// A thin Vulkan layer for compute with ray queries: one device, one queue,
// buffers, acceleration structures and programs whose layouts come from the
// Slang reflection. libvulkan.so.1 is opened at run time (volk), so the binary
// has no link dependency on Vulkan.
namespace docksim::render::rt::vk {

// No Vulkan loader, or no device with ray queries.
class NoDevice : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class Context;

enum class Memory : std::uint8_t {
    // GPU only. Filled by copies or shaders.
    device,
    // Host writes, GPU reads. Kept mapped.
    upload,
    // GPU writes, host reads. Kept mapped.
    readback,
};

// Frees a VMA buffer and its allocation.
struct BufferFree {
    VmaAllocator_T* allocator = nullptr;
    VkBuffer buffer = VK_NULL_HANDLE;
    void operator()(VmaAllocation_T* allocation) const noexcept;
};

class Buffer {
public:
    Buffer() = default;
    Buffer(Context& ctx, std::size_t bytes, VkBufferUsageFlags usage, Memory memory);

    [[nodiscard]] VkBuffer handle() const noexcept {
        return allocation_ ? allocation_.get_deleter().buffer : VK_NULL_HANDLE;
    }
    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] VkDeviceAddress address() const;
    // Host pointer of an upload or readback buffer; null for device memory.
    [[nodiscard]] std::byte* mapped() const noexcept { return mapped_; }
    // Make host writes visible to the GPU, and GPU writes visible to the host.
    // No-ops on coherent memory.
    void flush() const;
    void invalidate() const;

private:
    Context* ctx_ = nullptr;
    std::unique_ptr<VmaAllocation_T, BufferFree> allocation_;
    std::size_t size_ = 0;
    std::byte* mapped_ = nullptr;
};

class Accel {
public:
    Accel();
    Accel(const Accel&) = delete;
    Accel& operator=(const Accel&) = delete;
    Accel(Accel&& other) noexcept;
    Accel& operator=(Accel&& other) noexcept;
    ~Accel();

    [[nodiscard]] VkAccelerationStructureKHR handle() const noexcept;
    [[nodiscard]] VkDeviceAddress address() const noexcept { return address_; }

private:
    friend class Context;
    struct Handle;
    // The structure lives in storage_, so storage_ must outlive handle_.
    Buffer storage_;
    std::unique_ptr<Handle> handle_;
    VkDeviceAddress address_ = 0;
};

struct Instance {
    const Accel* blas;
    // Read in the shader as InstanceID(); 24 bits.
    std::uint32_t custom_index;
};

class Context {
public:
    // name_filter: part of the device name. allow_cpu admits Mesa lavapipe.
    // Throws NoDevice.
    static std::shared_ptr<Context> open(const std::optional<std::string>& name_filter,
                                         bool allow_cpu);
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
    Context(Context&&) = delete;
    Context& operator=(Context&&) = delete;
    ~Context();

    [[nodiscard]] const std::string& name() const noexcept { return name_; }
    [[nodiscard]] bool software() const noexcept { return software_; }
    [[nodiscard]] VkDevice device() const noexcept;
    [[nodiscard]] VmaAllocator_T* allocator() const noexcept;

    // Record, submit and wait. Serialised: one submission at a time.
    void run(const std::function<void(VkCommandBuffer)>& record);
    // Copy host data into a buffer (any memory) at offset. Throws std::out_of_range past its end.
    void upload(Buffer& dst, std::span<const std::byte> data, std::size_t offset = 0);
    // Copy out.size() bytes of a buffer, from offset, into host memory.
    void download(const Buffer& src, std::span<std::byte> out, std::size_t offset = 0);
    void copy(const Buffer& src, Buffer& dst, std::size_t bytes);

    // Opaque triangles: vertices xyz float, indices uint32 triples.
    [[nodiscard]] Accel build_blas(std::span<const float> vertices,
                                   std::span<const std::uint32_t> indices);
    // At least one instance: an empty top level crashed NVIDIA drivers.
    [[nodiscard]] Accel build_tlas(std::span<const Instance> instances);

    // A storage buffer filled with data.
    [[nodiscard]] Buffer storage(std::span<const std::byte> data, VkBufferUsageFlags extra = 0);

private:
    friend class Program;
    struct Impl;
    Context();
    [[nodiscard]] Accel build_accel(VkAccelerationStructureTypeKHR type,
                                    const VkAccelerationStructureGeometryKHR& geometry,
                                    std::uint32_t primitives);

    std::unique_ptr<Impl> impl_;
    std::string name_;
    bool software_ = false;
    // Recursive: a record callback of run() can upload, which runs again.
    std::recursive_mutex mutex_;
};

// One compute entry point. Bindings, push constants and uniform blocks are
// named as in the shader; the layout comes from the reflection JSON.
class Program {
public:
    Program(Context& ctx, std::span<const std::byte> spirv, std::string_view reflection_json);
    Program(const Program&) = delete;
    Program& operator=(const Program&) = delete;
    Program(Program&&) = delete;
    Program& operator=(Program&&) = delete;
    ~Program();

    // The arguments of one dispatch. Uniform blocks are copied into buffers the
    // call owns, so a call must outlive the submission that uses it.
    class Call {
    public:
        explicit Call(const Program& p);
        Call& bind(std::string_view name, const Buffer& buffer);
        Call& bind(std::string_view name, const Accel& accel);
        Call& push(std::string_view name, float v);
        Call& push(std::string_view name, std::int32_t v);
        Call& push(std::string_view name, std::uint32_t v);
        Call& push(std::string_view name, const Vec3& v);
        // A field of a ConstantBuffer parameter: block.field. The C++ type must have the size of
        // the shader field, else the call throws std::invalid_argument.
        Call& uniform(std::string_view block, std::string_view field, float v);
        Call& uniform(std::string_view block, std::string_view field, std::int32_t v);
        Call& uniform(std::string_view block, std::string_view field, std::uint32_t v);
        Call& uniform(std::string_view block, std::string_view field, const Vec3& v);

    private:
        friend class Program;
        static void put(std::vector<std::byte>& target, std::size_t offset, const void* data,
                        std::size_t size);
        const Program* program_;
        std::map<std::uint32_t, VkDescriptorBufferInfo> buffers_;
        std::map<std::uint32_t, VkAccelerationStructureKHR> accels_;
        std::vector<std::byte> push_;
        std::map<std::string, std::vector<std::byte>, std::less<>> blocks_;
        std::vector<Buffer> owned_;
    };

    // threads is the total thread count along x.
    void dispatch(VkCommandBuffer cmd, Call& call, std::uint32_t threads) const;

private:
    struct Slot {
        std::uint32_t binding;
        VkDescriptorType type;
    };
    struct Field {
        std::uint32_t offset;
        std::uint32_t size;
    };
    struct Block {
        std::uint32_t binding{};
        std::uint32_t size{};
        std::map<std::string, Field, std::less<>> fields;
    };
    [[nodiscard]] const Slot& slot(std::string_view name) const;

    Context* ctx_;
    std::map<std::string, Slot, std::less<>> slots_;
    std::map<std::string, Field, std::less<>> push_fields_;
    std::map<std::string, Block, std::less<>> blocks_;
    std::uint32_t push_size_ = 0;
    std::uint32_t group_x_ = 64;
    struct Pipeline;
    std::unique_ptr<Pipeline> pipeline_;
};

}  // namespace docksim::render::rt::vk
