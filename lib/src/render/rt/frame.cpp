#include "render/rt/frame.hpp"

#include "osm/facades.hpp"
#include "render/materials.hpp"
#include "render/rt/mesh.hpp"
#include "render/rt/shaders.hpp"
#include "render/shading.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace docksim::render::rt {

namespace {

constexpr VkBufferUsageFlags storage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;

// RGB rows plus a valid mask as RGBA8 in one uint32 each (R lowest byte).
std::vector<std::uint32_t> pack(std::span<const std::uint8_t> rgb,
                                std::span<const std::uint8_t> valid) {
    std::vector<std::uint32_t> out(rgb.size() / 3);
    for (std::size_t i = 0; i < out.size(); ++i) {
        const std::uint32_t a = valid.empty() || valid[i] != 0 ? 0xFF000000U : 0U;
        out[i] = rgb[3 * i] | (std::uint32_t{rgb[3 * i + 1]} << 8U) |
                 (std::uint32_t{rgb[3 * i + 2]} << 16U) | a;
    }
    return out;
}

template <class T>
std::span<const std::byte> bytes(const std::vector<T>& v) {
    return std::as_bytes(std::span(v));
}

}  // namespace

struct FrameRenderer::Programs {
    explicit Programs(vk::Context& ctx)
        : trace(ctx, shaders::trace().spirv, shaders::trace().reflection),
          shade(ctx, shaders::shade().spirv, shaders::shade().reflection),
          subsample(ctx, shaders::subsample().spirv, shaders::subsample().reflection),
          downsample(ctx, shaders::downsample().spirv, shaders::downsample().reflection) {}
    vk::Program trace;
    vk::Program shade;
    vk::Program subsample;
    vk::Program downsample;
};

FrameRenderer::FrameRenderer(std::shared_ptr<vk::Context> ctx, std::shared_ptr<Scene> scene)
    : ctx_(std::move(ctx)), scene_(std::move(scene)), programs_(std::make_unique<Programs>(*ctx_)) {
    const auto& atlas = materials::atlas();
    atlas_ = ctx_->storage(bytes(pack(atlas.texels, {})));
    const auto& normals = materials::normal_atlas();
    normals_ = ctx_->storage(bytes(pack(normals.texels, {})));
    std::vector<float> info;
    for (const auto& m : atlas.info)
        info.insert(info.end(), {m.size_m, m.mean_r, m.mean_g, m.mean_b});
    mat_info_ = ctx_->storage(bytes(info));
    std::vector<float> layouts;
    for (const auto& l : shading::window_layouts) {
        layouts.insert(layouts.end(), {l.floor_m, l.bay_m, l.floor_lo, l.floor_hi, l.bay_lo,
                                       l.bay_hi, l.recess_m});
    }
    layouts_ = ctx_->storage(bytes(layouts));
    empty_level_ = ctx_->storage(bytes(std::vector<std::uint32_t>(4, 0U)));

    // A scene with one triangle far below the ground, for frames before any
    // piece is built: the shader always needs a valid top level.
    const std::vector<float> vertices{0, 0, -1e6F, 1, 0, -1e6F, 0, 1, -1e6F};
    const std::vector<std::uint32_t> indices{0, 1, 2};
    auto empty = std::make_shared<SceneSet>();
    empty->blases.push_back(std::make_shared<vk::Accel>(ctx_->build_blas(vertices, indices)));
    const std::array instances{
        vk::Instance{.blas = empty->blases.front().get(), .custom_index = 0}};
    empty->tlas = std::make_shared<vk::Accel>(ctx_->build_tlas(instances));
    empty->records =
        std::make_shared<vk::Buffer>(ctx_->storage(bytes(std::vector<float>(record_size, 0.0F))));
    empty_scene_ = std::move(empty);
}

FrameRenderer::~FrameRenderer() = default;

std::shared_ptr<const vk::Buffer> FrameRenderer::rays_buffer(const Job& job) {
    std::erase_if(rays_, [](const auto& entry) { return entry.second.first.expired(); });
    if (const auto it = rays_.find(job.rays.get()); it != rays_.end()) return it->second.second;
    std::vector<float> flat;
    flat.reserve(job.rays->size() * 3);
    for (const auto& r : *job.rays) flat.insert(flat.end(), {r.x, r.y, r.z});
    auto buffer = std::make_shared<const vk::Buffer>(ctx_->storage(bytes(flat)));
    rays_[job.rays.get()] = {job.rays, buffer};
    return buffer;
}

std::shared_ptr<vk::Buffer> FrameRenderer::level_buffer(
    const std::shared_ptr<const geodata::Mosaic>& mosaic) {
    std::erase_if(levels_, [](const auto& entry) { return entry.second.first.expired(); });
    const auto it = levels_.find(mosaic.get());
    if (it != levels_.end()) return it->second.second;
    auto buffer =
        std::make_shared<vk::Buffer>(ctx_->storage(bytes(pack(mosaic->rgb, mosaic->valid.data()))));
    levels_[mosaic.get()] = {mosaic, buffer};
    return buffer;
}

std::shared_ptr<const vk::Buffer> FrameRenderer::styles_buffer(int& count) {
    const auto snapshot = osm::styles().snapshot();
    if (snapshot.version != styles_version_ || !styles_) {
        std::vector<float> flat;
        for (const auto& row : snapshot.rows) flat.insert(flat.end(), row.begin(), row.end());
        styles_ = std::make_shared<const vk::Buffer>(ctx_->storage(bytes(flat)));
        styles_version_ = snapshot.version;
        style_count_ = static_cast<int>(snapshot.rows.size());
    }
    count = style_count_;
    return styles_;
}

image::RgbImage FrameRenderer::frame(const Job& job, const LevelPicker& levels_for) {
    const auto origin = scene_->origin();
    if (!origin) throw std::logic_error("ray tracing scene has no origin");
    auto set = scene_->frame(job.latitude, job.longitude, job.reach, job.wait);
    if (!set) set = empty_scene_;

    // Locked while the frame uses the caches and the GPU, but not while it waits for imagery.
    std::unique_lock lock(mutex_);
    const int w = job.width;
    const int h = job.height;
    const auto n = static_cast<std::uint32_t>(w * h);
    const auto rays = rays_buffer(job);
    const auto rot0 = Vec3{job.rot[0], job.rot[1], job.rot[2]};
    const auto rot1 = Vec3{job.rot[3], job.rot[4], job.rot[5]};
    const auto rot2 = Vec3{job.rot[6], job.rot[7], job.rot[8]};

    const vk::Buffer out_t(*ctx_, std::size_t{n} * 4, storage, vk::Memory::device);
    const vk::Buffer out_id(*ctx_, std::size_t{n} * 4, storage, vk::Memory::device);
    const vk::Buffer out_bary(*ctx_, std::size_t{n} * 8, storage, vk::Memory::device);
    const int sw = (w + level_stride - 1) / level_stride;
    const int sh = (h + level_stride - 1) / level_stride;
    const vk::Buffer small_t(*ctx_, static_cast<std::size_t>(sw * sh) * 4, storage,
                             vk::Memory::device);

    vk::Program::Call trace(programs_->trace);
    trace.bind("tlas", *set->tlas)
        .bind("base", *rays)
        .push("rot0", rot0)
        .push("rot1", rot1)
        .push("rot2", rot2)
        .push("cam", job.camera)
        .push("reach", job.reach)
        .push("count", n)
        .bind("out_t", out_t)
        .bind("out_id", out_id)
        .bind("out_bary", out_bary);
    vk::Program::Call sub(programs_->subsample);
    sub.bind("src", out_t)
        .push("src_width", w)
        .push("stride", level_stride)
        .push("width", sw)
        .push("height", sh)
        .bind("dst", small_t);
    ctx_->run([&](VkCommandBuffer cmd) {
        programs_->trace.dispatch(cmd, trace, n);
        programs_->subsample.dispatch(cmd, sub, static_cast<std::uint32_t>(sw * sh));
    });

    std::vector<float> distance(static_cast<std::size_t>(sw * sh));
    ctx_->download(small_t, std::as_writable_bytes(std::span(distance)));
    for (auto& d : distance) {
        if (d < 0) d = std::numeric_limits<float>::infinity();
    }
    lock.unlock();
    const auto mosaics = levels_for(distance, sw, sh, level_stride);
    lock.lock();

    // Three float4 per level; see LevelMaps in render.slang.
    std::vector<float> params(std::size_t{4} * 3 * 4, 0.0F);
    std::vector<std::shared_ptr<vk::Buffer>> level_buffers;
    const auto count = static_cast<int>(std::min<std::size_t>(mosaics.size(), 4));
    for (int k = 0; k < count; ++k) {
        const auto& m = mosaics[static_cast<std::size_t>(k)];
        const auto pm =
            origin->pixel_map(m->zoom, static_cast<double>(m->x0), static_cast<double>(m->y0));
        const std::array<float, 12> p{static_cast<float>(m->height()),
                                      static_cast<float>(m->width()),
                                      0,
                                      0,
                                      pm.cx,
                                      pm.cy,
                                      pm.kx,
                                      pm.ky1,
                                      pm.ky2,
                                      pm.ky3,
                                      static_cast<float>(m->zoom),
                                      0};
        std::ranges::copy(p, params.begin() + static_cast<std::ptrdiff_t>(k) * 12);
        level_buffers.push_back(level_buffer(m));
    }
    const auto level_params = ctx_->storage(bytes(params));

    int style_count = 0;
    const auto styles = styles_buffer(style_count);
    const auto light = shading::light_params(job.sun_position.elevation, job.sun);
    const auto sky = shading::sky_params(job.sun_position.elevation, job.sun_position.azimuth);
    const auto exposure = shading::exposure_params(job.sun_position.elevation);
    // Averaging s x s samples divides the noise by s; scale it back up.
    const float sigma =
        job.noise ? exposure.sigma * static_cast<float>(std::max(1, job.supersample)) : 0.0F;

    const vk::Buffer rgb(*ctx_, std::size_t{n} * 4, storage, vk::Memory::device);
    vk::Program::Call shade(programs_->shade);
    shade.bind("tlas", *set->tlas)
        .bind("base", *rays)
        .bind("dist", out_t)
        .bind("ids", out_id)
        .bind("records", *set->records)
        .bind("bary", out_bary)
        .bind("styles", *styles)
        .bind("layouts", layouts_)
        .bind("atlas", atlas_)
        .bind("normals", normals_)
        .bind("mat_info", mat_info_)
        .bind("level_params", level_params)
        .bind("out_rgb", rgb)
        .uniform("view", "rot0", rot0)
        .uniform("view", "rot1", rot1)
        .uniform("view", "rot2", rot2)
        .uniform("view", "cam", job.camera)
        .uniform("view", "reach", job.reach)
        .uniform("view", "width", w)
        .uniform("view", "height", h)
        .uniform("view", "pixel_angle", job.pixel_angle)
        .uniform("view", "texel0", job.texel0)
        .uniform("view", "style_count", style_count)
        .uniform("view", "level_count", count)
        .uniform("light", "sun", job.sun)
        .uniform("light", "day", light.day)
        .uniform("light", "flat", light.flat)
        .uniform("light", "shadows", job.shadows ? 1 : 0)
        .uniform("light", "horizon", sky.horizon)
        .uniform("light", "zenith", sky.zenith)
        .uniform("light", "low", sky.low)
        .uniform("light", "sin_az", sky.sin_az)
        .uniform("light", "cos_az", sky.cos_az)
        .uniform("light", "gain", exposure.gain)
        .uniform("light", "sigma", sigma)
        .uniform("light", "seed", job.seed);
    for (int k = 0; k < 4; ++k) {
        const auto name = std::format("levels.rgb{}", k);
        shade.bind(name, k < count ? *level_buffers[static_cast<std::size_t>(k)] : empty_level_);
    }

    const int s = std::max(1, job.supersample);
    const int ow = w / s;
    const int oh = h / s;
    const vk::Buffer small(*ctx_, static_cast<std::size_t>(ow * oh) * 4, storage,
                           vk::Memory::device);
    vk::Program::Call down(programs_->downsample);
    if (s > 1) {
        down.bind("src", rgb)
            .push("src_width", w)
            .push("s", s)
            .push("width", ow)
            .push("height", oh)
            .bind("dst", small);
    }
    ctx_->run([&](VkCommandBuffer cmd) {
        programs_->shade.dispatch(cmd, shade, n);
        if (s > 1) programs_->downsample.dispatch(cmd, down, static_cast<std::uint32_t>(ow * oh));
    });

    std::vector<std::uint32_t> packed(static_cast<std::size_t>(ow * oh));
    ctx_->download(s > 1 ? small : rgb, std::as_writable_bytes(std::span(packed)));
    image::RgbImage out(ow, oh);
    auto px = out.bytes();
    for (std::size_t i = 0; i < packed.size(); ++i) {
        px[3 * i] = static_cast<std::uint8_t>(packed[i] & 0xFFU);
        px[3 * i + 1] = static_cast<std::uint8_t>((packed[i] >> 8U) & 0xFFU);
        px[3 * i + 2] = static_cast<std::uint8_t>((packed[i] >> 16U) & 0xFFU);
    }
    return out;
}

}  // namespace docksim::render::rt
