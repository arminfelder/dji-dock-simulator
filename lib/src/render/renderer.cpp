#include "render/renderer.hpp"

#include "device/geo.hpp"
#include "log/log.hpp"
#include "osm/cells.hpp"
#include "osm/facades.hpp"
#include "render/cpu/kernel.hpp"
#include "render/materials.hpp"
#include "render/shading.hpp"
#include "render/sun.hpp"

#ifdef DOCKSIM_WITH_VULKAN_RT
#include "render/rt/frame.hpp"
#endif

#include <asio/post.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <latch>
#include <limits>

namespace docksim::render {

namespace {

// The texture extent is estimated from every n-th ray.
constexpr int extent_stride = 8;
// The ground sample distance assumes at least this height, metres.
constexpr double min_gsd_height_m = 5.0;
// Rays per pixel row band handed to one thread at a time.
constexpr int rows_per_task = 8;

class Profiler {
public:
    explicit Profiler(std::map<std::string, double>* target)
        : target_(target), last_(Clock::now()) {}
    void mark(const char* name) {
        if (target_ == nullptr) return;
        const auto now = Clock::now();
        (*target_)[name] += std::chrono::duration<double>(now - last_).count();
        last_ = now;
    }

private:
    using Clock = std::chrono::steady_clock;
    std::map<std::string, double>* target_;
    Clock::time_point last_;
};

// Calls fn(row) for every row in [0, rows), spread over the pool.
template <class Fn>
void parallel_rows(asio::thread_pool& pool, int rows, Fn&& fn) {
    const int tasks = (rows + rows_per_task - 1) / rows_per_task;
    std::latch done(tasks);
    for (int t = 0; t < tasks; ++t) {
        asio::post(pool, [&, t] {
            const int end = std::min(rows, (t + 1) * rows_per_task);
            for (int row = t * rows_per_task; row < end; ++row) fn(row);
            done.count_down();
        });
    }
    done.wait();
}

GridView view_of(const std::shared_ptr<const osm::LocalGrid>& grid,
                 const raster::LocalFrame& frame) {
    if (!grid) return {};
    return {.grid = grid.get(), .map = grid->map(frame)};
}

// Imagery mosaics for the ground extent seen by the rays at (x, y), widened
// 15 % for the frame edge. hit(x, y) is the hit distance or infinity.
std::vector<std::shared_ptr<const geodata::Mosaic>> pick_levels(
    geodata::Imagery& imagery, const raster::LocalFrame& frame, const Pose& pose, double dfov_deg,
    int width, int height, double far, bool wait, int cols, int rows,
    const std::function<Vec3(int, int)>& ray_at, const std::function<float(int, int)>& hit_at) {
    double south = frame.lat0;
    double north = frame.lat0;
    double west = frame.lon0;
    double east = frame.lon0;
    bool any = false;
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            const Vec3 r = ray_at(x, y);
            const float hit = hit_at(x, y);
            const float d = std::isfinite(hit) ? hit : static_cast<float>(far);
            const auto [lat, lon] = frame.to_latlon(r.x * d, r.y * d);
            if (!any) {
                south = north = lat;
                west = east = lon;
                any = true;
            }
            south = std::min(south, lat);
            north = std::max(north, lat);
            west = std::min(west, lon);
            east = std::max(east, lon);
        }
    }
    const double mlat = (north - south) * 0.15;
    const double mlon = (east - west) * 0.15;
    const double gsd = std::max(units::metres_above_home(pose.height), min_gsd_height_m) * 2.0 *
                       std::tan(geo::radians(dfov_deg) / 2.0) / std::hypot(width, height);
    return imagery.levels(
        {.south = south - mlat, .west = west - mlon, .north = north + mlat, .east = east + mlon},
        frame.lat0, frame.lon0, gsd, wait);
}

std::vector<cpu::StyleRow> style_rows() {
    const auto snapshot = osm::styles().snapshot();
    std::vector<cpu::StyleRow> rows;
    rows.reserve(snapshot.rows.size());
    for (const auto& r : snapshot.rows) {
        rows.push_back({.colour = {r[0], r[1], r[2]},
                        .layout = static_cast<int>(r[3]),
                        .weathering = r[4],
                        .material = static_cast<int>(r[5])});
    }
    return rows;
}

// The largest up component of the view rays: every fourth ray, and the two top corners
// that the grid can miss. It sets how far the scene must reach.
template <class Ray>
float highest_ray(const Ray& ray, int width, int height) {
    float max_up = -1.0F;
    for (int y = 0; y < height; y += 4) {
        for (int x = 0; x < width; x += 4) max_up = std::max(max_up, ray(x, y).z);
    }
    return std::max({max_up, ray(width - 1, 0).z, ray(0, 0).z});
}

}  // namespace

Renderer::Renderer(Sources sources)
    : sources_(std::move(sources)),
      pool_(static_cast<std::size_t>(std::max(1, sources_.threads))) {}

Renderer::~Renderer() {
    pool_.join();
}

std::unique_ptr<Renderer> Renderer::from_config(const MediaConfig& media, int threads) {
    Sources s;
    s.imagery = std::make_shared<geodata::Imagery>(media.imagery_url, media.imagery_max_zoom,
                                                   media.cache_dir);
    s.terrain = std::make_shared<geodata::Terrain>(media.dem_url, media.dem_zoom, media.cache_dir,
                                                   media.dem_near_zoom, media.dem_near_radius);
    s.buildings = osm::make_provider(media);
    s.shadows = media.shadows;
    s.threads = threads;
#ifdef DOCKSIM_WITH_VULKAN_RT
    if (media.render_device == RenderDevice::rt || media.render_device == RenderDevice::automatic) {
        try {
            auto ctx = rt::vk::Context::open(media.gpu_device, media.rt_allow_cpu);
            rt::SceneSources scene{
                .terrain = s.terrain,
                .cells = media.buildings ? osm::make_cell_store(media) : nullptr,
                .scan = media.building_scan ? std::make_shared<raster::als::HeightAboveGround>(
                                                  osm::expand_user(media.cache_dir).string())
                                            : nullptr,
                .building_radius = media.building_radius,
                .far_radius = media.building_far_radius,
                .scan_radius = media.building_scan_radius,
            };
            s.rt = std::make_shared<rt::FrameRenderer>(
                ctx, std::make_shared<rt::Scene>(ctx, std::move(scene)));
            // The heightfield grid is not needed for ray tracing.
            s.buildings.reset();
        } catch (const rt::vk::NoDevice& e) {
            if (media.render_device == RenderDevice::rt) throw;
            log::get("dock_sim.render")
                ->info("renderer uses the CPU, no ray tracing GPU: {}", e.what());
        }
    } else if (media.render_device == RenderDevice::gpu) {
        log::get("dock_sim.render")
            ->warn("render_device gpu (OpenCL) is gone; using the CPU kernels");
    }
#endif
    return std::make_unique<Renderer>(std::move(s));
}

bool Renderer::gpu() const noexcept {
#ifdef DOCKSIM_WITH_VULKAN_RT
    return sources_.rt != nullptr;
#else
    return false;
#endif
}

image::RgbImage Renderer::render_rt([[maybe_unused]] const Pose& pose,
                                    [[maybe_unused]] int out_width, [[maybe_unused]] int out_height,
                                    [[maybe_unused]] double dfov_deg,
                                    [[maybe_unused]] const FrameOptions& options) {
#ifdef DOCKSIM_WITH_VULKAN_RT
    auto& frames = *sources_.rt;
    auto& terrain = *sources_.terrain;
    const int s = std::max(1, options.supersample);
    const int width = out_width * s;
    const int height = out_height * s;
    const raster::LocalFrame frame{.lat0 = units::in_degrees(pose.position.lat),
                                   .lon0 = units::in_degrees(pose.position.lon)};
    const auto rays = camera_rays(width, height, dfov_deg);
    const auto rot =
        rotation(units::in_degrees(pose.heading), units::in_degrees(pose.gimbal_pitch));
    const auto ray = [&](int x, int y) {
        return to_world((*rays)[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                                static_cast<std::size_t>(x)],
                        rot);
    };
    const double far =
        reach(highest_ray(ray, width, height), units::metres_above_home(pose.height));

    // Positions count from the home point and its ground.
    const double home_lat = units::in_degrees(pose.home.lat);
    const double home_lon = units::in_degrees(pose.home.lon);
    const float home_ground = terrain.point(home_lat, home_lon);
    frames.scene().set_origin(home_lat, home_lon, home_ground);
    const auto origin = frames.scene().origin().value();
    const auto [east, north] = origin.to_local(frame.lat0, frame.lon0);
    const auto sun_pos = sun_position(frame.lat0, frame.lon0, pose.time);
    const rt::Job job{
        .rays = rays,
        .width = width,
        .height = height,
        .dfov = dfov_deg,
        .rot = rot,
        // Pose height counts from the dock; a dock on a roof starts at the roof.
        .camera = {static_cast<float>(east), static_cast<float>(north),
                   frames.scene().home_roof(options.wait_for_data) +
                       static_cast<float>(units::metres_above_home(pose.height))},
        .latitude = frame.lat0,
        .longitude = frame.lon0,
        .reach = static_cast<float>(far),
        .sun_position = sun_pos,
        .sun = sun_vector(sun_pos),
        .shadows = sources_.shadows && sun_pos.elevation > 0.0,
        .seed = static_cast<std::uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(pose.time.time_since_epoch())
                .count() &
            0xFFFFFFFF),
        .noise = sources_.sensor_noise,
        .pixel_angle = static_cast<float>(pixel_angle(width, height, dfov_deg)),
        .texel0 = static_cast<float>(geodata::texel_metres(origin.lat0, 0)),
        .supersample = s,
        .wait = options.wait_for_data,
    };
    const auto levels_for = [&](std::span<const float> distance, int cols, int rows, int stride) {
        return pick_levels(
            *sources_.imagery, frame, pose, dfov_deg, width, height, far, options.wait_for_data,
            cols, rows,
            [&](int x, int y) {
                return ray(std::min(x * stride, width - 1), std::min(y * stride, height - 1));
            },
            [&](int x, int y) {
                return distance[static_cast<std::size_t>(y) * static_cast<std::size_t>(cols) +
                                static_cast<std::size_t>(x)];
            });
    };
    return frames.frame(job, levels_for);
#else
    throw std::logic_error("this build has no ray tracing");
#endif
}

image::RgbImage Renderer::render(const Pose& pose, int out_width, int out_height, double dfov_deg,
                                 const FrameOptions& options) {
    // RT frames lock inside FrameRenderer. CPU frames lock only around the work on pool_ below,
    // so that a frame that waits for tiles or OSM cells does not hold up the others.
    if (gpu()) return render_rt(pose, out_width, out_height, dfov_deg, options);
    Profiler prof(options.profile);
    const int s = std::max(1, options.supersample);
    const int width = out_width * s;
    const int height = out_height * s;
    const raster::LocalFrame frame{.lat0 = units::in_degrees(pose.position.lat),
                                   .lon0 = units::in_degrees(pose.position.lon)};
    const auto rays = camera_rays(width, height, dfov_deg);
    const auto rot =
        rotation(units::in_degrees(pose.heading), units::in_degrees(pose.gimbal_pitch));
    const auto ray = [&](int x, int y) {
        return to_world((*rays)[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                                static_cast<std::size_t>(x)],
                        rot);
    };

    const double far =
        reach(highest_ray(ray, width, height), units::metres_above_home(pose.height));

    auto& terrain = *sources_.terrain;
    const double home_lat = units::in_degrees(pose.home.lat);
    const double home_lon = units::in_degrees(pose.home.lon);
    const float home_ground = terrain.point(home_lat, home_lon);
    const auto relief_mosaics = terrain.relief(frame.lat0, frame.lon0, far, options.wait_for_data);
    const auto sun_pos = sun_position(frame.lat0, frame.lon0, pose.time);
    const Vec3 sun = sun_vector(sun_pos);
    const bool shadows = sources_.shadows && sun_pos.elevation > 0.0;
    prof.mark("setup");

    std::optional<osm::Buildings> buildings;
    if (sources_.buildings)
        buildings = sources_.buildings->grid(frame.lat0, frame.lon0, options.wait_for_data);
    // Pose height counts from the dock. A dock on a roof starts at the roof.
    const float home_roof = buildings ? buildings->height_at(home_lat, home_lon) : 0.0F;
    const float camera_up =
        home_ground + home_roof + static_cast<float>(units::metres_above_home(pose.height));

    cpu::Scene scene{
        .relief = geodata::to_relief(relief_mosaics, frame, home_ground),
        .buildings = buildings ? BuildingView{.near = view_of(buildings->near, frame),
                                              .far = view_of(buildings->far, frame)}
                               : BuildingView{},
        .top = relief_mosaics.top() + (buildings ? buildings->top() : 0.0F),
    };
    const cpu::MarchParams march{
        .camera_up = camera_up, .reach = static_cast<float>(far), .sun = sun, .shadows = shadows};
    prof.mark("buildings");

    std::unique_lock compute(frame_mutex_);
    std::vector<cpu::Surface> surfaces(static_cast<std::size_t>(width) *
                                       static_cast<std::size_t>(height));
    const auto at = [&](int x, int y) -> cpu::Surface& {
        return surfaces[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                        static_cast<std::size_t>(x)];
    };
    const int scale = std::max(1, options.march_scale);
    if (scale > 1) {
        // Coarse rays first; each fine ray then starts just before the nearest
        // coarse hit around it, which skips most of the empty air.
        const int cw = (width + scale - 1) / scale + 1;
        const int ch = (height + scale - 1) / scale + 1;
        std::vector<float> coarse(static_cast<std::size_t>(cw) * static_cast<std::size_t>(ch));
        parallel_rows(pool_, ch, [&](int cy) {
            for (int cx = 0; cx < cw; ++cx) {
                const int x = std::min(cx * scale, width - 1);
                const int y = std::min(cy * scale, height - 1);
                coarse[static_cast<std::size_t>(cy) * static_cast<std::size_t>(cw) +
                       static_cast<std::size_t>(cx)] = cpu::march(scene, march, ray(x, y)).distance;
            }
        });
        prof.mark("march_coarse");
        parallel_rows(pool_, height, [&](int y) {
            for (int x = 0; x < width; ++x) {
                const int cx = x / scale;
                const int cy = y / scale;
                float nearest = std::numeric_limits<float>::infinity();
                for (int dy = -1; dy <= 2; ++dy) {
                    for (int dx = -1; dx <= 2; ++dx) {
                        const int ix = std::clamp(cx + dx, 0, cw - 1);
                        const int iy = std::clamp(cy + dy, 0, ch - 1);
                        nearest = std::min(
                            nearest,
                            coarse[static_cast<std::size_t>(iy) * static_cast<std::size_t>(cw) +
                                   static_cast<std::size_t>(ix)]);
                    }
                }
                // Neighbours that all see sky still start from the camera: a
                // thin building can stand between coarse rays.
                const float start = std::isfinite(nearest) ? 0.8F * nearest : 0.5F;
                at(x, y) = cpu::march(scene, march, ray(x, y), start);
            }
        });
    } else {
        parallel_rows(pool_, height, [&](int y) {
            for (int x = 0; x < width; ++x) at(x, y) = cpu::march(scene, march, ray(x, y));
        });
    }
    prof.mark("march");

    // Imagery levels for the ground extent of the frame: every extent_stride-th ray.
    const int cols = (width + extent_stride - 1) / extent_stride;
    const int rows = (height + extent_stride - 1) / extent_stride;
    compute.unlock();
    const auto mosaics = pick_levels(
        *sources_.imagery, frame, pose, dfov_deg, width, height, far, options.wait_for_data, cols,
        rows, [&](int x, int y) { return ray(x * extent_stride, y * extent_stride); },
        [&](int x, int y) { return at(x * extent_stride, y * extent_stride).distance; });
    compute.lock();
    const Levels levels = geodata::to_levels(mosaics, frame);
    prof.mark("levels");

    const auto styles = style_rows();
    const auto [anchor_e, anchor_n] = window_anchor(pose);
    const auto exposure = shading::exposure_params(sun_pos.elevation);
    const cpu::ShadeParams shade{
        .camera_up = camera_up,
        .reach = static_cast<float>(far),
        .anchor_e = anchor_e,
        .anchor_n = anchor_n,
        .pixel_angle = static_cast<float>(pixel_angle(width, height, dfov_deg)),
        .texel0 = static_cast<float>(geodata::texel_metres(frame.lat0, 0)),
        .levels = levels,
        .styles = styles,
        .atlas = &materials::atlas(),
        .sun = sun,
        .light = shading::light_params(sun_pos.elevation, sun),
        .sky = shading::sky_params(sun_pos.elevation, sun_pos.azimuth),
        .exposure = exposure,
        // Averaging s x s samples divides the noise by s; scale it back up.
        .sigma = sources_.sensor_noise ? exposure.sigma * static_cast<float>(s) : 0.0F,
        .seed = static_cast<std::uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(pose.time.time_since_epoch())
                .count() &
            0xFFFFFFFF),
        .width = width,
        .height = height,
    };
    image::RgbImage full(width, height);
    parallel_rows(pool_, height, [&](int y) {
        for (int x = 0; x < width; ++x) {
            const auto rgb = cpu::shade(shade, y * width + x, ray(x, y), at(x, y));
            std::ranges::copy(rgb, full.pixel(x, y));
        }
    });
    prof.mark("shade");
    if (s == 1) return full;
    image::RgbImage small(out_width, out_height);
    cpu::downsample(full.bytes(), width, s, small.bytes(), out_width, out_height);
    prof.mark("downsample");
    return small;
}

}  // namespace docksim::render
