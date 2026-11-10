#include "sim/views.hpp"

#include "device/catalog.hpp"
#include "device/geo.hpp"
#include "device/gimbal.hpp"
#include "log/log.hpp"
#include "media/photo.hpp"

#include <asio/io_context.hpp>
#include <asio/post.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>

namespace docksim::sim {

namespace {

const auto& logger() {
    static const auto log = log::get("dock_sim.views");
    return log;
}

// The lens field of view at the current zoom, through the gimbal rules.

double lens_dfov(const DroneSnapshot& d, const std::string& lens) {
    struct Probe {
        double latitude = 0;
        double longitude = 0;
        double height = 0;
        double attitude_head = 0;
        double gimbal_pitch = 0;
        double gimbal_yaw = 0;
        double zoom_factor{};
        double ir_zoom_factor{};
        [[nodiscard]] static double camera_heading() { return 0; }
    };
    const Probe probe{.zoom_factor = d.zoom_factor, .ir_zoom_factor = d.ir_zoom_factor};
    return gimbal::lens_dfov(probe, gimbal::lens_from_name(lens), catalog::wide_dfov);
}

}  // namespace

Cameras::Cameras(const Config& cfg, std::function<DroneSnapshot()> sample,
                 std::shared_ptr<render::Renderer> renderer)
    : cfg_(&cfg),
      sample_(std::move(sample)),
      lighting_(cfg.media.lighting_time, cfg.media.lighting_timezone, cfg.media.lighting_frozen),
      renderer_(std::move(renderer)) {
    if (const auto& base = lighting_.base()) {
        logger()->info("lighting time starts at {:%FT%T}Z ({})",
                       std::chrono::floor<std::chrono::seconds>(*base),
                       lighting_.frozen() ? "frozen" : "running");
    }
}

namespace {

DroneSnapshot snapshot_of(const device::DroneState& drone, const device::Weather* weather) {
    const bool wet = weather != nullptr && !weather->noise;
    return {.latitude = drone.latitude,
            .longitude = drone.longitude,
            .height = drone.height,
            .camera_heading = drone.camera_heading(),
            .gimbal_pitch = drone.gimbal_pitch,
            .zoom_factor = drone.zoom_factor,
            .ir_zoom_factor = drone.ir_zoom_factor,
            .home_latitude = drone.home.latitude,
            .home_longitude = drone.home.longitude,
            .course = drone.course,
            .horizontal_speed = drone.horizontal_speed,
            .vertical_speed = drone.vertical_speed,
            .rainfall = wet ? weather->rainfall : 0,
            .wind_speed = wet ? weather->wind_speed : 0.0,
            .wind_direction = wet ? weather->wind_direction : 0.0};
}

// Rain in a view. Wind direction is where the wind comes from; the air moves the other way.
std::optional<render::RainScene> rain_of(const DroneSnapshot& d, double heading, double pitch,
                                         double dfov, bool moving) {
    if (d.rainfall <= 0) return std::nullopt;
    const double to = geo::radians(d.wind_direction + 180.0);
    const double course = geo::radians(d.course);
    const double speed = moving ? d.horizontal_speed : 0.0;
    return render::RainScene{
        .rainfall = d.rainfall,
        .air_east = d.wind_speed * std::sin(to),
        .air_north = d.wind_speed * std::cos(to),
        .camera_east = speed * std::sin(course),
        .camera_north = speed * std::cos(course),
        .camera_up = moving ? d.vertical_speed : 0.0,
        .heading = heading,
        .pitch = pitch,
        .dfov = dfov,
    };
}

}  // namespace

struct DroneSampler::State {
    State(Executor e, const device::DroneState& d, const device::Weather* w)
        : ex(std::move(e)),
          drone(&d),
          weather(w),
          latest(std::make_shared<const DroneSnapshot>(snapshot_of(d, w))) {}

    [[nodiscard]] bool on_dock_thread() const {
        const auto* io = ex.target<asio::io_context::executor_type>();
        return io != nullptr && io->running_in_this_thread();
    }

    void refresh() {
        if (const auto* d = drone.load())
            latest.store(std::make_shared<const DroneSnapshot>(snapshot_of(*d, weather)));
    }

    Executor ex;
    std::atomic<const device::DroneState*> drone;
    // Read on the dock thread only, while drone is set.
    const device::Weather* weather;
    std::atomic<std::shared_ptr<const DroneSnapshot>> latest;
    std::atomic<bool> refresh_queued = false;
};

DroneSampler::DroneSampler(Executor ex, const device::DroneState& drone,
                           const device::Weather* weather)
    : state_(std::make_shared<State>(std::move(ex), drone, weather)) {}

DroneSampler::~DroneSampler() {
    state_->drone.store(nullptr);
}

std::function<DroneSnapshot()> DroneSampler::function() const {
    return [state = state_] {
        if (state->on_dock_thread()) {
            state->refresh();
        } else if (!state->refresh_queued.exchange(true)) {
            asio::post(state->ex, [state] {
                state->refresh();
                state->refresh_queued.store(false);
            });
        }
        return *state->latest.load();
    };
}

render::Renderer& Cameras::renderer() {
    const std::scoped_lock lock(mutex_);
    if (!renderer_)
        renderer_ = render::Renderer::from_config(cfg_->media, cfg_->live.render_workers);
    return *renderer_;
}

View Cameras::drone_view(const std::string& lens) {
    const auto d = sample_();
    if (d.height < in_dock_height_m)
        return {.pose = std::nullopt, .dfov = 0.0, .lens = lens, .idle_text = "AIRCRAFT IN DOCK"};
    return {
        .pose = render::Pose{.position = geo::point(d.latitude, d.longitude),
                             .height = units::above_home(d.height),
                             .home = geo::point(d.home_latitude, d.home_longitude),
                             .heading = d.camera_heading * units::degree,
                             .gimbal_pitch = d.gimbal_pitch * units::degree,
                             .time = lighting_.now()},
        .dfov = lens_dfov(d, lens),
        .lens = lens,
        .rain = rain_of(d, d.camera_heading, d.gimbal_pitch, lens_dfov(d, lens), true),
    };
}

View Cameras::dock_view() const {
    const auto d = sample_();
    const auto home = geo::point(d.home_latitude, d.home_longitude);
    return {
        .pose = render::Pose{.position = home,
                             .height = units::above_home(dock_camera_height_m),
                             .home = home,
                             .heading = 0.0 * units::degree,
                             .gimbal_pitch = dock_camera_pitch * units::degree,
                             .time = lighting_.now()},
        .dfov = dock_camera_dfov,
        .lens = "wide",
        .still = true,
        .rain = rain_of(d, 0.0, dock_camera_pitch, dock_camera_dfov, false),
    };
}

image::RgbImage Cameras::photo(const wayline::Shot& shot, std::pair<int, int> size,
                               const std::string& caption) {
    const auto& media = cfg_->media;
    if (media.renderer == PhotoRenderer::imagery) {
        try {
            auto& r = renderer();
            const auto d = sample_();
            const render::Pose pose{
                .position = geo::point(shot.latitude, shot.longitude),
                .height = units::above_home(shot.height),
                .home = geo::point(d.home_latitude, d.home_longitude),
                .heading = shot.heading * units::degree,
                .gimbal_pitch = shot.gimbal_pitch * units::degree,
                .time = lighting_.at(shot.time)};
            render::FrameOptions options;
            options.supersample = r.gpu() ? media.supersample : 1;
            // Photos take the time to march every pixel from the camera.
            options.march_scale = 1;
            auto frame = r.render(pose, size.first, size.second, catalog::wide_dfov, options);
            if (const auto rain =
                    rain_of(d, shot.heading, shot.gimbal_pitch, catalog::wide_dfov, false)) {
                const auto at = std::chrono::duration<double>(shot.time.time_since_epoch());
                render::apply_rain(frame, *rain, at.count());
            }
            return frame;
        } catch (const std::exception& e) {
            logger()->warn("photo render failed, using a test pattern: {}", e.what());
        }
    }
    return media::synthetic_image(size, caption);
}

}  // namespace docksim::sim
