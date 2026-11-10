#include "docksim/render.hpp"
#include "image/image.hpp"
#include "log/log.hpp"
#include "render/renderer.hpp"
#include "render/sun.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <thread>
#include <tuple>

namespace docksim {

namespace {

// The CLI renders one thing at a time, so it can use every core.
int all_cores() {
    return static_cast<int>(std::max(1U, std::thread::hardware_concurrency()));
}

}  // namespace

void render_photo(const MediaConfig& media, const PhotoRequest& r,
                  const std::filesystem::path& output) {
    auto renderer = render::Renderer::from_config(media, all_cores());
    sim::WallTime when = std::chrono::system_clock::now();
    if (r.time) {
        const auto parsed = render::parse_time(*r.time);
        if (!parsed)
            throw std::runtime_error("--time must be ISO 8601, for example 2026-06-21T10:00:00Z");
        when = *parsed;
    }
    const render::Pose pose{.position = geo::point(r.latitude, r.longitude),
                            .height = units::above_home(r.altitude),
                            .home = geo::point(r.latitude, r.longitude),
                            .heading = r.heading * units::degree,
                            .gimbal_pitch = r.pitch * units::degree,
                            .time = when};
    render::FrameOptions options;
    options.march_scale = 1;
    options.supersample = renderer->gpu() ? media.supersample : 1;
    const auto frame = renderer->render(pose, r.width, r.height, r.dfov, options);
    const auto data =
        output.extension() == ".png" ? image::encode_png(frame) : image::encode_jpeg(frame, 90);
    std::ofstream file(output, std::ios::binary);
    file.write(reinterpret_cast<const char*>(data.data()),
               static_cast<std::streamsize>(data.size()));
    if (!file) throw std::runtime_error("cannot write " + output.string());
    const auto sun = render::sun_position(r.latitude, r.longitude, when);
    log::get("dock_sim.render")
        ->info("wrote {}, sun elevation {:.1f}, azimuth {:.1f}", output.string(), sun.elevation,
               sun.azimuth);
}

std::vector<BenchResult> bench(const MediaConfig& media, int frames) {
    auto renderer = render::Renderer::from_config(media, all_cores());
    const auto t0 =
        std::chrono::sys_days{std::chrono::year{2026} / 10 / 14} + std::chrono::hours{10};
    std::vector<BenchResult> results;
    for (const auto& [w, h] : {std::pair{1280, 720}, std::pair{1920, 1080}}) {
        double total = 0;
        double worst = 0;
        for (int i = -1; i < frames; ++i) {
            const render::Pose pose{.position = geo::point(47.264 + std::max(i, 0) * 2e-6, 11.401),
                                    .height = units::above_home(80),
                                    .home = geo::point(47.263, 11.401),
                                    .heading = 10 * units::degree,
                                    .gimbal_pitch = -30 * units::degree,
                                    .time = t0 + std::chrono::seconds{std::max(i, 0)}};
            render::FrameOptions options;
            options.supersample = renderer->gpu() ? 2 : 1;
            const auto started = std::chrono::steady_clock::now();
            std::ignore = renderer->render(pose, w, h, 84.0, options);
            const double ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - started)
                                  .count();
            // The first frame loads tiles and buildings.
            if (i < 0) continue;
            total += ms;
            worst = std::max(worst, ms);
        }
        results.push_back({renderer->gpu() ? "gpu" : "cpu", w, h, total / frames, worst});
    }
    return results;
}

}  // namespace docksim
