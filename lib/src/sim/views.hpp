#pragma once

#include "device/state.hpp"
#include "image/image.hpp"
#include "render/camera.hpp"
#include "render/rain.hpp"
#include "render/renderer.hpp"
#include "render/sun.hpp"
#include "sim/async.hpp"
#include "wayline/engine.hpp"

#include <docksim/config.hpp>

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

// Camera views of one dock: the live feeds and the photos of a flight.
namespace docksim::sim {

// Below this height the aircraft camera sees the inside of the dock.
inline constexpr double in_dock_height_m = 0.5;
// Dock camera: on top of the dock, tilted down across the landing site.
inline constexpr double dock_camera_height_m = 2.0;
inline constexpr double dock_camera_pitch = -15.0;
inline constexpr double dock_camera_dfov = 110.0;

struct View {
    // Empty: nothing to render (aircraft in the dock).
    std::optional<render::Pose> pose;
    double dfov = 84.0;
    std::string lens = "wide";
    // Extra overlay line, for example the reason for a dark frame.
    std::string idle_text;
    // A static view renders again only after a refresh interval.
    bool still = false;
    // Set while it rains. Apply per frame with render::apply_rain, after the render.
    std::optional<render::RainScene> rain;
};

// The aircraft state that a view needs, copied on the simulation thread.
struct DroneSnapshot {
    double latitude;
    double longitude;
    double height;
    double camera_heading;
    double gimbal_pitch;
    double zoom_factor;
    double ir_zoom_factor;
    double home_latitude;
    double home_longitude;
    // Degrees clockwise from north, metres per second.
    double course;
    double horizontal_speed;
    double vertical_speed;
    // From the dock weather. rainfall 0 is dry.
    int rainfall;
    double wind_speed;
    // Degrees clockwise from north that the wind comes from.
    double wind_direction;
};

// The latest aircraft state for render threads. A render thread never waits for the dock thread:
// it gets the last snapshot and asks the dock thread for a new one. Destroy it on the dock thread.
class DroneSampler {
public:
    // weather can be null: then the views are always dry.
    DroneSampler(Executor ex, const device::DroneState& drone,
                 const device::Weather* weather = nullptr);
    DroneSampler(const DroneSampler&) = delete;
    DroneSampler& operator=(const DroneSampler&) = delete;
    DroneSampler(DroneSampler&&) = delete;
    DroneSampler& operator=(DroneSampler&&) = delete;
    ~DroneSampler();

    // Safe on any thread, also after this object is gone.
    [[nodiscard]] std::function<DroneSnapshot()> function() const;

private:
    struct State;
    std::shared_ptr<State> state_;
};

class Cameras {
public:
    // sample returns the current aircraft state; it must be safe to call from
    // any thread (see DroneSampler).
    Cameras(const Config& cfg, std::function<DroneSnapshot()> sample,
            std::shared_ptr<render::Renderer> renderer = nullptr);

    // One renderer for every feed and photo, so that they share the caches. Created on first use.
    [[nodiscard]] render::Renderer& renderer();

    [[nodiscard]] View drone_view(const std::string& lens);
    [[nodiscard]] View dock_view() const;

    // The view of a shot. Runs on a worker thread. Falls back to the test pattern.
    [[nodiscard]] image::RgbImage photo(const wayline::Shot& shot, std::pair<int, int> size,
                                        const std::string& caption);

    [[nodiscard]] const render::LightingClock& lighting() const { return lighting_; }

private:
    const Config* cfg_;
    std::function<DroneSnapshot()> sample_;
    render::LightingClock lighting_;
    std::mutex mutex_;
    std::shared_ptr<render::Renderer> renderer_;
};

}  // namespace docksim::sim
