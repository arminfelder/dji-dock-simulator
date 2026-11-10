#pragma once

#include "protocol/codes.hpp"
#include "services/registry.hpp"
#include "sim/async.hpp"
#include "util/json.hpp"
#include "wayline/engine.hpp"

#include <optional>
#include <string_view>

namespace docksim::sim {
class DockContext;
}
namespace docksim::flight {
class ManualFlight;
}

// Camera services of the Dock 2 thing model and the camera setters on drc/down.
// The state is in DroneState::camera; the aircraft osd and the DRC camera
// pushes show it. The wire rules are in docs/protocol-quirks.md under Camera.
namespace docksim::services {

// Seconds from camera_photo_take to its camera_photo_take_progress.
inline constexpr double photo_seconds = 1.0;

class CameraControl {
public:
    CameraControl(sim::DockContext& ctx, flight::ManualFlight& manual)
        : ctx_(ctx), manual_(manual) {}

    void register_services(Registry& registry);

    // A camera setter of drc/down. Empty if the method is not a camera method.
    std::optional<code::Result> drc_set(std::string_view method, const Json& data);

    // Bring record_time up to the clock. Call before the camera state goes out.
    void refresh();

    code::Result switch_mode(int mode);
    code::Result take_photo();
    code::Result stop_photo();
    code::Result start_recording();
    code::Result stop_recording();

private:
    asio::awaitable<void> photo(wayline::Shot shot);

    sim::DockContext& ctx_;
    flight::ManualFlight& manual_;
    sim::TaskGroup::Handle photo_;
    // Monotonic seconds at camera_recording_start.
    double record_start_ = 0.0;
};

}  // namespace docksim::services
