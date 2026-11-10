#pragma once

#include "flight/manual.hpp"
#include "live/streamer.hpp"
#include "media/uploads.hpp"
#include "services/areas.hpp"
#include "services/camera.hpp"
#include "services/hms.hpp"
#include "services/logs.hpp"
#include "services/maintenance.hpp"
#include "services/registry.hpp"
#include "services/services.hpp"
#include "sim/context.hpp"
#include "sim/views.hpp"
#include "wayline/engine.hpp"

#include <docksim/config.hpp>

#include <memory>
#include <optional>
#include <stdexcept>

namespace docksim::sim {

// The config reply carried no valid app license. The dock stays offline.
class LicenseError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// The dock sends the config request again after this many seconds.
inline constexpr auto config_retry = std::chrono::seconds{10};
// The cloud sets connect_storage = storage_update > login_time, strictly
// greater. This gap keeps storage_config_get after the update_topo.
inline constexpr double storage_announce_delay_s = 3.0;

struct Dependencies {
    services::SessionFactory sessions;
    media::StorageFactory storage;
    // Empty: render each photo through the cameras (aerial imagery or test pattern).
    media::PhotoMaker photos;
    wayline::Download download;
    std::unique_ptr<live::Streamer> streamer;
    // Checks the app license of the config reply. Empty: device.license_key does the check.
    std::function<bool(std::string_view, std::string_view, std::string_view)> verify_license;
    std::optional<std::uint64_t> weather_seed;
    // Downloads custom flight area and offline map files. Empty: every download fails.
    services::Fetch fetch;
};

// One simulated dock with its aircraft. Wires the parts; holds no protocol detail itself.
class DockSim {
public:
    DockSim(const Executor& ex, const Config& cfg, transport::Transport& transport, Clock& clock,
            asio::thread_pool& workers, Dependencies deps);

    // Go online. The transport must be connected.
    asio::awaitable<void> start(bool telemetry = true);
    asio::awaitable<void> stop();

    asio::awaitable<void> update_topo(bool online);
    asio::awaitable<void> publish_state();
    asio::awaitable<void> bind_to_organization();
    // One round of osd: charge, then the two dock messages, then the aircraft.
    asio::awaitable<void> osd_tick();

    [[nodiscard]] DockContext& ctx() { return ctx_; }
    [[nodiscard]] wayline::MissionEngine& mission() { return mission_; }
    [[nodiscard]] flight::ManualFlight& manual() { return manual_; }
    [[nodiscard]] media::MediaUploads& uploads() { return uploads_; }
    [[nodiscard]] services::CameraControl& camera() { return camera_; }
    [[nodiscard]] services::LiveService& live() { return live_; }
    [[nodiscard]] services::Maintenance& maintenance() { return maintenance_; }
    [[nodiscard]] services::FlightAreas& flight_areas() { return areas_; }
    [[nodiscard]] services::OfflineMap& offline_map() { return offline_map_; }
    [[nodiscard]] services::DeviceLogs& logs() { return logs_; }
    [[nodiscard]] services::HmsReporter& hms() { return hms_; }
    [[nodiscard]] const services::Registry& registry() const { return registry_; }
    [[nodiscard]] Cameras& cameras() { return *cameras_; }
    // For work on other threads that can outlive this object.
    [[nodiscard]] std::shared_ptr<Cameras> shared_cameras() const { return cameras_; }
    // The code that refuses a launch now, if any: emergency stop, weather, battery, faults.
    [[nodiscard]] std::optional<code::Result> preflight() const;

private:
    asio::awaitable<void> activate();
    asio::awaitable<void> announce_storage();
    asio::awaitable<void> dock_osd();
    asio::awaitable<void> drone_osd();
    asio::awaitable<void> dock_osd_loop();
    asio::awaitable<void> drone_osd_loop();
    asio::awaitable<void> state_loop();
    asio::awaitable<void> topo_loop();
    asio::awaitable<void> on_services(Json message);
    asio::awaitable<void> on_property_set(std::string topic, Json message);
    // device_reboot: the dock drops its links, then comes back.
    asio::awaitable<void> reboot_offline();
    asio::awaitable<void> reboot_online();

    DockContext ctx_;
    DroneSampler sampler_;
    std::shared_ptr<Cameras> cameras_;
    media::MediaUploads uploads_;
    wayline::MissionEngine mission_;
    flight::ManualFlight manual_;
    services::CameraControl camera_;
    services::LiveService live_;
    services::DrcControl drc_;
    services::PayloadControl payload_;
    services::Maintenance maintenance_;
    services::FlightAreas areas_;
    services::OfflineMap offline_map_;
    services::DeviceLogs logs_;
    services::HmsReporter hms_;
    services::Registry registry_;
    std::function<bool(std::string_view, std::string_view, std::string_view)> verify_license_;
};

}  // namespace docksim::sim
