#pragma once

#include "live/streamer.hpp"
#include "services/registry.hpp"
#include "sim/async.hpp"
#include "transport/transport.hpp"

#include <docksim/config.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <tuple>

namespace docksim::device {
struct DockState;
}
namespace docksim::sim {
class DockContext;
}
namespace docksim::wayline {
class MissionEngine;
}
namespace docksim::flight {
class ManualFlight;
}
namespace docksim::services {
class CameraControl;
}

namespace docksim::services {

// Wayline task services. The replies carry result only, as the real dock sends them.
void register_flighttask(Registry& registry, wayline::MissionEngine& mission,
                         const std::string& dock_model);

// Payload-control services of the Dock 2 DRC thing model.
class PayloadControl {
public:
    explicit PayloadControl(sim::DockContext& ctx) : ctx_(ctx) {}
    void register_services(Registry& registry);

private:
    sim::DockContext& ctx_;
    double last_drag_ = 0.0;
};

class LiveService {
public:
    LiveService(sim::DockContext& ctx, std::unique_ptr<live::Streamer> streamer)
        : ctx_(ctx), streamer_(std::move(streamer)) {}
    void register_services(Registry& registry);
    asio::awaitable<void> close();
    [[nodiscard]] live::Streamer& streamer() { return *streamer_; }

private:
    sim::DockContext& ctx_;
    std::unique_ptr<live::Streamer> streamer_;
};

using SessionFactory = std::function<std::shared_ptr<transport::Session>(
    const BrokerConfig& cfg, const std::string& client_id)>;

struct BrokerAddress {
    std::string host;
    int port = 0;
    bool tls = false;
    BrokerTransport transport = BrokerTransport::tcp;
    std::string path = "/mqtt";

    friend bool operator==(const BrokerAddress&, const BrokerAddress&) = default;
};

// Host, port, TLS, transport and WebSocket path of a broker address. The
// scheme in the address wins over enable_tls. An empty host means unusable.
[[nodiscard]] BrokerAddress split_broker(std::string_view address, bool enable_tls);

// The dock leaves DRC when no heart_beat came for this time, seconds.
inline constexpr double drc_heartbeat_timeout_s = 60.0;
// drc_mode_enter without osd_frequency or hsi_frequency, hertz.
inline constexpr int drc_default_osd_hz = 10;
inline constexpr int drc_default_hsi_hz = 1;
inline constexpr int drc_max_osd_hz = 30;

// DRC mode: a second broker session that the cloud names in drc_mode_enter.
// The wire rules (nested mqtt_broker, exact drc/down subscription, flat
// drc_status_notify, drc/up replies and pushes) are in docs/protocol-quirks.md under DRC.
class DrcControl {
public:
    DrcControl(sim::DockContext& ctx, SessionFactory sessions, flight::ManualFlight& manual,
               CameraControl& camera)
        : ctx_(ctx), sessions_(std::move(sessions)), manual_(manual), camera_(camera) {}
    void register_services(Registry& registry);
    asio::awaitable<Reply> enter(const Json& data);
    asio::awaitable<Reply> exit();
    [[nodiscard]] bool connected() const { return session_ != nullptr; }

private:
    struct Link {
        BrokerConfig broker;
        std::string client_id;
        friend bool operator==(const Link&, const Link&) = default;
    };

    asio::awaitable<void> run(std::shared_ptr<transport::Session> session);
    asio::awaitable<void> close();
    asio::awaitable<void> notify(code::DrcState state, code::Result result);
    asio::awaitable<void> on_down(Json message);
    asio::awaitable<void> pushes(std::shared_ptr<transport::Session> session);
    asio::awaitable<void> push_states(std::shared_ptr<transport::Session> session);
    asio::awaitable<void> push_camera(std::shared_ptr<transport::Session> session);
    asio::awaitable<void> push(std::shared_ptr<transport::Session> session, std::string method,
                               Json data);
    asio::awaitable<void> reply(Json request, std::string method, code::Result result);
    asio::awaitable<void> idle_exit();
    void set_frequencies(const Json& data);

    sim::DockContext& ctx_;
    SessionFactory sessions_;
    flight::ManualFlight& manual_;
    CameraControl& camera_;
    std::shared_ptr<transport::Session> session_;
    sim::TaskGroup::Handle runner_;
    sim::TaskGroup::Handle pusher_;
    std::optional<Link> link_;
    int osd_hz_ = drc_default_osd_hz;
    int hsi_hz_ = drc_default_hsi_hz;
    double last_heartbeat_ = 0.0;
    std::int64_t up_seq_ = 0;
    // Each enter and exit increments this. An enter that waits for its broker
    // keeps its session only if no other enter or exit came during the wait.
    std::uint64_t attempt_ = 0;
};

// The drc_mode_enter password is a broker JWT. Keep it out of the log.
[[nodiscard]] Json redact_passwords(const Json& value);

}  // namespace docksim::services
