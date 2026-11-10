#pragma once

#include "device/state.hpp"
#include "sim/async.hpp"
#include "sim/clock.hpp"
#include "transport/requester.hpp"
#include "transport/transport.hpp"

#include <docksim/config.hpp>

#include <asio/thread_pool.hpp>

#include <chrono>
#include <string>

namespace docksim::sim {

struct EventOptions {
    bool need_reply = true;
    // Empty: a new bid. A job reports its progress under the bid of its services message.
    std::string bid;
};

// What every part of one simulated dock shares: config, clock, state, the wire.
class DockContext {
public:
    DockContext(const Executor& ex, const Config& cfg, Clock& clock,
                transport::Transport& transport, asio::thread_pool& workers);

    [[nodiscard]] const Config& cfg() const noexcept { return *cfg_; }
    [[nodiscard]] Clock& clock() noexcept { return *clock_; }
    [[nodiscard]] transport::Transport& transport() noexcept { return *transport_; }
    [[nodiscard]] Executor executor() const { return ex_; }
    [[nodiscard]] asio::thread_pool& workers() noexcept { return *workers_; }
    [[nodiscard]] const std::string& dock_sn() const noexcept { return cfg_->device.dock_sn; }
    [[nodiscard]] const std::string& drone_sn() const noexcept { return cfg_->device.drone_sn; }

    device::DockState dock;
    device::DroneState drone;

    // bid of the services message whose handler runs. Valid until the first co_await
    // of the handler only.
    std::string request_bid;

    asio::awaitable<void> emit_event(std::string method, Json data, EventOptions options = {});
    asio::awaitable<Json> request(
        std::string method, Json data,
        std::chrono::steady_clock::duration timeout = std::chrono::seconds{20});

    // Run work in the background. A failure is logged, never lost.
    TaskGroup::Handle spawn(asio::awaitable<void> work, std::string name);
    asio::awaitable<void> settle() { return tasks_.settle(); }
    asio::awaitable<void> cancel_all() { return tasks_.cancel_all(); }
    [[nodiscard]] std::size_t running() const { return tasks_.size(); }

private:
    Executor ex_;
    const Config* cfg_;
    Clock* clock_;
    transport::Transport* transport_;
    asio::thread_pool* workers_;
    transport::Requester requests_;
    TaskGroup tasks_;
};

}  // namespace docksim::sim
