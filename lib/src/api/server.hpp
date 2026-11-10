#pragma once

#include "sim/async.hpp"

#include <docksim/config.hpp>

#include <asio/thread_pool.hpp>

#include <filesystem>
#include <memory>

namespace docksim::control {
class SimControl;
}

// Instructor REST API, WebSocket push and web UI on Drogon.
namespace docksim::api {

// The web UI directory: cfg.web_root, else <executable>/../share/dock-sim/web.
[[nodiscard]] std::filesystem::path web_root(const ApiConfig& cfg);

// Drogon runs on its own thread. Every request goes to the executor of the dock;
// renders go to the worker pool. Drogon is a process singleton: start one server per process.
class ApiServer {
public:
    ApiServer(sim::Executor ex, control::SimControl& control, asio::thread_pool& workers,
              ApiConfig cfg);
    ApiServer(const ApiServer&) = delete;
    ApiServer& operator=(const ApiServer&) = delete;
    ApiServer(ApiServer&&) = delete;
    ApiServer& operator=(ApiServer&&) = delete;
    ~ApiServer();

    // False if the port is not free; the dock then runs without the API.
    bool start();
    // Call on the executor of the dock, before the dock is destroyed.
    void stop();
    // The bound port after start(). Useful when cfg.port is 0.
    [[nodiscard]] int port() const;

    struct State;

private:
    std::shared_ptr<State> state_;
};

}  // namespace docksim::api
