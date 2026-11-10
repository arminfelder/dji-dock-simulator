#pragma once

#include "util/json.hpp"

#include <asio/awaitable.hpp>

#include <functional>
#include <optional>
#include <string>

namespace docksim::transport {

using Handler = std::function<asio::awaitable<void>(std::string topic, Json message)>;

// A broker session. Subscriptions are kept and restored on every connect.
class Transport {
public:
    Transport() = default;
    Transport(const Transport&) = delete;
    Transport& operator=(const Transport&) = delete;
    Transport(Transport&&) = delete;
    Transport& operator=(Transport&&) = delete;
    virtual ~Transport() = default;

    // Call handler for every inbound topic that ends with suffix. Every
    // matching handler runs, in the order of registration.
    virtual void on(std::string suffix, Handler handler) = 0;
    // Empty qos uses the configured default.
    virtual void subscribe(std::string topic, std::optional<int> qos = std::nullopt) = 0;
    virtual asio::awaitable<void> publish(std::string topic, Json payload,
                                          std::optional<int> qos = std::nullopt) = 0;
};

// The session dropped and reconnect is off.
class ConnectionLost : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// The broker refused the first connect. Bad credentials do not get better with retries.
class ConnectionRefused : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class Session : public Transport {
public:
    // Hold the session until cancelled. Without reconnect, a lost session
    // throws ConnectionLost.
    virtual asio::awaitable<void> run(bool reconnect) = 0;
    [[nodiscard]] virtual bool connected() const = 0;
    virtual asio::awaitable<void> wait_connected() = 0;
    // Cancel the inbound-message handlers that still run, and wait for them.
    // Call it before the objects that the handlers use are destroyed.
    virtual asio::awaitable<void> cancel_handlers() = 0;
};

}  // namespace docksim::transport
