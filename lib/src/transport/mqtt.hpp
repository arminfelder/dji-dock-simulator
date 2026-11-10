#pragma once

#include "sim/async.hpp"
#include "transport/transport.hpp"

#include <docksim/config.hpp>

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace docksim::transport {

// MQTT 3.1.1 session on Eclipse Paho (MQTTAsync). One instance per client id.
// Paho calls back on its own threads; every callback is posted to the executor.
class MqttTransport final : public Session {
public:
    MqttTransport(const sim::Executor& ex, BrokerConfig cfg, std::string client_id);
    MqttTransport(const MqttTransport&) = delete;
    MqttTransport& operator=(const MqttTransport&) = delete;
    MqttTransport(MqttTransport&&) = delete;
    MqttTransport& operator=(MqttTransport&&) = delete;
    ~MqttTransport() override;

    void on(std::string suffix, Handler handler) override;
    void subscribe(std::string topic, std::optional<int> qos) override;
    asio::awaitable<void> publish(std::string topic, Json payload, std::optional<int> qos) override;

    // A refused first connect throws ConnectionRefused. Without reconnect, a
    // lost session throws ConnectionLost.
    asio::awaitable<void> run(bool reconnect) override;
    [[nodiscard]] bool connected() const override { return connected_.is_set(); }
    asio::awaitable<void> wait_connected() override { return connected_.wait(); }
    asio::awaitable<void> cancel_handlers() override { return handler_tasks_.cancel_all(); }

    [[nodiscard]] const std::string& client_id() const { return client_id_; }

    struct Impl;

private:
    sim::Executor ex_;
    BrokerConfig cfg_;
    std::string client_id_;
    sim::Event connected_;
    // A deque keeps the handlers in place when on() adds one; a running handler coroutine refers to
    // its closure.
    std::deque<std::pair<std::string, Handler>> handlers_;
    std::vector<std::pair<std::string, int>> subscriptions_;
    std::shared_ptr<Impl> impl_;
    sim::TaskGroup handler_tasks_;
};

}  // namespace docksim::transport
