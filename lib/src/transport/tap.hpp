#pragma once

#include "transport/transport.hpp"

#include <functional>
#include <string>
#include <utility>

namespace docksim::transport {

enum class Direction : std::uint8_t { in, out };

using TapSink = std::function<void(Direction, const std::string& topic, const Json& payload)>;

// Passes every call to the inner transport and shows each message to the sink first.
// Only messages of topics with a handler are seen inbound.
class TappedTransport final : public Transport {
public:
    TappedTransport(Transport& inner, TapSink sink) : inner_(inner), sink_(std::move(sink)) {}

    void on(std::string suffix, Handler handler) override {
        inner_.on(std::move(suffix),
                  [this, handler = std::move(handler)](std::string topic,
                                                       Json message) -> asio::awaitable<void> {
                      sink_(Direction::in, topic, message);
                      co_await handler(std::move(topic), std::move(message));
                  });
    }

    void subscribe(std::string topic, std::optional<int> qos) override {
        inner_.subscribe(std::move(topic), qos);
    }

    asio::awaitable<void> publish(std::string topic, Json payload,
                                  std::optional<int> qos) override {
        sink_(Direction::out, topic, payload);
        co_await inner_.publish(std::move(topic), std::move(payload), qos);
    }

private:
    Transport& inner_;
    TapSink sink_;
};

}  // namespace docksim::transport
