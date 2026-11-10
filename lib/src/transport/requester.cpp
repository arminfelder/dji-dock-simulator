#include "transport/requester.hpp"

#include "protocol/envelope.hpp"

#include <format>

namespace docksim::transport {

Requester::Requester(sim::Executor ex, Transport& transport, std::string topic, std::string gateway)
    : ex_(std::move(ex)),
      transport_(transport),
      topic_(std::move(topic)),
      gateway_(std::move(gateway)) {
    transport_.on("/requests_reply", [this](std::string, Json message) -> asio::awaitable<void> {
        const auto it = pending_.find(message.value("tid", ""));
        if (it != pending_.end()) {
            auto data = message.value("data", Json::object());
            it->second->set(data.is_null() ? Json::object() : std::move(data));
            pending_.erase(it);
        }
        co_return;
    });
}

asio::awaitable<Json> Requester::request(std::string method, Json data,
                                         std::chrono::steady_clock::duration timeout) {
    Json message = envelope::make(std::move(data), method, {{"gateway", gateway_}});
    const std::string tid = message["tid"];
    auto reply = std::make_shared<sim::OneShot<Json>>(ex_);
    pending_[tid] = reply;
    try {
        co_await transport_.publish(topic_, std::move(message));
        auto result = co_await reply->wait_for(timeout);
        pending_.erase(tid);
        if (!result) throw RequestTimeout(std::format("no requests_reply for {}", method));
        co_return std::move(*result);
    } catch (...) {
        pending_.erase(tid);
        throw;
    }
}

}  // namespace docksim::transport
