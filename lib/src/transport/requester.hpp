#pragma once

#include "sim/async.hpp"
#include "transport/transport.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

namespace docksim::transport {

class RequestTimeout : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Sends a requests message and waits for the requests_reply with the same tid.
class Requester {
public:
    Requester(sim::Executor ex, Transport& transport, std::string topic, std::string gateway);

    // The data of the reply. Throws RequestTimeout.
    asio::awaitable<Json> request(
        std::string method, Json data,
        std::chrono::steady_clock::duration timeout = std::chrono::seconds{20});

private:
    sim::Executor ex_;
    Transport& transport_;
    std::string topic_;
    std::string gateway_;
    std::map<std::string, std::shared_ptr<sim::OneShot<Json>>> pending_;
};

}  // namespace docksim::transport
