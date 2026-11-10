#pragma once

#include "transport/tap.hpp"
#include "util/json.hpp"

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

// The last MQTT messages of the dock, for the instructor console.
namespace docksim::control {

struct WireEntry {
    std::uint64_t seq = 0;
    std::int64_t time_ms = 0;
    transport::Direction direction = transport::Direction::in;
    std::string topic;
    // Empty if the message has no method.
    std::string method;
    // The payload as JSON text. Cut at max_body bytes.
    std::string body;
};

// Thread-safe ring buffer.
class WireLog {
public:
    static constexpr std::size_t default_capacity = 500;
    static constexpr std::size_t max_body = 8192;

    explicit WireLog(std::size_t capacity = default_capacity) : capacity_(capacity) {}

    void add(transport::Direction direction, const std::string& topic, const Json& payload);
    // Entries with seq greater than since, oldest first.
    [[nodiscard]] std::vector<WireEntry> since(std::uint64_t since) const;
    [[nodiscard]] std::uint64_t last_seq() const;

    [[nodiscard]] transport::TapSink sink();

private:
    std::size_t capacity_;
    mutable std::mutex mutex_;
    std::deque<WireEntry> entries_;
    std::uint64_t seq_ = 0;
};

[[nodiscard]] Json to_json(const WireEntry& entry);

}  // namespace docksim::control
