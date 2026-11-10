#include "control/wire_log.hpp"

#include "protocol/envelope.hpp"

namespace docksim::control {

void WireLog::add(transport::Direction direction, const std::string& topic, const Json& payload) {
    WireEntry entry{
        .seq = 0,
        .time_ms = envelope::now_ms(),
        .direction = direction,
        .topic = topic,
        .method = payload.is_object() ? payload.value("method", std::string{}) : std::string{},
        .body = payload.dump(),
    };
    if (entry.body.size() > max_body) {
        entry.body.resize(max_body);
        entry.body += "...";
    }
    const std::scoped_lock lock(mutex_);
    entry.seq = ++seq_;
    entries_.push_back(std::move(entry));
    while (entries_.size() > capacity_) entries_.pop_front();
}

std::vector<WireEntry> WireLog::since(std::uint64_t since) const {
    const std::scoped_lock lock(mutex_);
    std::vector<WireEntry> out;
    for (const auto& e : entries_) {
        if (e.seq > since) out.push_back(e);
    }
    return out;
}

std::uint64_t WireLog::last_seq() const {
    const std::scoped_lock lock(mutex_);
    return seq_;
}

transport::TapSink WireLog::sink() {
    return [this](transport::Direction d, const std::string& topic, const Json& payload) {
        add(d, topic, payload);
    };
}

Json to_json(const WireEntry& entry) {
    return {
        {"seq", entry.seq},
        {"time", entry.time_ms},
        {"direction", entry.direction == transport::Direction::in ? "in" : "out"},
        {"topic", entry.topic},
        {"method", entry.method},
        {"body", entry.body},
    };
}

}  // namespace docksim::control
