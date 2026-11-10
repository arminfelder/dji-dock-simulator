#pragma once

#include "util/json.hpp"

#include <docksim/config.hpp>

#include <string_view>
#include <vector>

#include <asio/awaitable.hpp>

namespace docksim::sim {
class DockContext;
}

// HMS alarms (DJI 60.hms.md) from maintenance.hms of the config.
namespace docksim::services {

class HmsReporter {
public:
    explicit HmsReporter(sim::DockContext& ctx) : ctx_(ctx) {}

    // data of the hms event. list is empty if the config has no alarm.
    [[nodiscard]] Json data() const;
    asio::awaitable<void> send();
    // Send the list, then again after each hms_interval. Returns at once if
    // the list is empty, and after one send if hms_interval is 0.
    asio::awaitable<void> run();

    // Alarms raised at run time, after the alarms of the config. A second raise of
    // one code replaces the first. Call send() to report the change.
    void raise(HmsAlarm alarm);
    // False if no runtime alarm has this code.
    bool clear(std::string_view code);
    [[nodiscard]] const std::vector<HmsAlarm>& raised() const { return raised_; }

private:
    sim::DockContext& ctx_;
    std::vector<HmsAlarm> raised_;
};

}  // namespace docksim::services
