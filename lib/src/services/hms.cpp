#include "services/hms.hpp"

#include "device/catalog.hpp"
#include "sim/context.hpp"

#include <algorithm>

namespace docksim::services {

Json HmsReporter::data() const {
    const auto& cfg = ctx_.cfg();
    Json list = Json::array();
    std::vector<const HmsAlarm*> all;
    for (const auto& alarm : cfg.maintenance.hms) all.push_back(&alarm);
    for (const auto& alarm : raised_) all.push_back(&alarm);
    for (const auto* entry : all) {
        const auto& alarm = *entry;
        const bool dock = alarm.device == "dock";
        list.push_back({
            {"code", alarm.code},
            {"level", alarm.level},
            {"module", alarm.module},
            {"in_the_sky", alarm.in_the_sky ? 1 : 0},
            {"imminent", alarm.imminent ? 1 : 0},
            {"device_type",
             catalog::model_key(dock ? cfg.device.dock_model : cfg.device.drone_model)},
            {"args",
             {{"component_index", alarm.component_index}, {"sensor_index", alarm.sensor_index}}},
        });
    }
    return {{"list", std::move(list)}};
}

void HmsReporter::raise(HmsAlarm alarm) {
    std::erase_if(raised_, [&](const HmsAlarm& a) { return a.code == alarm.code; });
    raised_.push_back(std::move(alarm));
}

bool HmsReporter::clear(std::string_view code) {
    return std::erase_if(raised_, [&](const HmsAlarm& a) { return a.code == code; }) > 0;
}

asio::awaitable<void> HmsReporter::send() {
    co_await ctx_.emit_event("hms", data(), {.need_reply = false});
}

asio::awaitable<void> HmsReporter::run() {
    const auto& cfg = ctx_.cfg().maintenance;
    if (cfg.hms.empty()) co_return;
    while (true) {
        if (ctx_.dock.online) co_await send();
        if (cfg.hms_interval <= 0.0) co_return;
        co_await ctx_.clock().sleep(cfg.hms_interval);
    }
}

}  // namespace docksim::services
