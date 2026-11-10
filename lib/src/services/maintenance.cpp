#include "services/maintenance.hpp"

#include "log/log.hpp"
#include "protocol/envelope.hpp"
#include "protocol/payloads.hpp"
#include "protocol/topics.hpp"
#include "sim/context.hpp"

#include <algorithm>
#include <format>
#include <initializer_list>

namespace docksim::services {

namespace {

using code::DockMode;
using code::Result;

const auto& logger() {
    static const auto log = log::get("dock_sim.maintenance");
    return log;
}

// Every remote-debug job of the real Dock 2 starts with these steps.
std::vector<std::string> with_checks(std::initializer_list<std::string_view> steps) {
    std::vector<std::string> out{"get_bid", "check_work_mode", "check_task_state"};
    for (const auto step : steps) out.emplace_back(step);
    return out;
}

Reply with_status(std::string_view status) {
    return {{"result", 0}, {"output", {{"status", status}}}};
}

// One of the listed values, else BadParameter.
int choice(const Json& data, std::string_view key, std::initializer_list<int> allowed) {
    const auto value = static_cast<int>(integer(data, key));
    if (!std::ranges::contains(allowed, value))
        throw BadParameter(std::format("{} {} is not allowed", key, value));
    return value;
}

void need_dongle(const Json& data) {
    if (text(data, "imei", "").empty()) throw BadParameter("imei is missing");
    const auto type = text(data, "device_type", "");
    if (type != "dock" && type != "drone") throw BadParameter("device_type must be dock or drone");
}

}  // namespace

Reply in_progress() {
    return with_status("in_progress");
}

void Maintenance::register_services(Registry& registry, const std::string& dock_model) {
    auto& dock = ctx_.dock;
    auto& drone = ctx_.drone;

    registry.add("debug_mode_open", [this](const Json&) -> asio::awaitable<Reply> {
        auto& d = ctx_.dock;
        if (rebooting_) co_return result(Result::restarting);
        if (upgrading() || d.mode_code == DockMode::upgrading)
            co_return result(Result::debug_refused_upgrading);
        if (d.mode_code == DockMode::remote_debug) co_return with_status("ok");
        if (d.mode_code != DockMode::idle || airborne())
            co_return result(Result::debug_refused_task);
        d.mode_code = DockMode::remote_debug;
        logger()->info("remote debug mode on");
        co_return with_status("ok");
    });
    registry.add("debug_mode_close", [this](const Json&) -> asio::awaitable<Reply> {
        if (ctx_.dock.mode_code == DockMode::remote_debug) {
            ctx_.dock.mode_code = DockMode::idle;
            logger()->info("remote debug mode off");
        }
        co_return with_status("ok");
    });

    // The step keys are the ones of the remote-debug jobs of the real Dock 2.
    add_job(registry, {.method = "cover_open",
                       .steps = with_checks({"check_scram_state", "open_cover"}),
                       .apply = [&dock] { dock.cover_state = 1; }});
    add_job(registry,
            {.method = "cover_close",
             .steps = with_checks({"check_scram_state", "turn_on_drone", "drone_paddle_forward",
                                   "drone_paddle_reverse", "drone_paddle_stop", "close_cover"}),
             .apply = [&dock] { dock.cover_state = 0; }});
    add_job(registry, {.method = "cover_force_close",
                       .steps = {"get_bid", "check_work_mode", "check_scram_state",
                                 "drone_paddle_forward", "drone_paddle_reverse",
                                 "drone_paddle_stop", "close_cover", "check_task_state"},
                       .apply = [&dock] { dock.cover_state = 0; }});
    // Dock 2 and Dock 3 firmware has no putter services.
    if (dock_model == "dock1") {
        add_job(registry, {.method = "putter_open", .steps = {"free_putter"}, .apply = [&dock] {
                               dock.putter_state = 1;
                           }});
        add_job(registry, {.method = "putter_close", .steps = {"close_putter"}, .apply = [&dock] {
                               dock.putter_state = 0;
                           }});
    }
    add_job(registry, {.method = "charge_open",
                       .steps = with_checks({"check_scram_state", "start_charge"}),
                       .check = [&dock](const Json&) -> std::optional<Result> {
                           if (dock.drone_in_dock == 0) return Result::not_supported;
                           return std::nullopt;
                       },
                       .apply = [&dock] { dock.charging_enabled = true; }});
    add_job(registry,
            {.method = "charge_close", .steps = with_checks({"stop_charge"}), .apply = [&dock] {
                 dock.charging_enabled = false;
                 dock.drone_charge_state = 0;
             }});
    add_job(registry, {.method = "drone_open",
                       .steps = with_checks({"check_scram_state", "open_drone"}),
                       .check = [&dock](const Json&) -> std::optional<Result> {
                           if (dock.drone_in_dock == 0) return Result::drone_power_on_failed;
                           return std::nullopt;
                       },
                       .apply = [&dock] { dock.drone_powered = true; }});
    // The cloud RemoteDebugStepKeyEnum has no close_drone. It drops these events.
    add_job(registry, {.method = "drone_close",
                       .steps = with_checks({"check_scram_state", "close_drone"}),
                       .check = [&dock](const Json&) -> std::optional<Result> {
                           if (dock.drone_in_dock == 0) return Result::not_supported;
                           return std::nullopt;
                       },
                       .apply = [&dock] { dock.drone_powered = false; }});
    add_job(registry, {.method = "drone_format",
                       .check = [&dock](const Json&) -> std::optional<Result> {
                           if (!dock.drone_powered || dock.drone_in_dock == 0)
                               return Result::not_supported;
                           return std::nullopt;
                       },
                       .apply =
                           [&drone] {
                               drone.camera_photos = 0;
                               drone.camera.recorded_s = 0;
                           }});
    add_job(registry,
            {.method = "device_format", .apply = [&dock] { dock.media_remain_upload = 0; }});
    add_job(registry, {.method = "esim_activate",
                       .needs_debug = false,
                       .check = [](const Json& data) -> std::optional<Result> {
                           need_dongle(data);
                           return std::nullopt;
                       },
                       .apply = [] {}});
    add_job(registry, {.method = "esim_operator_switch",
                       .needs_debug = false,
                       .check = [](const Json& data) -> std::optional<Result> {
                           need_dongle(data);
                           std::ignore = choice(data, "telecom_operator", {1, 2, 3});
                           return std::nullopt;
                       },
                       .apply = [] {}});
    add_job(registry, {.method = "rtk_calibration",
                       .needs_debug = false,
                       .check = [](const Json& data) -> std::optional<Result> {
                           const auto it = data.find("devices");
                           if (it == data.end() || !it->is_array() || it->empty())
                               throw BadParameter("devices must be a list");
                           return std::nullopt;
                       },
                       .apply = [] {}});

    add_command(registry, "supplement_light_open", false, [&dock](const Json&) {
        dock.supplement_light_state = 1;
        return Result::ok;
    });
    add_command(registry, "supplement_light_close", false, [&dock](const Json&) {
        dock.supplement_light_state = 0;
        return Result::ok;
    });
    add_command(registry, "alarm_state_switch", false, [&dock](const Json& data) {
        dock.alarm_state = choice(data, "action", {0, 1});
        return Result::ok;
    });
    add_command(registry, "battery_maintenance_switch", false, [&dock](const Json& data) {
        // 1 is waiting: the dock starts the maintenance when the pack needs it.
        dock.battery_maintenance_state = choice(data, "action", {0, 1});
        return Result::ok;
    });
    add_command(registry, "battery_store_mode_switch", false, [&dock](const Json& data) {
        dock.battery_store_mode = choice(data, "action", {1, 2});
        return Result::ok;
    });
    add_command(registry, "sdr_workmode_switch", false, [&dock](const Json& data) {
        dock.link_workmode = choice(data, "link_workmode", {0, 1});
        return Result::ok;
    });
    add_command(registry, "air_conditioner_mode_switch", true, [&dock](const Json& data) {
        dock.air_conditioner_state = choice(data, "action", {0, 1, 2, 3});
        return Result::ok;
    });
    add_command(registry, "sim_slot_switch", false, [](const Json& data) {
        need_dongle(data);
        std::ignore = choice(data, "sim_slot", {1, 2});
        return Result::ok;
    });

    registry.add("device_reboot", [this](const Json&) -> asio::awaitable<Reply> {
        if (rebooting_) co_return result(Result::restarting);
        if (upgrading()) co_return result(Result::reboot_refused_upgrading);
        if (ctx_.dock.mode_code != DockMode::remote_debug)
            co_return result(Result::remote_debug_required);
        if (job_running()) co_return result(Result::dock_busy);
        job_ = ctx_.spawn(reboot(ctx_.request_bid), "device-reboot");
        co_return in_progress();
    });
    registry.add("ota_create", [this](const Json& data) -> asio::awaitable<Reply> {
        co_return co_await ota_create(data);
    });
}

bool Maintenance::airborne() const {
    return ctx_.dock.drone_in_dock == 0;
}

std::optional<Result> Maintenance::refuse(bool needs_debug) const {
    if (rebooting_) return Result::restarting;
    if (upgrading()) return Result::debug_refused_upgrading;
    if (needs_debug && ctx_.dock.mode_code != DockMode::remote_debug)
        return Result::remote_debug_required;
    if (job_running()) return Result::dock_busy;
    return std::nullopt;
}

void Maintenance::add_job(Registry& registry, Job job) {
    auto method = job.method;
    registry.add(std::move(method),
                 [this, job = std::move(job)](const Json& data) -> asio::awaitable<Reply> {
                     if (const auto refused = refuse(job.needs_debug)) co_return result(*refused);
                     if (job.check) {
                         if (const auto refused = job.check(data)) co_return result(*refused);
                     }
                     job_ = ctx_.spawn(run_job(job, ctx_.request_bid), job.method);
                     co_return in_progress();
                 });
}

void Maintenance::add_command(Registry& registry, std::string method, bool needs_debug,
                              std::function<Result(const Json& data)> apply) {
    registry.add(
        std::move(method),
        [this, needs_debug, apply = std::move(apply)](const Json& data) -> asio::awaitable<Reply> {
            if (rebooting_) co_return result(Result::restarting);
            if (needs_debug && ctx_.dock.mode_code != DockMode::remote_debug)
                co_return result(Result::remote_debug_required);
            const auto code = apply(data);
            if (code != Result::ok) co_return result(code);
            co_return with_status("ok");
        });
}

asio::awaitable<void> Maintenance::job_progress(const Job& job, std::string_view status, int done,
                                                int count, const std::string& bid) {
    Json progress = {{"percent", done * 100 / count}};
    if (!job.steps.empty()) {
        const int current = std::min(done + 1, count);
        progress["current_step"] = current;
        progress["step_key"] = job.steps[static_cast<std::size_t>(current - 1)];
        progress["step_result"] = 0;
        progress["total_steps"] = count;
    }
    co_await ctx_.emit_event(
        job.method,
        {{"result", 0}, {"output", {{"status", status}, {"progress", std::move(progress)}}}},
        {.bid = bid});
}

asio::awaitable<void> Maintenance::job_steps(const Job& job, const std::string& bid) {
    const int count = job.steps.empty() ? job_progress_steps : static_cast<int>(job.steps.size());
    const double step = ctx_.cfg().maintenance.job_duration / count;
    for (int i = 0; i < count; ++i) {
        co_await job_progress(job, "in_progress", i, count, bid);
        co_await ctx_.clock().sleep(step);
    }
}

asio::awaitable<void> Maintenance::run_job(Job job, std::string bid) {
    co_await job_steps(job, bid);
    job.apply();
    logger()->info("{} done", job.method);
    const int count = job.steps.empty() ? job_progress_steps : static_cast<int>(job.steps.size());
    co_await job_progress(job, "ok", count, count, bid);
}

asio::awaitable<void> Maintenance::reboot(std::string bid) {
    const Job job{.method = "device_reboot",
                  .steps = {"get_bid", "upgrading_prevent_reboot", "check_work_mode",
                            "check_task_state", "write_reboot_param_file"}};
    const auto& cfg = ctx_.cfg().maintenance;
    co_await job_steps(job, bid);
    const auto count = static_cast<int>(job.steps.size());
    co_await job_progress(job, "ok", count, count, bid);
    logger()->info("device_reboot: offline for {} s", cfg.reboot_duration);
    rebooting_ = true;
    ctx_.dock.online = false;
    if (hooks_.offline) co_await hooks_.offline();
    co_await ctx_.clock().sleep(cfg.reboot_duration);
    // A restarted dock is out of remote debug mode.
    ctx_.dock.mode_code = DockMode::idle;
    ctx_.dock.online = true;
    rebooting_ = false;
    logger()->info("device_reboot: online again");
    if (hooks_.online) co_await hooks_.online();
}

asio::awaitable<Reply> Maintenance::ota_create(const Json& data) {
    const auto it = data.find("devices");
    if (it == data.end() || !it->is_array() || it->empty() || it->size() > 2)
        throw BadParameter("devices must list one or two devices");
    for (const auto& device : *it) {
        if (!device.is_object()) throw BadParameter("a device must be an object");
        const auto sn = text(device, "sn", "");
        if (sn != ctx_.dock_sn() && sn != ctx_.drone_sn())
            throw BadParameter(std::format("sn {} is not the dock or its aircraft", sn));
        if (text(device, "product_version", "").empty())
            throw BadParameter("product_version is missing");
    }
    auto& dock = ctx_.dock;
    if (rebooting_) co_return result(Result::restarting);
    if (upgrading()) co_return result(Result::ota_upgrading);
    if (dock.mode_code != DockMode::idle || airborne() || job_running())
        co_return result(Result::ota_dock_busy);
    dock.mode_code = DockMode::upgrading;
    for (const auto& device : *it) {
        if (device["sn"] == ctx_.dock_sn()) {
            dock.firmware_upgrading = true;
        } else {
            ctx_.drone.firmware_upgrading = true;
        }
    }
    ota_ = ctx_.spawn(ota(*it, ctx_.request_bid), "ota");
    co_return in_progress();
}

asio::awaitable<void> Maintenance::publish_firmware(const std::string& sn) {
    const bool is_dock = sn == ctx_.dock_sn();
    const auto& version = is_dock ? ctx_.dock.firmware_version : ctx_.drone.firmware_version;
    const bool upgrading = is_dock ? ctx_.dock.firmware_upgrading : ctx_.drone.firmware_upgrading;
    co_await ctx_.transport().publish(
        topic::state(sn), envelope::make(payloads::firmware_group(version, upgrading), {},
                                         {{"gateway", ctx_.dock_sn()}}));
}

asio::awaitable<void> Maintenance::ota(Json devices, std::string bid) {
    const auto& cfg = ctx_.cfg().maintenance;
    for (const auto& device : devices) co_await publish_firmware(device["sn"].get<std::string>());
    const auto report = [&](std::string_view status, int percent,
                            bool download) -> asio::awaitable<void> {
        // The real dock sends current_step (1 download, 2 update) and no step_key.
        co_await ctx_.emit_event(
            "ota_progress",
            {{"result", 0},
             {"output",
              {{"status", status},
               {"progress", {{"percent", percent}, {"current_step", download ? 1 : 2}}}}}},
            {.bid = bid});
    };
    const double step = cfg.ota_duration / ota_progress_steps;
    for (int i = 0; i < ota_progress_steps; ++i) {
        co_await report("in_progress", i * 100 / ota_progress_steps, i < ota_progress_steps / 2);
        co_await ctx_.clock().sleep(step);
    }
    for (const auto& device : devices) {
        const auto version = device["product_version"].get<std::string>();
        if (device["sn"] == ctx_.dock_sn()) {
            ctx_.dock.firmware_version = version;
            ctx_.dock.firmware_upgrading = false;
        } else {
            ctx_.drone.firmware_version = version;
            ctx_.drone.firmware_upgrading = false;
        }
        logger()->info("firmware of {} is now {}", device["sn"].get<std::string>(), version);
    }
    if (ctx_.dock.mode_code == DockMode::upgrading) ctx_.dock.mode_code = DockMode::idle;
    // The cloud compares the stored firmware_version when the job ends, so the state goes first.
    for (const auto& device : devices) co_await publish_firmware(device["sn"].get<std::string>());
    co_await report("ok", 100, false);
}

}  // namespace docksim::services
