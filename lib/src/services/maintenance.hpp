#pragma once

#include "protocol/codes.hpp"
#include "services/registry.hpp"
#include "sim/async.hpp"
#include "util/json.hpp"

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace docksim::sim {
class DockContext;
}

// Remote debug (DJI 70.cmd.md) and firmware update (DJI 80.firmware.md). The
// wire rules are in docs/protocol-quirks.md under Maintenance.
namespace docksim::services {

// The parts of device_reboot that only the whole dock can do.
struct RebootHooks {
    // Close DRC and live video and send the topology offline.
    std::function<asio::awaitable<void>()> offline;
    // Send the topology online and every state group.
    std::function<asio::awaitable<void>()> online;
};

// Progress events of a job without step keys, or of an update.
inline constexpr int job_progress_steps = 2;
inline constexpr int ota_progress_steps = 10;

class Maintenance {
public:
    Maintenance(sim::DockContext& ctx, RebootHooks hooks) : ctx_(ctx), hooks_(std::move(hooks)) {}

    // debug_mode_*, the hardware commands and jobs of the remote-debug panel,
    // device_reboot and ota_create.
    void register_services(Registry& registry, const std::string& dock_model);

    [[nodiscard]] bool job_running() const { return !job_.done(); }
    [[nodiscard]] bool rebooting() const { return rebooting_; }
    [[nodiscard]] bool upgrading() const { return !ota_.done(); }

private:
    // A job applies its effect at the end. It can refuse first with a result code.
    struct Job {
        std::string method;
        // The step keys of the job, in order. Empty: the progress carries
        // percent only. A step key that the cloud RemoteDebugStepKeyEnum does
        // not know makes it drop the event.
        std::vector<std::string> steps;
        bool needs_debug = true;
        std::function<std::optional<code::Result>(const Json& data)> check;
        std::function<void()> apply;
    };

    void add_job(Registry& registry, Job job);
    // A command answers at once with output.status ok.
    void add_command(Registry& registry, std::string method, bool needs_debug,
                     std::function<code::Result(const Json& data)> apply);
    [[nodiscard]] std::optional<code::Result> refuse(bool needs_debug) const;
    [[nodiscard]] bool airborne() const;

    asio::awaitable<void> run_job(Job job, std::string bid);
    asio::awaitable<void> reboot(std::string bid);
    asio::awaitable<void> job_steps(const Job& job, const std::string& bid);
    // done steps of count. percent is done * 100 / count.
    asio::awaitable<void> job_progress(const Job& job, std::string_view status, int done, int count,
                                       const std::string& bid);
    asio::awaitable<Reply> ota_create(const Json& data);
    asio::awaitable<void> ota(Json devices, std::string bid);
    asio::awaitable<void> publish_firmware(const std::string& sn);

    sim::DockContext& ctx_;
    RebootHooks hooks_;
    sim::TaskGroup::Handle job_;
    sim::TaskGroup::Handle ota_;
    bool rebooting_ = false;
};

// The reply of a job that runs on: output.status in_progress.
[[nodiscard]] Reply in_progress();

}  // namespace docksim::services
