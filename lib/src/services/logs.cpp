#include "services/logs.hpp"

#include "log/log.hpp"
#include "sim/context.hpp"
#include "util/digest.hpp"

#include <algorithm>
#include <chrono>
#include <format>
#include <memory>

namespace docksim::services {

namespace {

constexpr int log_total_steps = 3;
constexpr std::int64_t day_s = 86'400;

const auto& logger() {
    static const auto log = log::get("dock_sim.logs");
    return log;
}

// "0" aircraft or "3" dock. The cloud LogModuleEnum has a string JsonValue.
std::string module_name(const Json& value) {
    const auto name = value.is_string() ? value.get<std::string>() : value.dump();
    if (name != "0" && name != "3")
        throw BadParameter(std::format("module {} is not 0 or 3", name));
    return name;
}

std::vector<std::string> module_list(const Json& data) {
    const auto it = data.find("module_list");
    if (it == data.end() || it->is_null()) return {"0", "3"};
    if (!it->is_array()) throw BadParameter("module_list must be a list");
    std::vector<std::string> out;
    for (const auto& item : *it) out.push_back(module_name(item));
    if (out.empty()) out = {"0", "3"};
    return out;
}

std::int64_t seconds(sim::WallTime t) {
    return std::chrono::floor<std::chrono::seconds>(t).time_since_epoch().count();
}

std::int64_t millis(sim::WallTime t) {
    return std::chrono::floor<std::chrono::milliseconds>(t).time_since_epoch().count();
}

}  // namespace

void DeviceLogs::register_services(Registry& registry) {
    registry.add("fileupload_list",
                 [this](const Json& data) -> asio::awaitable<Reply> { co_return ok(list(data)); });
    registry.add("fileupload_start", [this](const Json& data) -> asio::awaitable<Reply> {
        co_return co_await start(data);
    });
    registry.add("fileupload_update", [this](const Json& data) -> asio::awaitable<Reply> {
        if (text(data, "status", "") != "cancel") throw BadParameter("status must be cancel");
        for (auto& module : module_list(data)) cancelled_.insert(std::move(module));
        logger()->info("log upload: cancel {}", data.dump());
        co_return ok();
    });
}

std::string DeviceLogs::module_sn(const std::string& module) const {
    return module == "3" ? ctx_.dock_sn() : ctx_.drone_sn();
}

std::string DeviceLogs::log_text(const std::string& module, int boot) const {
    std::string out;
    const auto sn = module_sn(module);
    for (int line = 1; line <= 40; ++line)
        out += std::format("{:04d} {} boot {}: simulated {} log line\n", line, sn, boot,
                           module == "3" ? "dock" : "aircraft");
    return out;
}

Json DeviceLogs::list(const Json& data) const {
    const auto now = seconds(ctx_.clock().now());
    Json files = Json::array();
    for (const auto& module : module_list(data)) {
        Json entries = Json::array();
        for (int k = 0; k < log_boot_count; ++k) {
            const int boot = log_boot_count - k;
            const std::int64_t end = now - k * day_s;
            entries.push_back({
                {"boot_index", boot},
                {"start_time", end - day_s + 60},
                {"end_time", end},
                {"size", log_text(module, boot).size()},
            });
        }
        files.push_back({{"device_sn", module_sn(module)},
                         {"module", module},
                         {"result", 0},
                         {"list", std::move(entries)}});
    }
    return {{"files", std::move(files)}};
}

asio::awaitable<Reply> DeviceLogs::start(const Json& data) {
    if (uploading()) co_return result(code::Result::log_busy);
    const Json params = data.value("params", Json::object());
    const auto it = params.find("files");
    if (it == params.end() || !it->is_array() || it->empty())
        throw BadParameter("params.files must list the files");
    if (!data.contains("bucket") || !data.contains("credentials"))
        throw BadParameter("bucket and credentials are needed");
    std::vector<File> files;
    for (const auto& item : *it) {
        File file;
        file.module = module_name(item.value("module", Json()));
        file.device_sn = module_sn(file.module);
        file.key = text(item, "object_key", "");
        if (file.key.empty()) throw BadParameter("object_key is missing");
        for (const auto& entry : item.value("list", Json::array()))
            file.boots.push_back(static_cast<int>(integer(entry, "boot_index")));
        if (file.boots.empty()) {
            for (int boot = log_boot_count; boot > 0; --boot) file.boots.push_back(boot);
        }
        files.push_back(std::move(file));
    }
    cancelled_.clear();
    upload_ = ctx_.spawn(upload(data, std::move(files), ctx_.request_bid), "log-upload");
    co_return ok();
}

bool DeviceLogs::cancelled(File& file) const {
    if (!cancelled_.contains(file.module)) return false;
    file.status = "canceled";
    return true;
}

asio::awaitable<void> DeviceLogs::report(const std::vector<File>& files, std::string_view status,
                                         const std::string& bid) {
    Json list = Json::array();
    for (const auto& f : files) {
        list.push_back({
            {"module", f.module},
            {"size", f.size},
            {"device_sn", f.device_sn},
            {"key", f.key},
            {"fingerprint", f.fingerprint},
            {"progress",
             {
                 {"current_step", f.step},
                 {"total_step", log_total_steps},
                 {"progress", f.percent},
                 {"finish_time", f.finish_time},
                 {"upload_rate", 0},
                 {"status", f.status},
                 {"result", f.result},
             }},
        });
    }
    co_await ctx_.emit_event(
        "fileupload_progress",
        {{"result", 0}, {"output", {{"status", status}, {"ext", {{"files", std::move(list)}}}}}},
        {.need_reply = false, .bid = bid});
}

asio::awaitable<void> DeviceLogs::upload(Json request, std::vector<File> files, std::string bid) {
    const double step = ctx_.cfg().maintenance.log_step_duration;
    std::shared_ptr<media::Storage> storage;
    try {
        storage = storage_(request);
    } catch (const std::exception& e) {
        logger()->error("log upload: no storage: {}", e.what());
    }
    for (auto& file : files) {
        if (!storage) {
            file.status = "failed";
            file.result = std::to_underlying(code::Result::log_upload_failed);
            continue;
        }
        if (cancelled(file)) continue;
        file.step = 1;
        file.status = "file_pull";
        co_await report(files, "in_progress", bid);
        co_await ctx_.clock().sleep(step);
        if (cancelled(file)) continue;

        std::vector<std::pair<std::string, std::string>> members;
        for (const int boot : file.boots)
            members.emplace_back(std::format("{}_{}.log", file.device_sn, boot),
                                 log_text(file.module, boot));
        const auto blob = digest::zip(members);
        file.size = blob.size();
        file.fingerprint = digest::md5_hex(blob);
        file.step = 2;
        file.status = "file_zip";
        co_await report(files, "in_progress", bid);
        co_await ctx_.clock().sleep(step);
        if (cancelled(file)) continue;

        file.step = 3;
        file.status = "file_uploading";
        co_await report(files, "in_progress", bid);
        std::string error;
        try {
            co_await sim::to_thread(ctx_.workers(), [&] {
                storage->put(file.key, blob, "application/zip");
                return 0;
            });
        } catch (const std::exception& e) {
            error = e.what();
        }
        if (!error.empty()) {
            logger()->error("log upload of {} failed: {}", file.key, error);
            file.status = "failed";
            file.result = std::to_underlying(code::Result::log_upload_failed);
            continue;
        }
        co_await ctx_.clock().sleep(step);
        file.percent = 100;
        file.status = "ok";
        file.finish_time = millis(ctx_.clock().now());
        logger()->info("log upload: {} ({} bytes)", file.key, file.size);
    }
    const auto any = [&](std::string_view status) {
        return std::ranges::any_of(files, [&](const File& f) { return f.status == status; });
    };
    const std::string_view status = any("failed") ? "failed" : any("canceled") ? "canceled" : "ok";
    co_await report(files, status, bid);
}

}  // namespace docksim::services
