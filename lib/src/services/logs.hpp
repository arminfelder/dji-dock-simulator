#pragma once

#include "media/uploads.hpp"
#include "services/registry.hpp"
#include "sim/async.hpp"
#include "util/json.hpp"

#include <set>
#include <string>
#include <vector>

namespace docksim::sim {
class DockContext;
}

// Device logs (DJI 90.log.md): fileupload_list, fileupload_start and
// fileupload_update. Each requested file is a ZIP with one synthetic text log
// per boot index, uploaded with the credentials of fileupload_start. The wire
// rules are in docs/protocol-quirks.md under Maintenance.
namespace docksim::services {

// Boot indexes that fileupload_list offers per module. The newest comes first.
inline constexpr int log_boot_count = 3;

class DeviceLogs {
public:
    DeviceLogs(sim::DockContext& ctx, media::StorageFactory storage)
        : ctx_(ctx), storage_(std::move(storage)) {}

    void register_services(Registry& registry);
    [[nodiscard]] bool uploading() const { return !upload_.done(); }

private:
    struct File {
        std::string module;
        std::string device_sn;
        std::string key;
        std::vector<int> boots;
        std::size_t size = 0;
        std::string fingerprint;
        // 1 pull, 2 compress, 3 upload.
        int step = 0;
        int percent = 0;
        std::string status = "sent";
        int result = 0;
        std::int64_t finish_time = 0;
    };

    [[nodiscard]] Json list(const Json& data) const;
    asio::awaitable<Reply> start(const Json& data);
    asio::awaitable<void> upload(Json request, std::vector<File> files, std::string bid);
    asio::awaitable<void> report(const std::vector<File>& files, std::string_view status,
                                 const std::string& bid);
    // True if fileupload_update cancelled the module. Marks the file canceled.
    bool cancelled(File& file) const;
    [[nodiscard]] std::string module_sn(const std::string& module) const;
    [[nodiscard]] std::string log_text(const std::string& module, int boot) const;

    sim::DockContext& ctx_;
    media::StorageFactory storage_;
    sim::TaskGroup::Handle upload_;
    std::set<std::string> cancelled_;
};

}  // namespace docksim::services
