#pragma once

#include "services/registry.hpp"
#include "sim/clock.hpp"
#include "wayline/engine.hpp"

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace docksim::sim {
class DockContext;
}

// Media after a flight: wait, render, upload to S3, report each file to the cloud.
namespace docksim::media {

// Seconds between two files. The real dock uploads one at a time.
inline constexpr double upload_pacing_s = 0.5;

// Pose and camera of one photo, for EXIF, XMP and file_upload_callback.
struct PhotoMeta {
    double latitude;
    double longitude;
    double absolute_altitude;
    double relative_altitude;
    double heading;
    double gimbal_pitch;
    sim::WallTime time;
    std::string model;
};

// A bucket with the credentials from storage_config_get. put() blocks.
class Storage {
public:
    Storage() = default;
    Storage(const Storage&) = delete;
    Storage& operator=(const Storage&) = delete;
    Storage(Storage&&) = delete;
    Storage& operator=(Storage&&) = delete;
    virtual ~Storage() = default;

    [[nodiscard]] virtual std::string object_key(std::string_view name) const = 0;
    virtual void put(const std::string& key, std::span<const std::byte> blob,
                     std::string_view content_type) = 0;
};

using StorageFactory = std::function<std::unique_ptr<Storage>(const Json& output)>;

// The JPEG of one shot. Runs on a worker thread.
using PhotoMaker =
    std::function<std::vector<std::byte>(const wayline::Shot& shot, std::pair<int, int> size,
                                         const std::string& caption, const PhotoMeta& meta)>;

class MediaUploads {
public:
    MediaUploads(sim::DockContext& ctx, StorageFactory storage, PhotoMaker photos)
        : ctx_(ctx), storage_(std::move(storage)), photos_(std::move(photos)) {}

    void register_services(services::Registry& registry);

    // Called when the aircraft has landed. The upload starts after upload_delay.
    void queue(const wayline::Task& task);
    // A photo outside a wayline mission. It waits under its flight id until release().
    void hold(const std::string& flight_id, const wayline::Shot& shot);
    // The aircraft of this flight is in the dock: queue the photos that wait.
    void release(const std::string& flight_id);
    asio::awaitable<void> upload(std::string flight_id);

    [[nodiscard]] const std::map<std::string, wayline::Task>& pending() const { return pending_; }
    [[nodiscard]] const std::map<std::string, wayline::Task>& held() const { return held_; }

private:
    asio::awaitable<void> delayed(std::string flight_id);
    // Keep failed media pending, so that Force upload can try again.
    void requeue(wayline::Task task);

    sim::DockContext& ctx_;
    StorageFactory storage_;
    PhotoMaker photos_;
    std::map<std::string, wayline::Task> pending_;
    std::map<std::string, wayline::Task> held_;
    // Files already uploaded per flight. A retry after a failure continues after them,
    // so that the cloud gets no second file_upload_callback for a file.
    std::map<std::string, int> uploaded_;
};

}  // namespace docksim::media
