#include "media/uploads.hpp"

#include "device/catalog.hpp"
#include "device/geo.hpp"
#include "log/log.hpp"
#include "protocol/payloads.hpp"
#include "sim/context.hpp"

#include <format>

namespace docksim::media {

namespace {

const auto& logger() {
    static const auto log = log::get("dock_sim.media");
    return log;
}

}  // namespace

void MediaUploads::register_services(services::Registry& registry) {
    // Force upload in the web UI. Upload this flight now, before the others.
    registry.add("upload_flighttask_media_prioritize",
                 [this](const Json& data) -> asio::awaitable<services::Reply> {
                     const auto flight_id = services::text(data, "flight_id", "");
                     if (!pending_.contains(flight_id)) {
                         logger()->warn("upload_flighttask_media_prioritize: no media waits for {}",
                                        flight_id);
                         co_return services::ok();
                     }
                     co_await ctx_.emit_event("highest_priority_upload_flighttask_media",
                                              {{"flight_id", flight_id}});
                     ctx_.spawn(upload(flight_id), std::format("media-now-{}", flight_id));
                     co_return services::ok();
                 });
}

void MediaUploads::queue(const wayline::Task& task) {
    const auto& cfg = ctx_.cfg().media;
    if (!cfg.enabled) return;
    pending_.insert_or_assign(task.flight_id, task);
    uploaded_.erase(task.flight_id);
    ctx_.dock.media_remain_upload = task.media_count;
    logger()->info("media: {} file(s) of {} wait {:.0f} s for upload", task.media_count,
                   task.flight_id, cfg.upload_delay);
    ctx_.spawn(delayed(task.flight_id), std::format("media-{}", task.flight_id));
}

void MediaUploads::hold(const std::string& flight_id, const wayline::Shot& shot) {
    auto& task = held_[flight_id];
    task.flight_id = flight_id;
    ++task.media_count;
    task.shots.push_back(shot);
}

void MediaUploads::release(const std::string& flight_id) {
    auto node = held_.extract(flight_id);
    if (!node.empty()) queue(node.mapped());
}

asio::awaitable<void> MediaUploads::delayed(std::string flight_id) {
    co_await ctx_.clock().sleep(ctx_.cfg().media.upload_delay);
    co_await upload(std::move(flight_id));
}

asio::awaitable<void> MediaUploads::upload(std::string flight_id) {
    const auto it = pending_.find(flight_id);
    if (it == pending_.end()) co_return;
    wayline::Task task = std::move(it->second);
    pending_.erase(it);

    Json reply;
    try {
        reply = co_await ctx_.request("storage_config_get", {{"module", 0}});
    } catch (const transport::RequestTimeout& e) {
        logger()->error("{}", e.what());
        requeue(std::move(task));
        co_return;
    }
    const Json output = reply.value("output", Json::object());
    const Json result = reply.value("result", Json());
    if (!(result.is_null() || result == 0) || !output.is_object() || !output.contains("bucket")) {
        logger()->error("storage_config_get failed: {}", reply.dump());
        requeue(std::move(task));
        co_return;
    }

    const std::shared_ptr<Storage> storage = storage_(output);
    const auto& cfg = ctx_.cfg();
    const auto& device = cfg.device;
    const auto home = ctx_.drone.home;
    const auto size = cfg.media.image_size;
    const auto model_key = catalog::model_key(device.drone_model);
    std::vector<wayline::Shot> shots = task.shots;
    if (shots.empty()) {
        for (int i = 0; i < task.media_count; ++i) {
            shots.push_back({home.latitude, home.longitude, cfg.flight.takeoff_altitude, 0.0,
                             -90.0, ctx_.clock().now(), 0});
        }
    }
    const auto count = static_cast<int>(shots.size());
    const auto done = uploaded_.find(task.flight_id);
    const int first = done == uploaded_.end() ? 1 : done->second + 1;
    for (int number = first; number <= count; ++number) {
        const auto& shot = shots[static_cast<std::size_t>(number - 1)];
        const auto name = std::format("DJI_{}_{:04d}.jpeg", task.flight_id.substr(0, 8), number);
        const PhotoMeta meta{
            .latitude = shot.latitude,
            .longitude = shot.longitude,
            .absolute_altitude = home.altitude + shot.height,
            .relative_altitude = shot.height,
            .heading = shot.heading,
            .gimbal_pitch = shot.gimbal_pitch,
            .time = shot.time,
            .model = std::string(catalog::camera_name(device.drone_model)),
        };
        const auto caption = std::format("{}\n{}", name, task.flight_id);
        const auto key = storage->object_key(name);
        try {
            co_await sim::to_thread(ctx_.workers(), [&] {
                const auto blob = photos_(shot, size, caption, meta);
                storage->put(key, blob, "image/jpeg");
                return 0;
            });
        } catch (const std::exception& e) {
            logger()->error("upload failed for {}: {}", key, e.what());
            requeue(std::move(task));
            co_return;
        }
        co_await ctx_.emit_event(
            "file_upload_callback",
            payloads::file_upload_callback({
                .file_name = name,
                .object_key = key,
                .path = task.flight_id,
                .flight_id = task.flight_id,
                .drone_model_key = model_key,
                .payload_model_key = catalog::payload_model_key(ctx_.drone.payload_index),
                .latitude = meta.latitude,
                .longitude = meta.longitude,
                .absolute_altitude = meta.absolute_altitude,
                .relative_altitude = meta.relative_altitude,
                .expected = count,
                .uploaded = number,
                .gimbal_yaw = geo::signed_angle(shot.heading),
                .created_time = shot.time,
            }));
        uploaded_[task.flight_id] = number;
        ctx_.dock.media_remain_upload = count - number;
        co_await ctx_.clock().sleep(upload_pacing_s);
    }
    uploaded_.erase(task.flight_id);
    logger()->info("media: {}/{} file(s) of {} uploaded", count, count, flight_id);
}

void MediaUploads::requeue(wayline::Task task) {
    auto id = task.flight_id;
    pending_.try_emplace(std::move(id), std::move(task));
}

}  // namespace docksim::media
