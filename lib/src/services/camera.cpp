#include "services/camera.hpp"

#include "flight/manual.hpp"
#include "log/log.hpp"
#include "sim/context.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <string>
#include <vector>

namespace docksim::services {

namespace {

using code::Result;

const auto& logger() {
    static const auto log = log::get("dock_sim.camera");
    return log;
}

// LensStorageSettingsEnum of the cloud.
constexpr std::array<std::string_view, 5> storage_lenses{"current", "zoom", "wide", "vision", "ir"};

int enum_value(const Json& data, std::string_view key, int min, int max) {
    const auto value = integer(data, key, std::int64_t{min} - 1);
    if (value < min || value > max)
        throw BadParameter(std::format("{} must be {} to {}", key, min, max));
    return static_cast<int>(value);
}

int enum_value(const Json& data, std::string_view key, int min, int max, int fallback) {
    if (!data.is_object() || !data.contains(std::string(key))) return fallback;
    return enum_value(data, key, min, max);
}

double fraction(const Json& data, std::string_view key, double fallback) {
    const double value = number(data, key, fallback);
    if (value < 0.0 || value > 1.0) throw BadParameter(std::format("{} must be 0 to 1", key));
    return value;
}

std::vector<std::string> storage(const Json& data, std::string_view key) {
    const auto it = data.find(key);
    if (it == data.end() || !it->is_array())
        throw BadParameter(std::format("{} must be a list", key));
    std::vector<std::string> out;
    for (const auto& item : *it) {
        if (!item.is_string() || !std::ranges::contains(storage_lenses, item.get<std::string>()))
            throw BadParameter(std::format("{} holds an unknown lens: {}", key, item.dump()));
        out.push_back(item.get<std::string>());
    }
    return out;
}

// camera_type of the exposure and focus requests: ExposureCameraTypeEnum, wide or zoom.
device::LensSettings& lens(device::CameraState& camera, const Json& data) {
    const auto type = text(data, "camera_type", "wide");
    if (type == "wide") return camera.wide;
    if (type == "zoom") return camera.zoom;
    throw BadParameter(std::format("camera_type {} has no exposure settings", type));
}

int setting(const Json& data, std::string_view key, int current) {
    return static_cast<int>(integer(data, key, current));
}

}  // namespace

void CameraControl::register_services(Registry& registry) {
    auto& camera = ctx_.drone.camera;
    const auto add = [&registry](std::string name, std::function<Result(const Json&)> apply) {
        registry.add(std::move(name),
                     [apply = std::move(apply)](const Json& data) -> asio::awaitable<Reply> {
                         co_return result(apply(data));
                     });
    };
    add("camera_mode_switch",
        [this](const Json& data) { return switch_mode(enum_value(data, "camera_mode", 0, 4)); });
    add("camera_photo_take", [this](const Json&) { return take_photo(); });
    add("camera_photo_stop", [this](const Json&) { return stop_photo(); });
    add("camera_recording_start", [this](const Json&) { return start_recording(); });
    add("camera_recording_stop", [this](const Json&) { return stop_recording(); });
    add("camera_screen_split", [&camera](const Json& data) {
        camera.screen_split_enable = flag(data, "enable", camera.screen_split_enable);
        return Result::ok;
    });
    add("photo_storage_set", [&camera](const Json& data) {
        camera.photo_storage_settings = storage(data, "photo_storage_settings");
        return Result::ok;
    });
    add("video_storage_set", [&camera](const Json& data) {
        camera.video_storage_settings = storage(data, "video_storage_settings");
        return Result::ok;
    });
    add("camera_exposure_mode_set", [&camera](const Json& data) {
        auto& settings = lens(camera, data);
        settings.exposure_mode = enum_value(data, "exposure_mode", 1, 4);
        return Result::ok;
    });
    add("camera_exposure_set", [&camera](const Json& data) {
        auto& settings = lens(camera, data);
        // ExposureValueEnum: 1 is -5.0 EV, 16 is 0 EV, 31 is +5.0 EV.
        settings.exposure_value = enum_value(data, "exposure_value", 1, 31);
        return Result::ok;
    });
    add("camera_focus_mode_set", [&camera](const Json& data) {
        auto& settings = lens(camera, data);
        settings.focus_mode = enum_value(data, "focus_mode", 0, 2);
        return Result::ok;
    });
    add("camera_focus_value_set", [&camera](const Json& data) {
        auto& settings = lens(camera, data);
        settings.focus_value =
            std::clamp(static_cast<int>(integer(data, "focus_value", settings.focus_value)),
                       device::focus_value_min, device::focus_value_max);
        return Result::ok;
    });
    add("camera_point_focus_action", [&camera](const Json& data) {
        auto& settings = lens(camera, data);
        std::ignore = fraction(data, "x", 0.5);
        std::ignore = fraction(data, "y", 0.5);
        // FocusStateEnum 2: the focus succeeded.
        settings.focus_state = 2;
        return Result::ok;
    });
    add("ir_metering_mode_set", [&camera](const Json& data) {
        camera.ir.mode = enum_value(data, "mode", 0, 2);
        return Result::ok;
    });
    add("ir_metering_point_set", [&camera](const Json& data) {
        camera.ir.point_x = fraction(data, "x", camera.ir.point_x);
        camera.ir.point_y = fraction(data, "y", camera.ir.point_y);
        return Result::ok;
    });
    add("ir_metering_area_set", [&camera](const Json& data) {
        camera.ir.area_x = fraction(data, "x", camera.ir.area_x);
        camera.ir.area_y = fraction(data, "y", camera.ir.area_y);
        camera.ir.area_width = fraction(data, "width", camera.ir.area_width);
        camera.ir.area_height = fraction(data, "height", camera.ir.area_height);
        return Result::ok;
    });
}

std::optional<Result> CameraControl::drc_set(std::string_view method, const Json& data) {
    auto& camera = ctx_.drone.camera;
    if (method == "drc_camera_mode_switch")
        return switch_mode(enum_value(data, "camera_mode", 0, 4, camera.camera_mode));
    if (method == "drc_linkage_zoom_set") {
        camera.linkage_zoom_state = integer(data, "state", camera.linkage_zoom_state ? 1 : 0) != 0;
    } else if (method == "drc_video_resolution_set") {
        camera.video_resolution =
            enum_value(data, "video_resolution", 0, 1, camera.video_resolution);
    } else if (method == "drc_photo_storage_set") {
        camera.photo_storage_settings = storage(data, "photo_storage_settings");
    } else if (method == "drc_video_storage_set") {
        camera.video_storage_settings = storage(data, "video_storage_settings");
    } else if (method == "drc_interval_photo_set") {
        camera.interval_photo_interval = number(data, "interval", camera.interval_photo_interval);
    } else if (method == "drc_camera_iso_set") {
        auto& settings = lens(camera, data);
        settings.iso = setting(data, "iso_value", settings.iso);
    } else if (method == "drc_camera_shutter_set") {
        auto& settings = lens(camera, data);
        settings.shutter_speed = setting(data, "shutter_value", settings.shutter_speed);
    } else if (method == "drc_camera_aperture_value_set") {
        auto& settings = lens(camera, data);
        settings.aperture_value = setting(data, "aperture_value", settings.aperture_value);
    } else if (method == "drc_camera_mechanical_shutter_set") {
        auto& settings = lens(camera, data);
        settings.mechanical_shutter_state =
            setting(data, "mechanical_shutter_state", settings.mechanical_shutter_state);
    } else if (method == "drc_camera_dewarping_set") {
        auto& settings = lens(camera, data);
        settings.dewarping_state = setting(data, "dewarping_state", settings.dewarping_state);
    } else if (!method.starts_with("drc_camera_")) {
        return std::nullopt;
    }
    return Result::ok;
}

void CameraControl::refresh() {
    auto& camera = ctx_.drone.camera;
    if (camera.recording_state == 0) return;
    const int left = device::record_capacity_s - camera.recorded_s;
    camera.record_time =
        std::min(left, static_cast<int>(std::floor(ctx_.clock().monotonic() - record_start_)));
    if (camera.record_time >= left) {
        logger()->info("camera: storage full, recording stops");
        camera.recording_state = 0;
        camera.recorded_s += camera.record_time;
        camera.record_time = 0;
    }
}

Result CameraControl::switch_mode(int mode) {
    auto& camera = ctx_.drone.camera;
    if (camera.photo_state != 0 || camera.recording_state != 0) return Result::camera_mode_failed;
    camera.camera_mode = mode;
    return Result::ok;
}

Result CameraControl::take_photo() {
    auto& drone = ctx_.drone;
    if (drone.camera.photo_state != 0 || drone.remain_photo_num() <= 0) return Result::photo_failed;
    drone.camera.photo_state = 1;
    photo_ = ctx_.spawn(photo({
                            .latitude = drone.latitude,
                            .longitude = drone.longitude,
                            .height = drone.height,
                            .heading = drone.camera_heading(),
                            .gimbal_pitch = drone.gimbal_pitch,
                            .time = ctx_.clock().now(),
                        }),
                        "camera-photo");
    return Result::ok;
}

asio::awaitable<void> CameraControl::photo(wayline::Shot shot) {
    co_await ctx_.clock().sleep(photo_seconds);
    auto& camera = ctx_.drone.camera;
    camera.photo_state = 0;
    manual_.store_photo(shot);
    // The cloud reads this event as EventsDataRequest: result next to output.
    co_await ctx_.emit_event("camera_photo_take_progress",
                             {
                                 {"result", Result::ok},
                                 {"output",
                                  {
                                      {"ext", {{"camera_mode", camera.camera_mode}}},
                                      {"progress", {{"current_step", 0}, {"percent", 100}}},
                                      {"status", code::FlighttaskStatus::ok},
                                  }},
                             });
}

Result CameraControl::stop_photo() {
    auto& camera = ctx_.drone.camera;
    if (camera.photo_state != 0) {
        std::exchange(photo_, {}).cancel();
        camera.photo_state = 0;
    }
    return Result::ok;
}

Result CameraControl::start_recording() {
    auto& drone = ctx_.drone;
    auto& camera = drone.camera;
    if (camera.recording_state != 0 || camera.photo_state != 0 ||
        drone.remain_record_duration() <= 0) {
        return Result::recording_start_failed;
    }
    camera.camera_mode = 1;
    camera.recording_state = 1;
    camera.record_time = 0;
    record_start_ = ctx_.clock().monotonic();
    return Result::ok;
}

Result CameraControl::stop_recording() {
    auto& camera = ctx_.drone.camera;
    refresh();
    if (camera.recording_state == 0) return Result::recording_stop_failed;
    camera.recording_state = 0;
    camera.recorded_s += camera.record_time;
    camera.record_time = 0;
    return Result::ok;
}

}  // namespace docksim::services
