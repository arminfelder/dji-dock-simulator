#pragma once

#include "util/json.hpp"

#include <cstdint>

// Codes of the DJI Cloud API thing model and of the DJI error code table
// (reference/Cloud-API-Doc/docs/en/71.error-code.md). On the wire each code
// is its plain value.
namespace docksim::code {

// Dock property mode_code (Dock 2 thing model).
enum class DockMode : std::uint8_t {
    idle = 0,
    on_site_debug = 1,
    remote_debug = 2,
    upgrading = 3,
    working = 4,
    // Not in the Dock 2 thing model. The web UI names 5 ToBeCalibrated (Dock 3).
    to_be_calibrated = 5,
};

// Aircraft property mode_code.
enum class DroneMode : std::uint8_t {
    standby = 0,
    prepare = 1,
    ready = 2,
    manual = 3,
    auto_takeoff = 4,
    wayline = 5,
    panorama = 6,
    active_track = 7,
    adsb_avoidance = 8,
    auto_rth = 9,
    auto_landing = 10,
    force_landing = 11,
    three_prop_landing = 12,
    upgrading = 13,
    disconnected = 14,
    apas = 15,
    virtual_stick = 16,
    live_flight_control = 17,
};

// flighttask_progress ext.wayline_mission_state.
enum class WaylineState : std::uint8_t {
    prepare = 2,
    triggered = 4,
    entering = 5,
    executing = 6,
    interrupted = 7,
    recovering = 8,
    complete = 9,
};

// flighttask_progress progress.current_step, and the dock flighttask_step_code.
enum class FlighttaskStep : std::uint16_t {
    initial = 0,
    dock_working = 6,
    cover_open = 7,
    wait_flight_system = 9,
    download_kmz = 17,
    upload_kmz = 18,
    set_home = 23,
    trigger_wayline = 24,
    landing = 27,
    cover_close = 28,
    dock_idle = 29,
    media_count = 33,
    notify_result = 35,
    // Dock flighttask_step_code while no task runs.
    no_task = 255,
    none = 65534,
};

enum class FlighttaskStatus : std::uint8_t { sent, in_progress, paused, ok, canceled, failed };

NLOHMANN_JSON_SERIALIZE_ENUM(FlighttaskStatus, {
                                                   {FlighttaskStatus::sent, "sent"},
                                                   {FlighttaskStatus::in_progress, "in_progress"},
                                                   {FlighttaskStatus::paused, "paused"},
                                                   {FlighttaskStatus::ok, "ok"},
                                                   {FlighttaskStatus::canceled, "canceled"},
                                                   {FlighttaskStatus::failed, "failed"},
                                               })

// live_start_push url_type. The cloud maps it in UrlTypeEnum.
enum class UrlType : std::uint8_t { agora = 0, rtmp = 1, rtsp = 2, gb28181 = 3, webrtc = 4 };

// drc_status_notify drc_state, as the real dock sends it.
enum class DrcState : std::uint8_t { disconnected = 0, connected = 2 };

// status of takeoff_to_point_progress and fly_to_point_progress. The cloud
// maps it in TakeoffStatusEnum and FlyToStatusEnum.
enum class FlyStatus : std::uint8_t {
    task_ready,
    wayline_progress,
    wayline_ok,
    wayline_failed,
    wayline_cancel,
    task_finish,
};

NLOHMANN_JSON_SERIALIZE_ENUM(FlyStatus, {
                                            {FlyStatus::task_ready, "task_ready"},
                                            {FlyStatus::wayline_progress, "wayline_progress"},
                                            {FlyStatus::wayline_ok, "wayline_ok"},
                                            {FlyStatus::wayline_failed, "wayline_failed"},
                                            {FlyStatus::wayline_cancel, "wayline_cancel"},
                                            {FlyStatus::task_finish, "task_finish"},
                                        })

// joystick_invalid_notify reason. The cloud maps it in JoystickInvalidReasonEnum.
enum class JoystickInvalidReason : std::uint8_t {
    rc_lost = 0,
    low_battery_return = 1,
    low_battery_landing = 2,
    near_restricted_zone = 3,
    rc_takeover = 4,
};

// poi_status_notify reason. The cloud maps it in PoiStatusReasonEnum.
enum class PoiReason : std::uint8_t {
    normal = 0,
    unadapted_payload = 1,
    camera_mode_not_supported = 2,
    illegal_cmd = 3,
    positioning_failed = 4,
    on_the_ground = 5,
    drone_mode_error = 6,
    not_available_mode = 7,
    rc_disconnected = 8,
};

// in_flight_wayline_progress status. The cloud InFlightWaylineStatusEnum has
// an int JsonValue, so the status is a number on the wire.
enum class InFlightStatus : std::uint8_t {
    uploading = 1,
    uploaded = 2,
    progress = 3,
    paused = 4,
    cancel = 5,
    ok = 6,
    failed = 7,
    timeout = 8,
};

// reason of flight_areas_sync_progress and offline_map_sync_progress. The
// cloud maps it in FlightAreaSyncReasonEnum and OfflineMapSyncReasonEnum.
enum class SyncReason : std::uint8_t {
    success = 0,
    parse_failed = 1,
    download_failed = 3,
    // The file content did not load.
    load_failed = 8,
    // FlightAreaSyncReasonEnum only.
    checksum_failed = 12,
};

// result of a services_reply or an event. The comment after each member is
// the text of the DJI error table.
enum class Result : std::int32_t {
    ok = 0,
    live_unsupported = 13004,      // FUNCTION_NOT_SUPPORT (cloud LiveErrorCodeEnum)
    live_no_stream = 13008,        // NO_STREAM_DATA (cloud LiveErrorCodeEnum)
    not_supported = 314000,        // Operation not supported. Check device status
    param_error = 514171,          // Cloud command parameter error
    drc_connection_lost = 514303,  // Network error. Disconnected.
    rth_failed = 316024,     // Aircraft failed to return to home. Check if aircraft has taken off
    fly_to_failed = 319022,  // FlyTo task failed. Try again later
    rth_cancel_failed = 319043,      // Failed to cancel RTH. Try again later
    dock_busy = 514116,              // Unable to perform operation. Dock is executing other command
    route_download_failed = 314013,  // Dock unable to obtain flight route file and perform task
    pause_failed = 319020,           // Failed to pause flight task. Try again later
    resume_failed = 319042,          // Failed to resume flight. Try again later
    task_started = 321257,           // Task in progress. Failed to start task again
    task_not_started = 321259,       // Task not started. Unable to stop task
    photo_failed = 327006,           // Failed to take photo
    recording_start_failed = 327007,  // Failed to start recording
    recording_stop_failed = 327008,   // Failed to stop recording
    camera_mode_failed = 327009,      // Failed to switch camera modes
    remote_debug_active = 514146,  // Dock in remote debugging mode. Unable to perform task
    debug_refused_upgrading = 514147,  // Unable to start remote debugging. Updating device firmware
    debug_refused_task = 514148,       // Task in progress. Dock unable to enter remote debugging
    restarting = 514150,               // Restarting device. Unable to perform task
    reboot_refused_upgrading = 514151,  // Updating device firmware. Unable to restart device
    remote_debug_required = 514153,     // Dock exited remote debugging mode. Unable to perform
    drone_power_on_failed = 514109,     // Failed to power on aircraft. Check if aircraft is inside
    ota_upgrading = 312014,             // Updating device firmware. Wait until update completed
    ota_dock_busy = 312015,             // Unable to update device firmware. Dock busy
    ota_drone_not_in_dock = 312027,     // Failed to update firmware. Aircraft not detected inside
    log_upload_failed = 324019,         // Due to network anomalies ... the log upload has failed
    log_busy = 325009,                  // Dock is exporting logs to cloud. Try again later
    flight_areas_busy = 325010,         // Dock is updating custom flight area. Try again later
    offline_map_busy = 325011,          // Dock is updating offline map. Try again later
    target_restricted = 336513,         // FlyTo destination in Restricted Zone
    battery_low = 514103,               // Aircraft battery level lower than 30%. Unable to perform
    cover_open_failed = 514107,         // Failed to open dock cover. Make sure dock cover is clear
    emergency_stop = 514121,            // Emergency stop button pressed down. Release button
    heavy_rain = 514134,                // Heavy rainfall. Unable to perform task. Try again later
    wind_too_high = 514135,             // Unable to perform task. Wind speed too high

    // The codes below are wrong or missing in the DJI table. They stay until
    // the web UI reaction to a replacement is checked.
    unknown_service = 314001,          // Failed to distribute task. Try again later
    task_busy = 514001,                // not in the table
    wayline_download_failed = 514002,  // not in the table
    task_not_prepared = 514003,        // not in the table
    task_running = 514004,             // not in the table
    task_not_running = 514005,         // not in the table
    drc_bad_broker = 514100,           // Dock error. Restart dock and try again
    drc_broker_refused = 514101,       // Failed to push driving rods into place
};

}  // namespace docksim::code
