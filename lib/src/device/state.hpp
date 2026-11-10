#pragma once

#include "protocol/codes.hpp"

#include <docksim/config.hpp>

#include <random>
#include <string>
#include <vector>

// Dock and aircraft state: plain data plus the physics that changes it. The
// wire form is built in protocol/payloads.
namespace docksim::device {

// A dock charges the pack from 20 to 90 percent in about 25 minutes.
inline constexpr double charge_seconds_per_percent = 21.0;
// About 1 percent for each 18 s of flight, so a full pack lasts about 30 minutes.
inline constexpr double drain_seconds_per_percent = 18.0;
// The aircraft lands before the pack is empty.
inline constexpr double battery_floor_percent = 5.0;
// Battery percent at which the aircraft starts return to home, and at which it lands.
inline constexpr int return_home_power_percent = 30;
inline constexpr int landing_power_percent = 15;

// Position of the dock. The aircraft takes off from it and lands on it.
struct Site {
    double latitude = 0.0;
    double longitude = 0.0;
    // Metres above the ellipsoid.
    double altitude = 0.0;

    [[nodiscard]] static Site of(const DeviceConfig& c) {
        return {.latitude = c.latitude, .longitude = c.longitude, .altitude = c.altitude};
    }
};

// Weather at the dock. While noise is true, the OSD sends random calm values.
struct Weather {
    bool noise = true;
    // Metres per second.
    double wind_speed = 0.0;
    // Degrees clockwise from north. The wind comes from this direction.
    double wind_direction = 0.0;
    // RainfallEnum: 0 none, 1 light, 2 moderate, 3 heavy.
    int rainfall = 0;
    // Degrees Celsius.
    double temperature = 15.0;
    // Percent.
    int humidity = 50;
};

// Satellite fix. fix is is_calibration in the dock OSD and is_fixed in the aircraft OSD.
struct Gnss {
    int fix = 0;
    int quality = 0;
    int gps_number = 0;
    int rtk_number = 0;
};

struct DockState {
    explicit DockState(const DeviceConfig& c, std::uint64_t seed = std::random_device{}())
        : cfg(&c), site(Site::of(c)), firmware_version(c.dock_firmware_version), rng(seed) {}

    const DeviceConfig* cfg;
    Site site;
    Weather weather;
    Gnss gnss{.fix = 1, .quality = 5, .gps_number = 18, .rtk_number = 22};
    code::DockMode mode_code = code::DockMode::idle;
    int cover_state = 0;
    int putter_state = 0;
    int drone_in_dock = 1;
    int drone_charge_percent = 100;
    int drone_charge_state = 0;
    int job_number = 0;
    std::int64_t acc_time = 1'800'000;
    code::FlighttaskStep flighttask_step_code = code::FlighttaskStep::no_task;
    int flighttask_prepare_capacity = 1;
    int media_remain_upload = 0;
    int alarm_state = 0;
    int supplement_light_state = 0;
    double network_rate = 12.5;
    int emergency_stop_state = 0;
    // Fault injection: the cover does not open, so the dock does not launch.
    bool cover_fault = false;
    code::DrcState drc_state = code::DrcState::disconnected;
    // False while device_reboot holds the dock silent.
    bool online = true;
    bool drone_powered = true;
    // False after charge_close, until charge_open.
    bool charging_enabled = true;
    // 1 schedule mode, 2 standby mode.
    int battery_store_mode = 1;
    // 0 no maintenance, 1 waiting, 2 in maintenance.
    int battery_maintenance_state = 0;
    // 0 idle, 1 cooling, 2 heating, 3 dehumidification.
    int air_conditioner_state = 0;
    // 0 SDR only, 1 4G enhanced.
    int link_workmode = 0;
    std::string firmware_version;
    bool firmware_upgrading = false;
    // Weather noise for the OSD.
    std::mt19937_64 rng;

    [[nodiscard]] double uniform(double a, double b);
    [[nodiscard]] int randint(int a, int b);
};

// The aircraft takes this many photos and records this long on an empty card.
inline constexpr int photo_capacity = 4096;
inline constexpr int record_capacity_s = 3600;

// Exposure and focus of one visible lens. Every value is the int code of the
// cloud enum: ExposureModeEnum, CameraIsoEnum, CameraShutterSpeedEnum,
// ExposureValueEnum, CameraApertureEnum and FocusModeEnum.
struct LensSettings {
    int exposure_mode = 1;
    int iso = 0;
    int shutter_speed = 60;
    int exposure_value = 16;
    int aperture_value = 0;
    int focus_mode = 1;
    int focus_state = 0;
    int focus_value = 40;
    int dewarping_state = 0;
    int mechanical_shutter_state = 0;
};

// Focus values that the zoom and wide lenses accept.
inline constexpr int focus_value_min = 33;
inline constexpr int focus_value_max = 64;

// Point and area of IR spot metering, 0..1 from the top left of the frame.
struct IrMetering {
    int mode = 0;
    double point_x = 0.5;
    double point_y = 0.5;
    double area_x = 0.25;
    double area_y = 0.25;
    double area_width = 0.5;
    double area_height = 0.5;
};

struct CameraState {
    // CameraModeEnum: 0 photo, 1 video, 2 low light, 3 panorama, 4 timed shot.
    int camera_mode = 0;
    // CameraStateEnum: 0 idle, 1 working.
    int photo_state = 0;
    int recording_state = 0;
    // Seconds of the recording that runs.
    int record_time = 0;
    // Seconds of every recording that has ended.
    int recorded_s = 0;
    bool screen_split_enable = false;
    std::vector<std::string> photo_storage_settings{"current"};
    std::vector<std::string> video_storage_settings{"current"};
    // 0 1080p, 1 4K. The firmware sends an int.
    int video_resolution = 0;
    // PhotoSizeEnum, 0 is the default size.
    int photo_size = 0;
    // Seconds between two timed shots.
    double interval_photo_interval = 2.0;
    bool linkage_zoom_state = false;
    LensSettings wide;
    LensSettings zoom;
    IrMetering ir;
};

struct DroneState {
    explicit DroneState(const DeviceConfig& c);

    const DeviceConfig* cfg;
    // The dock position. It changes only while the aircraft is in the dock.
    Site home;
    Gnss gnss{.fix = 2, .quality = 5, .gps_number = 19, .rtk_number = 24};
    // The wind that the aircraft measures, for the drc OSD.
    double wind_speed = 0.0;
    double wind_direction = 0.0;
    code::DroneMode mode_code = code::DroneMode::standby;
    double latitude = 0.0;
    double longitude = 0.0;
    // Metres above the dock.
    double height = 0.0;
    double elevation = 0.0;
    // Degrees clockwise from north.
    double attitude_head = 0.0;
    // Degrees, -90 is nadir.
    double gimbal_pitch = -90.0;
    // Degrees clockwise, relative to the aircraft heading.
    double gimbal_yaw = 0.0;
    // Zoom camera factor against the 24 mm wide lens. The tele lens starts at 7x.
    double zoom_factor = 7.0;
    double ir_zoom_factor = 1.0;
    double horizontal_speed = 0.0;
    double vertical_speed = 0.0;
    // Degrees clockwise from north of the horizontal motion.
    double course = 0.0;
    double battery_level = 100.0;
    double total_flight_distance = 0.0;
    // Seconds. Kept as double, so that ticks below 1 s add up.
    double total_flight_time = 0.0;
    // Metres above the dock.
    double rth_altitude = 100.0;
    int night_lights_state = 0;
    int stealth_state = 0;
    int gear = 1;
    int camera_photos = 0;
    CameraState camera;
    std::string payload_index;
    std::string firmware_version;
    bool firmware_upgrading = false;

    [[nodiscard]] int battery_percent() const;
    // Degrees clockwise from north that the camera looks at.
    [[nodiscard]] double camera_heading() const;
    [[nodiscard]] double home_distance() const;
    [[nodiscard]] int remain_photo_num() const;
    [[nodiscard]] int remain_record_duration() const;
    void reset_to_dock();
};

// Charge the pack while the aircraft sits in the dock.
void charge(DockState& dock, DroneState& drone, double seconds);
// factor above 1 drains faster, for example in strong wind.
void drain(DroneState& drone, double seconds, double factor = 1.0);

}  // namespace docksim::device
