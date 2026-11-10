#pragma once

#include "device/state.hpp"
#include "util/json.hpp"

#include <chrono>
#include <string>
#include <vector>

// Wire form of the dock and aircraft state: osd and state property groups.
// Every rule here is in docs/protocol-quirks.md and pinned by the golden tests.
namespace docksim::payloads {

// The three dock osd messages, split as the real dock splits them. The dock
// sends osd_msg_half (mode_code and the dock state) and osd_msg_next
// (job_number, power and maintenance). It sends drc_state, wireless_link and
// the task keys in a third message. The cloud ignores a dock osd that has
// job_number, and takes drc_state only from a message without mode_code.
[[nodiscard]] std::vector<Json> dock_osd(device::DockState& dock);

// One object per property group. The cloud routes a state message by the
// first key group it recognises, so groups must not share a message.
[[nodiscard]] std::vector<Json> dock_state(const device::DockState& dock);

[[nodiscard]] Json drone_osd(const device::DroneState& drone);

// The backend reads the control-source group. payloads entries carry only
// sn, payload_index and control_source.
[[nodiscard]] std::vector<Json> drone_state(const device::DroneState& drone);
[[nodiscard]] Json control_source_group(const device::DroneState& drone);

// The battery object of the aircraft osd. drc_batteries_info_push sends the same object.
[[nodiscard]] Json drone_battery(const device::DroneState& drone);
// drc_batteries_info_push: drone_battery with the extra keys of the firmware.
[[nodiscard]] Json drc_batteries_info(const device::DroneState& drone);

// Obstacle distance with no obstacle in sight, millimetres.
inline constexpr int hsi_clear_mm = 60'000;

// data of the drc/up pushes. Heights in osd_info_push: height is ellipsoid,
// elevation is above the dock. speed_x is north, speed_y east, speed_z up.
[[nodiscard]] Json drc_osd_info(const device::DroneState& drone);
// The firmware sends around_distances. The cloud reads around_distance.
[[nodiscard]] Json drc_hsi_info(const device::DroneState& drone);
[[nodiscard]] Json drc_delay_info(const device::DroneState& drone);
[[nodiscard]] Json drc_drone_state(const device::DroneState& drone);

// drc_camera_state_push. video_resolution and linkage_zoom_state are ints, as
// in the firmware.
[[nodiscard]] Json drc_camera_state(const device::DroneState& drone);
// drc_camera_osd_info_push. The cloud reads wide_lense and zoom_lense of the
// stored push during a mission and unboxes wide_dewarping_state with no null check.
[[nodiscard]] Json drc_camera_osd_info(const device::DroneState& drone);

// The laser range finder reaches this far, metres.
inline constexpr double laser_range_max_m = 1200.0;

// Where the camera axis meets the plane of the dock. Not checked against
// firmware: error_state 0 has a target, 1 has none.
struct MeasureTarget {
    double latitude = 0.0;
    double longitude = 0.0;
    // Ellipsoid height, metres.
    double altitude = 0.0;
    // Slant range, metres.
    double distance = 0.0;
    int error_state = 1;
};
[[nodiscard]] MeasureTarget measure_target(const device::DroneState& drone);

// compatible_status true means compatible. The cloud unboxes it with no null check.
[[nodiscard]] Json firmware_group(const std::string& version, bool upgrading);

struct FileUpload {
    std::string file_name;
    std::string object_key;
    std::string path;
    std::string flight_id;
    std::string drone_model_key;
    std::string payload_model_key;
    double latitude;
    double longitude;
    double absolute_altitude;
    double relative_altitude;
    int expected;
    int uploaded;
    double gimbal_yaw = -90.0;
    std::chrono::system_clock::time_point created_time;
};

// The backend unboxes result with no null check.
// Without result, the cloud throws and does not count the file.
[[nodiscard]] Json file_upload_callback(const FileUpload& f);

}  // namespace docksim::payloads
