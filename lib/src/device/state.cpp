#include "device/state.hpp"

#include "device/catalog.hpp"
#include "device/geo.hpp"
#include "util/numeric.hpp"

#include <algorithm>
#include <cmath>

namespace docksim::device {

double DockState::uniform(double a, double b) {
    return std::uniform_real_distribution<double>(a, b)(rng);
}

int DockState::randint(int a, int b) {
    return std::uniform_int_distribution<int>(a, b)(rng);
}

DroneState::DroneState(const DeviceConfig& c)
    : cfg(&c),
      home(Site::of(c)),
      latitude(c.latitude),
      longitude(c.longitude),
      elevation(c.altitude),
      payload_index(catalog::camera_index(c.drone_model)),
      firmware_version(c.drone_firmware_version) {}

int DroneState::battery_percent() const {
    return std::clamp(static_cast<int>(std::nearbyint(battery_level)), 0, 100);
}

double DroneState::camera_heading() const {
    return num::floor_mod(attitude_head + gimbal_yaw, 360.0);
}

double DroneState::home_distance() const {
    return units::in_metres(
        geo::haversine(geo::point(home.latitude, home.longitude), geo::point(latitude, longitude)));
}

int DroneState::remain_photo_num() const {
    return std::max(0, photo_capacity - camera_photos);
}

int DroneState::remain_record_duration() const {
    return std::max(0, record_capacity_s - camera.recorded_s - camera.record_time);
}

void DroneState::reset_to_dock() {
    latitude = home.latitude;
    longitude = home.longitude;
    height = 0.0;
    gimbal_pitch = -90.0;
    gimbal_yaw = 0.0;
    horizontal_speed = 0.0;
    vertical_speed = 0.0;
    course = 0.0;
    mode_code = code::DroneMode::standby;
}

void charge(DockState& dock, DroneState& drone, double seconds) {
    if (dock.drone_in_dock != 0 && dock.charging_enabled && drone.battery_level < 100.0) {
        drone.battery_level =
            std::min(100.0, drone.battery_level + seconds / charge_seconds_per_percent);
        dock.drone_charge_state = 1;
    } else {
        dock.drone_charge_state = 0;
    }
    dock.drone_charge_percent = drone.battery_percent();
}

void drain(DroneState& drone, double seconds, double factor) {
    drone.battery_level = std::max(
        battery_floor_percent, drone.battery_level - seconds * factor / drain_seconds_per_percent);
}

}  // namespace docksim::device
