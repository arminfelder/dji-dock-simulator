#pragma once

#include <span>
#include <string>
#include <string_view>

namespace docksim::catalog {

inline constexpr std::string_view domain_drone = "0";
inline constexpr std::string_view domain_payload = "1";
inline constexpr std::string_view domain_rc = "2";
inline constexpr std::string_view domain_dock = "3";

struct DeviceType {
    std::string_view domain;
    int type;
    int sub_type;
};

// Dock camera payload index. Position 7 is FPV, the only position the cloud
// accepts for a dock camera.
inline constexpr std::string_view dock_camera_index = "165-0-7";

// Diagonal field of view of each wide lens, degrees, about 24 mm equivalent.
inline constexpr double wide_dfov = 84.0;

// Throws std::out_of_range for a model that is not in the DJI product list.
[[nodiscard]] const DeviceType& device(std::string_view model);

// device_model_key: domain-type-subtype.
[[nodiscard]] std::string model_key(std::string_view model);

// Camera index in the type-subtype-gimbalindex form that video_id uses.
[[nodiscard]] std::string_view camera_index(std::string_view drone_model);
[[nodiscard]] std::string_view camera_name(std::string_view drone_model);

// Lens types the camera can switch to. The cloud rejects any other name.
[[nodiscard]] std::span<const std::string_view> switchable_video_types(
    std::string_view drone_model);

// The cloud gives payload_model_key to DeviceEnum.find, which throws if nothing matches.
[[nodiscard]] std::string payload_model_key(std::string_view payload_index);

}  // namespace docksim::catalog
