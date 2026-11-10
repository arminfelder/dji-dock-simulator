#include "device/catalog.hpp"

#include <algorithm>
#include <array>
#include <format>
#include <stdexcept>
#include <utility>

namespace docksim::catalog {

namespace {

using namespace std::string_view_literals;

constexpr std::array devices{
    std::pair{"dock1"sv, DeviceType{.domain = domain_dock, .type = 1, .sub_type = 0}},
    std::pair{"dock2"sv, DeviceType{.domain = domain_dock, .type = 2, .sub_type = 0}},
    std::pair{"dock3"sv, DeviceType{.domain = domain_dock, .type = 3, .sub_type = 0}},
    std::pair{"m30"sv, DeviceType{.domain = domain_drone, .type = 67, .sub_type = 0}},
    std::pair{"m30t"sv, DeviceType{.domain = domain_drone, .type = 67, .sub_type = 1}},
    std::pair{"m3d"sv, DeviceType{.domain = domain_drone, .type = 91, .sub_type = 0}},
    std::pair{"m3td"sv, DeviceType{.domain = domain_drone, .type = 91, .sub_type = 1}},
    std::pair{"m4d"sv, DeviceType{.domain = domain_drone, .type = 100, .sub_type = 0}},
    std::pair{"m4td"sv, DeviceType{.domain = domain_drone, .type = 100, .sub_type = 1}},
};

struct Camera {
    std::string_view model;
    std::string_view index;
    std::string_view name;
    bool thermal;
};

constexpr std::array cameras{
    Camera{.model = "m30", .index = "52-0-0", .name = "M30 Camera", .thermal = false},
    Camera{.model = "m30t", .index = "53-0-0", .name = "M30T Camera", .thermal = true},
    Camera{.model = "m3d", .index = "80-0-0", .name = "Mavic 3E Camera", .thermal = false},
    Camera{.model = "m3td", .index = "81-0-0", .name = "Mavic 3T Camera", .thermal = true},
    Camera{.model = "m4d", .index = "98-0-0", .name = "M4D Camera", .thermal = false},
    Camera{.model = "m4td", .index = "99-0-0", .name = "M4TD Camera", .thermal = true},
};

constexpr std::array visual_lenses{"normal"sv, "wide"sv, "zoom"sv};
constexpr std::array thermal_lenses{"normal"sv, "wide"sv, "zoom"sv, "ir"sv};

const Camera* find_camera(std::string_view model) {
    const auto* it = std::ranges::find(cameras, model, &Camera::model);
    return it == cameras.end() ? nullptr : it;
}

}  // namespace

const DeviceType& device(std::string_view model) {
    const auto* it = std::ranges::find(devices, model, [](const auto& p) { return p.first; });
    if (it == devices.end()) throw std::out_of_range(std::format("unknown device model {}", model));
    return it->second;
}

std::string model_key(std::string_view model) {
    const auto& d = device(model);
    return std::format("{}-{}-{}", d.domain, d.type, d.sub_type);
}

std::string_view camera_index(std::string_view drone_model) {
    const auto* camera = find_camera(drone_model);
    if (camera != nullptr) return camera->index;
    const auto* fallback = find_camera("m3d");
    if (fallback == nullptr) throw std::logic_error("camera catalog lacks the m3d fallback");
    return fallback->index;
}

std::string_view camera_name(std::string_view drone_model) {
    const auto* camera = find_camera(drone_model);
    return camera != nullptr ? camera->name : "DJI Camera";
}

std::span<const std::string_view> switchable_video_types(std::string_view drone_model) {
    const auto* camera = find_camera(drone_model);
    if (camera == nullptr) return {};
    if (camera->thermal) return thermal_lenses;
    return visual_lenses;
}

std::string payload_model_key(std::string_view payload_index) {
    const auto first = payload_index.find('-');
    const auto second = payload_index.find('-', first + 1);
    if (first == std::string_view::npos) {
        throw std::invalid_argument(std::format("bad payload index {}", payload_index));
    }
    return std::format("{}-{}-{}", domain_payload, payload_index.substr(0, first),
                       payload_index.substr(first + 1, second - first - 1));
}

}  // namespace docksim::catalog
