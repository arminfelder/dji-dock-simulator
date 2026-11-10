#include "wayline/wpml.hpp"

#include "log/log.hpp"
#include "util/numeric.hpp"

#include <miniz.h>
#include <pugixml.hpp>

#include <algorithm>
#include <cmath>
#include <format>
#include <memory>
#include <stdexcept>
#include <string>

namespace docksim::wayline {

namespace {

std::string_view local_name(const pugi::xml_node& node) {
    const std::string_view name = node.name();
    const auto colon = name.find(':');
    return colon == std::string_view::npos ? name : name.substr(colon + 1);
}

bool is_wpml(const pugi::xml_node& node, std::string_view local) {
    const std::string_view name = node.name();
    return name.contains(':') && local_name(node) == local;
}

// First descendant (depth first, document order) with this local name.
pugi::xml_node find(const pugi::xml_node& root, std::string_view local) {
    return root.find_node([&](const pugi::xml_node& n) { return is_wpml(n, local); });
}

// A child path such as waypointHeadingParam/waypointHeadingMode below root.
pugi::xml_node find_path(const pugi::xml_node& root, std::string_view parent,
                         std::string_view child) {
    pugi::xml_node found;
    root.find_node([&](const pugi::xml_node& n) {
        if (!is_wpml(n, parent)) return false;
        for (const auto c : n.children()) {
            if (is_wpml(c, child)) {
                found = c;
                return true;
            }
        }
        return false;
    });
    return found;
}

std::string trimmed(const pugi::xml_node& node) {
    const std::string text = node.text().get();
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

std::optional<double> first_double(const pugi::xml_node& root, std::string_view local) {
    const auto node = find(root, local);
    if (!node) return std::nullopt;
    return num::parse_double(node.text().get());
}

std::optional<Waypoint> read_placemark(const pugi::xml_node& placemark, int fallback_index) {
    const auto coordinates = placemark.find_node([](const pugi::xml_node& n) {
        return local_name(n) == "coordinates" && !trimmed(n).empty();
    });
    if (!coordinates) return std::nullopt;
    std::string text = trimmed(coordinates);
    std::ranges::replace(text, '\n', ' ');
    const auto comma = text.find(',');
    if (comma == std::string::npos) return std::nullopt;
    const auto second = text.find(',', comma + 1);
    const auto lon = num::parse_double(std::string_view(text).substr(0, comma));
    const auto lat =
        num::parse_double(std::string_view(text).substr(comma + 1, second - comma - 1));
    if (!lon || !lat) throw std::runtime_error(std::format("bad coordinates: {}", text));

    Waypoint w;
    w.longitude = *lon;
    w.latitude = *lat;
    w.height = first_double(placemark, "executeHeight")
                   .or_else([&] { return first_double(placemark, "ellipsoidHeight"); })
                   .value_or(30.0);
    w.index =
        static_cast<int>(std::trunc(first_double(placemark, "index").value_or(fallback_index)));
    w.speed = first_double(placemark, "waypointSpeed");
    w.take_photo = static_cast<bool>(placemark.find_node([](const pugi::xml_node& n) {
        if (!is_wpml(n, "actionActuatorFunc")) return false;
        const auto func = trimmed(n);
        return func == "takePhoto" || func == "orientedShoot";
    }));

    const auto mode = find_path(placemark, "waypointHeadingParam", "waypointHeadingMode");
    if (!mode.empty() && (trimmed(mode) == "fixed" || trimmed(mode) == "smoothTransition")) {
        const auto angle = find_path(placemark, "waypointHeadingParam", "waypointHeadingAngle");
        if (!angle.empty()) w.heading = num::parse_double(angle.text().get());
    }
    w.gimbal_pitch = first_double(placemark, "gimbalPitchRotateAngle").or_else([&] {
        return first_double(placemark, "waypointGimbalPitchAngle");
    });
    if (const auto oriented = first_double(placemark, "aircraftHeading")) w.heading = oriented;
    return w;
}

std::string pick_member(mz_zip_archive& zip) {
    std::vector<std::string> names;
    for (mz_uint i = 0; i < mz_zip_reader_get_num_files(&zip); ++i) {
        std::array<char, 1024> name{};
        mz_zip_reader_get_filename(&zip, i, name.data(), static_cast<mz_uint>(name.size()));
        names.emplace_back(name.data());
    }
    for (const auto* candidate : {"wpmz/waylines.wpml", "waylines.wpml"}) {
        if (std::ranges::contains(names, candidate)) return candidate;
    }
    for (const auto& name : names) {
        if (name.ends_with(".wpml")) return name;
    }
    for (const auto& name : names) {
        if (name.ends_with(".kml")) return name;
    }
    throw std::runtime_error("no wpml or kml member in archive");
}

}  // namespace

int Wayline::photo_count() const {
    return static_cast<int>(std::ranges::count_if(waypoints, &Waypoint::take_photo));
}

Wayline parse_kmz(std::span<const std::byte> blob) {
    mz_zip_archive zip{};
    if (mz_zip_reader_init_mem(&zip, blob.data(), blob.size(), 0) == 0) {
        throw std::runtime_error("wayline file is not a zip archive");
    }
    const std::unique_ptr<mz_zip_archive, decltype(&mz_zip_reader_end)> guard(&zip,
                                                                              &mz_zip_reader_end);
    const auto member = pick_member(zip);
    std::size_t size = 0;
    void* data = mz_zip_reader_extract_file_to_heap(&zip, member.c_str(), &size, 0);
    if (data == nullptr) throw std::runtime_error(std::format("cannot extract {}", member));
    const std::unique_ptr<void, decltype(&mz_free)> owned(data, &mz_free);
    return parse_wpml(std::string_view(static_cast<const char*>(data), size));
}

Wayline parse_wpml(std::string_view xml) {
    pugi::xml_document doc;
    const auto parsed = doc.load_buffer(xml.data(), xml.size());
    if (!parsed) throw std::runtime_error(std::format("wayline XML: {}", parsed.description()));
    const auto root = doc.document_element();

    Wayline wayline;
    wayline.speed = first_double(root, "autoFlightSpeed").value_or(10.0);
    wayline.takeoff_height = first_double(root, "takeOffSecurityHeight").value_or(30.0);

    for (const auto& node : root.select_nodes("//*[local-name()='Placemark']")) {
        if (auto point = read_placemark(node.node(), static_cast<int>(wayline.waypoints.size()))) {
            wayline.waypoints.push_back(*point);
        }
    }
    if (wayline.waypoints.empty()) throw std::runtime_error("wayline has no waypoints");
    log::get("dock_sim.wpml")
        ->info("wayline parsed: {} waypoints, speed {:.1f} m/s", wayline.waypoints.size(),
               wayline.speed);
    return wayline;
}

}  // namespace docksim::wayline
