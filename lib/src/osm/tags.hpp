#pragma once

#include "util/json.hpp"

#include <algorithm>
#include <cctype>
#include <optional>
#include <string>
#include <string_view>

// Reads of OSM tag objects. Tag values are text; anything else is dumped.
namespace docksim::osm {

[[nodiscard]] inline std::optional<std::string> tag(const Json& tags, std::string_view key) {
    if (!tags.is_object()) return std::nullopt;
    const auto it = tags.find(key);
    if (it == tags.end() || it->is_null()) return std::nullopt;
    return it->is_string() ? it->get<std::string>() : it->dump();
}

[[nodiscard]] inline std::string tag_or(const Json& tags, std::string_view key,
                                        std::string_view fallback) {
    return tag(tags, key).value_or(std::string(fallback));
}

[[nodiscard]] inline bool has_tag(const Json& tags, std::string_view key) {
    return tags.is_object() && tags.contains(key);
}

[[nodiscard]] inline std::string lower(std::string s) {
    std::ranges::transform(s, s.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

[[nodiscard]] inline std::string trim(std::string_view s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front())) != 0)
        s.remove_prefix(1);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())) != 0)
        s.remove_suffix(1);
    return std::string(s);
}

}  // namespace docksim::osm
