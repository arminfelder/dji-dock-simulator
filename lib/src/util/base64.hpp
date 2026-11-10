#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace docksim::base64 {

[[nodiscard]] std::string encode(std::span<const std::uint8_t> data);

// Drops every character outside the base64 alphabet, then decodes.
// Empty if the remaining length or padding is invalid.
[[nodiscard]] std::optional<std::vector<std::uint8_t>> decode_lenient(std::string_view text);

}  // namespace docksim::base64
