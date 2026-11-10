#pragma once

#include <cstddef>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// Hex digests and a stored ZIP archive, for downloaded and uploaded files.
namespace docksim::digest {

// Lowercase hex.
[[nodiscard]] std::string sha256_hex(std::span<const std::byte> data);
[[nodiscard]] std::string md5_hex(std::span<const std::byte> data);

[[nodiscard]] std::span<const std::byte> bytes(std::string_view text);

// A ZIP archive with one member per (name, content) pair. Throws std::runtime_error.
[[nodiscard]] std::vector<std::byte> zip(
    const std::vector<std::pair<std::string, std::string>>& members);

}  // namespace docksim::digest
