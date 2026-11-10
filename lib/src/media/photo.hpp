#pragma once

#include "image/image.hpp"
#include "media/uploads.hpp"

#include <cstddef>
#include <random>
#include <string>
#include <utility>
#include <vector>

// Photo files: JPEG with the EXIF and DJI XMP tags the backend reads, and a test pattern.
namespace docksim::media {

inline constexpr int jpeg_quality = 85;

[[nodiscard]] image::RgbImage synthetic_image(std::pair<int, int> size, const std::string& caption,
                                              std::uint64_t seed = std::random_device{}());

// JPEG with EXIF (Make, Model, dates, GPS) and the drone-dji XMP tags that DJI tools and the
// backend read.
[[nodiscard]] std::vector<std::byte> encode_photo(const image::RgbImage& img,
                                                  const PhotoMeta& meta);

}  // namespace docksim::media
