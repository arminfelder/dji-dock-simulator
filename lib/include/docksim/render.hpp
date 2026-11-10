#pragma once

#include <docksim/config.hpp>

#include <filesystem>
#include <optional>
#include <string>

// One-off renders outside a running dock: a single photo, and a frame-rate bench.
namespace docksim {

struct PhotoRequest {
    double latitude = 0;
    double longitude = 0;
    // Metres above the camera position's ground (the home point is the camera position).
    double altitude = 60.0;
    double heading = 0.0;
    double pitch = -90.0;
    // ISO 8601; empty is now.
    std::optional<std::string> time;
    int width = 2000;
    int height = 1500;
    double dfov = 84.0;
};

// Render one photo with the media settings (tile URLs, cache, device) and
// write it as JPEG (or PNG for a .png file name). Throws std::runtime_error.
void render_photo(const MediaConfig& media, const PhotoRequest& request,
                  const std::filesystem::path& output);

struct BenchResult {
    std::string device;
    int width;
    int height;
    double mean_ms;
    double max_ms;
};

// Average frame time of a moving camera at 1280x720 and 1920x1080 over
// frames frames, after one warm-up frame.
[[nodiscard]] std::vector<BenchResult> bench(const MediaConfig& media, int frames = 20);

}  // namespace docksim
