#pragma once

namespace docksim::image {

// OpenCV logs at INFO to stderr, outside spdlog. Call this before the first
// OpenCV function: then OpenCV logs only its warnings.
void quiet_opencv();

}  // namespace docksim::image
