#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace docksim::image {

using Rgb = std::array<std::uint8_t, 3>;

// 8-bit RGB, rows top to bottom, no padding.
class RgbImage {
public:
    RgbImage() = default;
    RgbImage(int width, int height, Rgb fill = {0, 0, 0});

    [[nodiscard]] int width() const noexcept { return width_; }
    [[nodiscard]] int height() const noexcept { return height_; }
    [[nodiscard]] bool empty() const noexcept { return pixels_.empty(); }
    [[nodiscard]] std::span<std::uint8_t> bytes() noexcept { return pixels_; }
    [[nodiscard]] std::span<const std::uint8_t> bytes() const noexcept { return pixels_; }

    // Pointer to the 3 bytes of one pixel. No bounds check.
    [[nodiscard]] std::uint8_t* pixel(int x, int y) noexcept {
        return pixels_.data() + (static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) +
                                 static_cast<std::size_t>(x)) *
                                    3U;
    }
    [[nodiscard]] const std::uint8_t* pixel(int x, int y) const noexcept {
        return pixels_.data() + (static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) +
                                 static_cast<std::size_t>(x)) *
                                    3U;
    }

    // Out-of-range coordinates are ignored.
    void set(int x, int y, Rgb color) noexcept;
    // alpha 0..255 blends color over the pixel.
    void blend(int x, int y, Rgb color, int alpha) noexcept;

    friend bool operator==(const RgbImage&, const RgbImage&) = default;

private:
    int width_ = 0;
    int height_ = 0;
    std::vector<std::uint8_t> pixels_;
};

// Inclusive corners, one pixel wide.
void rectangle_outline(RgbImage& img, int x0, int y0, int x1, int y1, Rgb color);
void fill_rectangle(RgbImage& img, int x0, int y0, int x1, int y1, Rgb color, int alpha = 255);

struct TextBox {
    int width;
    int height;
};

// DejaVu Sans Mono, embedded. Lines split at '\n'. x, y is the top left.
void draw_text(RgbImage& img, int x, int y, std::string_view text, float pixel_height, Rgb color);
[[nodiscard]] TextBox measure_text(std::string_view text, float pixel_height);

[[nodiscard]] std::vector<std::byte> encode_jpeg(const RgbImage& img, int quality);

// Lossless PNG, for test snapshots and debug frames.
[[nodiscard]] std::vector<std::byte> encode_png(const RgbImage& img);

// Scaled to width x height: area averaging to shrink, bilinear to enlarge.
[[nodiscard]] RgbImage resize(const RgbImage& src, int width, int height);

// JPEG or PNG to RGB. Empty image if the data is not a picture.
[[nodiscard]] RgbImage decode(std::span<const std::byte> data);

}  // namespace docksim::image
