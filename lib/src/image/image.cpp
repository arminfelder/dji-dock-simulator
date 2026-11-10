#include "image/image.hpp"

#include "image/opencv.hpp"

#include <opencv2/core/utils/logger.hpp>
#include <opencv2/imgproc.hpp>
#include <stb_image.h>
#include <stb_image_write.h>
#include <stb_truetype.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace docksim::image {

namespace {

constexpr unsigned char font_data[] = {
#embed "../../resources/fonts/DejaVuSansMono.ttf"
};

const stbtt_fontinfo& font() {
    static stbtt_fontinfo info{};
    static std::once_flag once;
    std::call_once(once, [] {
        if (stbtt_InitFont(&info, font_data, stbtt_GetFontOffsetForIndex(font_data, 0)) == 0) {
            throw std::runtime_error("embedded font is not a TrueType font");
        }
    });
    return info;
}

template <class Line>
void for_each_line(std::string_view text, Line&& line) {
    std::size_t start = 0;
    int index = 0;
    while (start <= text.size()) {
        const auto end = text.find('\n', start);
        line(text.substr(start,
                         end == std::string_view::npos ? std::string_view::npos : end - start),
             index++);
        if (end == std::string_view::npos) break;
        start = end + 1;
    }
}

}  // namespace

RgbImage::RgbImage(int width, int height, Rgb fill)
    : width_(width),
      height_(height),
      pixels_(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3U) {
    if (width < 0 || height < 0) throw std::invalid_argument("negative image size");
    for (std::size_t i = 0; i < pixels_.size(); i += 3)
        std::ranges::copy(fill, pixels_.begin() + static_cast<std::ptrdiff_t>(i));
}

void RgbImage::set(int x, int y, Rgb color) noexcept {
    if (x < 0 || y < 0 || x >= width_ || y >= height_) return;
    const auto i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) +
                    static_cast<std::size_t>(x)) *
                   3U;
    std::ranges::copy(color, pixels_.begin() + static_cast<std::ptrdiff_t>(i));
}

void RgbImage::blend(int x, int y, Rgb color, int alpha) noexcept {
    if (x < 0 || y < 0 || x >= width_ || y >= height_ || alpha <= 0) return;
    const auto i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) +
                    static_cast<std::size_t>(x)) *
                   3U;
    for (std::size_t c = 0; c < 3; ++c) {
        const int old = pixels_[i + c];
        pixels_[i + c] =
            static_cast<std::uint8_t>((old * (255 - alpha) + color[c] * alpha + 127) / 255);
    }
}

void rectangle_outline(RgbImage& img, int x0, int y0, int x1, int y1, Rgb color) {
    for (int x = x0; x <= x1; ++x) {
        img.set(x, y0, color);
        img.set(x, y1, color);
    }
    for (int y = y0; y <= y1; ++y) {
        img.set(x0, y, color);
        img.set(x1, y, color);
    }
}

void fill_rectangle(RgbImage& img, int x0, int y0, int x1, int y1, Rgb color, int alpha) {
    for (int y = std::max(y0, 0); y <= std::min(y1, img.height() - 1); ++y) {
        for (int x = std::max(x0, 0); x <= std::min(x1, img.width() - 1); ++x)
            img.blend(x, y, color, alpha);
    }
}

TextBox measure_text(std::string_view text, float pixel_height) {
    const auto& f = font();
    const float scale = stbtt_ScaleForPixelHeight(&f, pixel_height);
    int ascent = 0;
    int descent = 0;
    int gap = 0;
    stbtt_GetFontVMetrics(&f, &ascent, &descent, &gap);
    const int line_height =
        static_cast<int>(std::ceil(static_cast<float>(ascent - descent + gap) * scale));
    TextBox box{.width = 0, .height = 0};
    for_each_line(text, [&](std::string_view line, int) {
        float width = 0.0F;
        for (const char c : line) {
            int advance = 0;
            int bearing = 0;
            stbtt_GetCodepointHMetrics(&f, static_cast<unsigned char>(c), &advance, &bearing);
            width += static_cast<float>(advance) * scale;
        }
        box.width = std::max(box.width, static_cast<int>(std::ceil(width)));
        box.height += line_height;
    });
    return box;
}

void draw_text(RgbImage& img, int x, int y, std::string_view text, float pixel_height, Rgb color) {
    const auto& f = font();
    const float scale = stbtt_ScaleForPixelHeight(&f, pixel_height);
    int ascent = 0;
    int descent = 0;
    int gap = 0;
    stbtt_GetFontVMetrics(&f, &ascent, &descent, &gap);
    const int line_height =
        static_cast<int>(std::ceil(static_cast<float>(ascent - descent + gap) * scale));
    const int baseline = static_cast<int>(std::lround(static_cast<float>(ascent) * scale));

    for_each_line(text, [&](std::string_view line, int index) {
        auto pen = static_cast<float>(x);
        const int base_y = y + index * line_height + baseline;
        for (const char ch : line) {
            const int c = static_cast<unsigned char>(ch);
            int w = 0;
            int h = 0;
            int xoff = 0;
            int yoff = 0;
            unsigned char* bitmap =
                stbtt_GetCodepointBitmap(&f, scale, scale, c, &w, &h, &xoff, &yoff);
            const std::unique_ptr<unsigned char, void (*)(unsigned char*)> owned(
                bitmap, [](unsigned char* p) { stbtt_FreeBitmap(p, nullptr); });
            const int origin_x = static_cast<int>(std::lround(pen)) + xoff;
            for (int row = 0; row < h; ++row) {
                for (int col = 0; col < w; ++col) {
                    img.blend(origin_x + col, base_y + yoff + row, color, bitmap[row * w + col]);
                }
            }
            int advance = 0;
            int bearing = 0;
            stbtt_GetCodepointHMetrics(&f, c, &advance, &bearing);
            pen += static_cast<float>(advance) * scale;
        }
    });
}

std::vector<std::byte> encode_jpeg(const RgbImage& img, int quality) {
    std::vector<std::byte> jpeg;
    const auto sink = [](void* context, void* data, int size) {
        auto& out = *static_cast<std::vector<std::byte>*>(context);
        const auto* bytes = static_cast<const std::byte*>(data);
        out.insert(out.end(), bytes, bytes + size);
    };
    if (stbi_write_jpg_to_func(sink, &jpeg, img.width(), img.height(), 3, img.bytes().data(),
                               quality) == 0) {
        throw std::runtime_error("JPEG encode failed");
    }
    return jpeg;
}

std::vector<std::byte> encode_png(const RgbImage& img) {
    std::vector<std::byte> png;
    const auto sink = [](void* context, void* data, int size) {
        auto& out = *static_cast<std::vector<std::byte>*>(context);
        const auto* bytes = static_cast<const std::byte*>(data);
        out.insert(out.end(), bytes, bytes + size);
    };
    if (stbi_write_png_to_func(sink, &png, img.width(), img.height(), 3, img.bytes().data(),
                               img.width() * 3) == 0) {
        throw std::runtime_error("PNG encode failed");
    }
    return png;
}

void quiet_opencv() {
    static std::once_flag once;
    std::call_once(once,
                   [] { cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING); });
}

RgbImage resize(const RgbImage& src, int width, int height) {
    quiet_opencv();
    RgbImage out(width, height);
    if (src.empty() || out.empty()) return out;
    // cv::Mat has no const view. cv::resize only reads src.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    auto* pixels = const_cast<std::uint8_t*>(src.bytes().data());
    const cv::Mat in(src.height(), src.width(), CV_8UC3, pixels);
    cv::Mat dst(height, width, CV_8UC3, out.bytes().data());
    const bool shrink = width <= src.width() && height <= src.height();
    cv::resize(in, dst, dst.size(), 0, 0, shrink ? cv::INTER_AREA : cv::INTER_LINEAR);
    return out;
}

RgbImage decode(std::span<const std::byte> data) {
    int w = 0;
    int h = 0;
    int channels = 0;
    unsigned char* pixels =
        stbi_load_from_memory(reinterpret_cast<const unsigned char*>(data.data()),
                              static_cast<int>(data.size()), &w, &h, &channels, 3);
    if (pixels == nullptr) return {};
    const std::unique_ptr<unsigned char, void (*)(void*)> owned(pixels, &stbi_image_free);
    RgbImage img(w, h);
    std::copy_n(pixels, img.bytes().size(), img.bytes().begin());
    return img;
}

}  // namespace docksim::image
