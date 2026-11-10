#pragma once

#include <algorithm>
#include <cstddef>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

namespace docksim::raster {

// Clip a little inside the last cell, so that ix + 1 stays a valid index.
inline constexpr float edge = 1.001F;

// A dense 2-D array, rows top to bottom.
template <class T>
class Grid2D {
public:
    Grid2D() = default;
    Grid2D(int width, int height, T fill = T{})
        : width_(width),
          height_(height),
          data_(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), fill) {
        if (width < 0 || height < 0) throw std::invalid_argument("negative grid size");
    }

    [[nodiscard]] int width() const noexcept { return width_; }
    [[nodiscard]] int height() const noexcept { return height_; }
    [[nodiscard]] bool empty() const noexcept { return data_.empty(); }
    [[nodiscard]] std::span<T> data() noexcept { return data_; }
    [[nodiscard]] std::span<const T> data() const noexcept { return data_; }

    [[nodiscard]] T& operator()(int x, int y) noexcept { return data_[index(x, y)]; }
    [[nodiscard]] const T& operator()(int x, int y) const noexcept { return data_[index(x, y)]; }

private:
    [[nodiscard]] std::size_t index(int x, int y) const noexcept {
        return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) +
               static_cast<std::size_t>(x);
    }

    int width_ = 0;
    int height_ = 0;
    std::vector<T> data_;
};

// Bilinear sample at fractional cell position x (column), y (row). Empty
// outside the grid. The render kernels have their own copy; keep them the same.
template <class T>
[[nodiscard]] std::optional<float> bilinear(const Grid2D<T>& g, float x, float y) noexcept {
    if (g.empty() || x < 0 || y < 0 || x > static_cast<float>(g.width() - 1) ||
        y > static_cast<float>(g.height() - 1)) {
        return std::nullopt;
    }
    x = std::min(x, static_cast<float>(g.width()) - edge);
    y = std::min(y, static_cast<float>(g.height()) - edge);
    const int ix = static_cast<int>(x);
    const int iy = static_cast<int>(y);
    const float fx = x - static_cast<float>(ix);
    const float fy = y - static_cast<float>(iy);
    const auto v = [&](int dx, int dy) { return static_cast<float>(g(ix + dx, iy + dy)); };
    const float top = v(0, 0) * (1 - fx) + v(1, 0) * fx;
    const float bottom = v(0, 1) * (1 - fx) + v(1, 1) * fx;
    return top * (1 - fy) + bottom * fy;
}

}  // namespace docksim::raster
