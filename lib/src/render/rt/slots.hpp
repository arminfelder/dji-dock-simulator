#pragma once

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace docksim::render::rt {

// First-fit allocator over triangle positions 0 .. capacity of the record pool.
class Slots {
public:
    explicit Slots(std::size_t capacity);

    [[nodiscard]] std::optional<std::size_t> alloc(std::size_t size);
    // Give back a range; neighbouring free ranges merge.
    void release(std::size_t start, std::size_t size);
    // More room at the end.
    void grow(std::size_t capacity);

    [[nodiscard]] std::size_t capacity() const noexcept { return capacity_; }
    // (start, size), sorted by start.
    [[nodiscard]] const std::vector<std::pair<std::size_t, std::size_t>>& free() const noexcept {
        return free_;
    }

private:
    std::size_t capacity_;
    std::vector<std::pair<std::size_t, std::size_t>> free_;
};

}  // namespace docksim::render::rt
