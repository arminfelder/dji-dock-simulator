#include "render/rt/slots.hpp"

#include <algorithm>

namespace docksim::render::rt {

Slots::Slots(std::size_t capacity) : capacity_(capacity) {
    if (capacity > 0) free_.emplace_back(0, capacity);
}

std::optional<std::size_t> Slots::alloc(std::size_t size) {
    for (auto it = free_.begin(); it != free_.end(); ++it) {
        auto& [start, length] = *it;
        if (length < size) continue;
        const std::size_t found = start;
        if (length == size) {
            free_.erase(it);
        } else {
            start += size;
            length -= size;
        }
        return found;
    }
    return std::nullopt;
}

void Slots::release(std::size_t start, std::size_t size) {
    if (size == 0) return;
    free_.emplace_back(start, size);
    std::ranges::sort(free_);
    std::vector<std::pair<std::size_t, std::size_t>> merged;
    for (const auto& [s, n] : free_) {
        if (!merged.empty() && merged.back().first + merged.back().second == s) {
            merged.back().second += n;
        } else {
            merged.emplace_back(s, n);
        }
    }
    free_ = std::move(merged);
}

void Slots::grow(std::size_t capacity) {
    if (capacity <= capacity_) return;
    release(capacity_, capacity - capacity_);
    capacity_ = capacity;
}

}  // namespace docksim::render::rt
