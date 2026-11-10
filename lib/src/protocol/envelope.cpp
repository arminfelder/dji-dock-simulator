#include "protocol/envelope.hpp"

#include <array>
#include <chrono>
#include <format>
#include <random>

namespace docksim::envelope {

std::int64_t now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

std::string new_id() {
    thread_local std::mt19937_64 rng{std::random_device{}()};
    std::array<std::uint8_t, 16> b{};
    for (std::size_t i = 0; i < b.size(); i += 8) {
        auto v = rng();
        for (std::size_t j = 0; j < 8; ++j, v >>= 8U) b[i + j] = static_cast<std::uint8_t>(v);
    }
    b[6] = static_cast<std::uint8_t>((b[6] & 0x0FU) | 0x40U);
    b[8] = static_cast<std::uint8_t>((b[8] & 0x3FU) | 0x80U);
    return std::format(
        "{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-"
        "{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
        b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13],
        b[14], b[15]);
}

Json make(Json data, std::string_view method, Json extra) {
    Json msg = {
        {"tid", new_id()}, {"bid", new_id()}, {"timestamp", now_ms()}, {"data", std::move(data)}};
    if (!method.empty()) msg["method"] = method;
    for (const auto& [key, value] : extra.items()) msg[key] = std::move(value);
    return msg;
}

Json reply(const Json& request, Json data, std::string_view gateway) {
    const auto id = [&](const char* key) {
        auto it = request.find(key);
        return it != request.end() ? *it : Json(new_id());
    };
    return {
        {"tid", id("tid")},      {"bid", id("bid")},
        {"timestamp", now_ms()}, {"method", request.value("method", Json())},
        {"gateway", gateway},    {"data", std::move(data)},
    };
}

Json ok_output(Json output) {
    return {{"result", 0}, {"output", std::move(output)}};
}

}  // namespace docksim::envelope
