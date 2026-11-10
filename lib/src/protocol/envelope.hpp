#pragma once

#include "util/json.hpp"

#include <cstdint>
#include <string>
#include <string_view>

// Cloud API message envelope.
namespace docksim::envelope {

[[nodiscard]] std::int64_t now_ms();

// Random UUID version 4, lowercase with hyphens.
[[nodiscard]] std::string new_id();

// {tid, bid, timestamp, data[, method]} plus the extra keys, in that order.
[[nodiscard]] Json make(Json data, std::string_view method = {}, Json extra = Json::object());

// Answer to a services or requests message. tid and bid must match the request.
[[nodiscard]] Json reply(const Json& request, Json data, std::string_view gateway);

[[nodiscard]] Json ok_output(Json output = Json::object());

}  // namespace docksim::envelope
