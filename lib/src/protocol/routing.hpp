#pragma once

#include <string_view>

// The methods that the gateway of a real dock forwards from the cloud.
namespace docksim::routing {

enum class Channel { services, drc_down };

// False when the gateway of this dock model drops the method. The dock then
// sends no reply. Dock 3 firmware is not known, so every method passes.
[[nodiscard]] bool routed(std::string_view dock_model, Channel channel, std::string_view method);

}  // namespace docksim::routing
