#pragma once

#include "services/registry.hpp"

#include <string>

// Services for parts that the simulated aircraft does not carry or that need
// no simulation: PSDK speaker and widgets, custom data to ESDK and PSDK, and
// the remote unlocking licenses (DJI 170.flysafe.md).
namespace docksim::services {

// The simulated aircraft has no PSDK or ESDK payload. These services answer
// NOT_SUPPORTED, as a dock answers for a payload that is not mounted.
void register_psdk(Registry& registry);

// unlock_license_list answers an empty, consistent list. Switch and update
// answer success and store nothing.
void register_remote_unlocking(Registry& registry);

}  // namespace docksim::services
