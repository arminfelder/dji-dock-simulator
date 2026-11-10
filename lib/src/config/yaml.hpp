#pragma once

#include "util/json.hpp"

#include <string_view>

namespace docksim::yaml {

// Parse a YAML document into a JSON tree. Plain scalars resolve as in
// YAML 1.1 (PyYAML safe_load): null, bool (yes/no/on/off too), int, float.
// Quoted scalars stay strings. Throws std::runtime_error on bad YAML.
[[nodiscard]] Json parse(std::string_view text);

}  // namespace docksim::yaml
