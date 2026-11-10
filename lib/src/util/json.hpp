#pragma once

#include <nlohmann/json.hpp>

namespace docksim {

// Insertion-ordered, so payload keys go on the wire in the order the builder writes them.
using Json = nlohmann::ordered_json;

}  // namespace docksim
