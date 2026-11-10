#include "docksim/version.hpp"

#include "build_info.hpp"

namespace docksim {

std::string_view version() noexcept {
    return build::version;
}

}  // namespace docksim
