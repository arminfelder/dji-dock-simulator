#pragma once

#include <docksim/config.hpp>

#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace docksim::log {

// A named logger that writes to stderr. Created on first use.
[[nodiscard]] std::shared_ptr<spdlog::logger> get(const std::string& name);

void set_level(LogLevel level);

}  // namespace docksim::log
