#include "log/log.hpp"

#include <spdlog/sinks/stdout_color_sinks.h>

#include <mutex>

namespace docksim::log {

namespace {

std::mutex& registry_mutex() {
    static std::mutex mutex;
    return mutex;
}

spdlog::level::level_enum to_spdlog(LogLevel level) {
    switch (level) {
        case LogLevel::debug:
            return spdlog::level::debug;
        case LogLevel::info:
            return spdlog::level::info;
        case LogLevel::warning:
            return spdlog::level::warn;
        case LogLevel::error:
            return spdlog::level::err;
    }
    return spdlog::level::info;
}

}  // namespace

std::shared_ptr<spdlog::logger> get(const std::string& name) {
    const std::scoped_lock lock(registry_mutex());
    if (auto existing = spdlog::get(name)) return existing;
    auto logger = spdlog::stderr_color_mt(name);
    logger->set_pattern("%Y-%m-%d %H:%M:%S,%e %^%-7l%$ %n: %v");
    logger->set_level(spdlog::get_level());
    return logger;
}

void set_level(LogLevel level) {
    spdlog::set_level(to_spdlog(level));
}

}  // namespace docksim::log
