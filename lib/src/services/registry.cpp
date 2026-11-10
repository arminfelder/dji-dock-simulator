#include "services/registry.hpp"

#include "log/log.hpp"
#include "util/numeric.hpp"

#include <cmath>
#include <format>

namespace docksim::services {

namespace {

const auto& logger() {
    static const auto log = log::get("dock_sim.services");
    return log;
}

const Json* field(const Json& data, std::string_view key) {
    if (!data.is_object()) return nullptr;
    const auto it = data.find(key);
    return it == data.end() || it->is_null() ? nullptr : &*it;
}

}  // namespace

Reply ok(Json output) {
    return {{"result", code::Result::ok}, {"output", std::move(output)}};
}

Reply result(code::Result code) {
    return {{"result", code}, {"output", Json::object()}};
}

void Registry::add(std::string name, Handler handler, bool stub) {
    if (services_.contains(name)) {
        throw std::logic_error(std::format("service {} is registered twice", name));
    }
    services_.emplace(name, Service{.name = name, .handler = std::move(handler), .stub = stub});
}

void Registry::stub(std::initializer_list<std::string_view> names) {
    for (const auto name : names) {
        add(std::string(name), [](const Json&) -> asio::awaitable<Reply> { co_return ok(); }, true);
    }
}

bool Registry::contains(std::string_view name) const {
    return services_.find(name) != services_.end();
}

std::vector<const Service*> Registry::services() const {
    std::vector<const Service*> out;
    for (const auto& [_, service] : services_) out.push_back(&service);
    return out;
}

asio::awaitable<Reply> Registry::dispatch(const std::string& method, const Json& data) const {
    const auto it = services_.find(method);
    if (it == services_.end()) {
        logger()->warn("unhandled service {}", method);
        co_return result(code::Result::unknown_service);
    }
    // The cloud repeats drc_mode_enter about once a second while the DRC panel is open.
    // Its handler logs a real session change itself.
    if (method == "drc_mode_enter")
        logger()->debug("service {}", method);
    else
        logger()->info("service {}", method);
    try {
        co_return co_await it->second.handler(data);
    } catch (const BadParameter& e) {
        logger()->error("service {}: bad parameters {}: {}", method, data.dump(), e.what());
    } catch (const Json::exception& e) {
        logger()->error("service {}: bad parameters {}: {}", method, data.dump(), e.what());
    } catch (const std::exception& e) {
        logger()->error("service {} failed: {}", method, e.what());
        co_return result(code::Result::not_supported);
    }
    co_return result(code::Result::param_error);
}

double number(const Json& data, std::string_view key, double fallback) {
    const Json* v = field(data, key);
    if (v == nullptr) return fallback;
    if (v->is_number()) return v->get<double>();
    if (v->is_boolean()) return v->get<bool>() ? 1.0 : 0.0;
    if (v->is_string()) {
        if (auto d = num::parse_double(v->get<std::string>())) return *d;
    }
    throw BadParameter(std::format("{} must be a number", key));
}

double number(const Json& data, std::string_view key) {
    if (field(data, key) == nullptr) throw BadParameter(std::format("{} is missing", key));
    return number(data, key, 0.0);
}

std::int64_t integer(const Json& data, std::string_view key, std::int64_t fallback) {
    const Json* v = field(data, key);
    if (v == nullptr) return fallback;
    if (v->is_number_integer()) return v->get<std::int64_t>();
    return static_cast<std::int64_t>(std::trunc(number(data, key, 0.0)));
}

std::int64_t integer(const Json& data, std::string_view key) {
    if (field(data, key) == nullptr) throw BadParameter(std::format("{} is missing", key));
    return integer(data, key, 0);
}

bool flag(const Json& data, std::string_view key, bool fallback) {
    const Json* v = field(data, key);
    if (v == nullptr) return fallback;
    if (v->is_boolean()) return v->get<bool>();
    if (v->is_number()) return v->get<double>() != 0.0;
    if (v->is_string()) return !v->get<std::string>().empty();
    return fallback;
}

std::string text(const Json& data, std::string_view key, std::string_view fallback) {
    const Json* v = field(data, key);
    if (v == nullptr) return std::string(fallback);
    if (v->is_string()) return v->get<std::string>();
    return v->dump();
}

}  // namespace docksim::services
