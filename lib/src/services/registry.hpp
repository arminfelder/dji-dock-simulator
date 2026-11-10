#pragma once

#include "protocol/codes.hpp"
#include "util/json.hpp"

#include <asio/awaitable.hpp>

#include <flat_map>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

// Service dispatch. Every routed services message gets exactly one reply.
namespace docksim::services {

using Reply = Json;
using Handler = std::function<asio::awaitable<Reply>(const Json& data)>;

// A request field has the wrong type or is missing. Answered with PARAM_ERROR.
class BadParameter : public std::invalid_argument {
public:
    using std::invalid_argument::invalid_argument;
};

[[nodiscard]] Reply ok(Json output = Json::object());
[[nodiscard]] Reply result(code::Result code);

struct Service {
    std::string name;
    Handler handler;
    // True when the handler answers success but changes nothing in the simulation.
    bool stub = false;
};

class Registry {
public:
    // Throws std::logic_error if the name is registered twice.
    void add(std::string name, Handler handler, bool stub = false);
    // Answer success without any effect.
    void stub(std::initializer_list<std::string_view> names);

    [[nodiscard]] bool contains(std::string_view name) const;
    [[nodiscard]] std::vector<const Service*> services() const;

    // Unknown method: UNKNOWN_SERVICE. Bad parameters: PARAM_ERROR. Any other
    // failure: NOT_SUPPORTED.
    [[nodiscard]] asio::awaitable<Reply> dispatch(const std::string& method,
                                                  const Json& data) const;

private:
    std::flat_map<std::string, Service, std::less<>> services_;
};

// Typed reads of request fields. Numbers in strings are accepted, as the
// Python core accepted them; anything else throws BadParameter.
[[nodiscard]] double number(const Json& data, std::string_view key, double fallback);
[[nodiscard]] double number(const Json& data, std::string_view key);
[[nodiscard]] std::int64_t integer(const Json& data, std::string_view key, std::int64_t fallback);
[[nodiscard]] std::int64_t integer(const Json& data, std::string_view key);
[[nodiscard]] bool flag(const Json& data, std::string_view key, bool fallback);
[[nodiscard]] std::string text(const Json& data, std::string_view key, std::string_view fallback);

}  // namespace docksim::services
