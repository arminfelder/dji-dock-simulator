#include "docksim/app.hpp"
#include "log/log.hpp"
#include "net/http.hpp"
#include "util/json.hpp"

#include <array>
#include <format>
#include <stdexcept>
#include <vector>

namespace docksim {

namespace {

const auto& logger() {
    static const auto log = log::get("dock_sim.login");
    return log;
}

std::string base(const HttpConfig& cfg) {
    std::string url = cfg.base_url;
    while (url.ends_with('/')) url.pop_back();
    return url;
}

http::Request request(const HttpConfig& cfg, http::Method method, std::string url) {
    return {.method = method,
            .url = std::move(url),
            .verify_tls = cfg.verify_tls || cfg.ca_file.has_value(),
            .ca_file = cfg.ca_file};
}

// The token key differs between backend builds, so search the response tree.
std::optional<std::string> find_token(const Json& body) {
    static constexpr std::array keys{"authToken", "auth_token", "token", "accessToken",
                                     "access_token"};
    std::vector<const Json*> stack{&body};
    while (!stack.empty()) {
        const Json* node = stack.back();
        stack.pop_back();
        if (node->is_object()) {
            for (const auto* key : keys) {
                const auto it = node->find(key);
                if (it != node->end() && it->is_string() && !it->get<std::string>().empty())
                    return it->get<std::string>();
            }
            for (const auto& [_, value] : node->items()) stack.push_back(&value);
        } else if (node->is_array()) {
            for (const auto& value : *node) stack.push_back(&value);
        }
    }
    return std::nullopt;
}

std::optional<std::string> text(const Json& org, const char* key) {
    const auto it = org.find(key);
    if (it == org.end() || it->is_null()) return std::nullopt;
    return it->is_string() ? it->get<std::string>() : it->dump();
}

}  // namespace

Provisioning provision(const HttpConfig& cfg) {
    if (!cfg.username || !cfg.password)
        throw std::runtime_error("http.username and http.password are required");
    auto auth = request(cfg, http::Method::post, base(cfg) + cfg.auth_path);
    logger()->info("authenticate {} as {}", auth.url, *cfg.username);
    auth.headers = {{"Content-Type", "application/json"}};
    auth.body =
        Json{{"email", *cfg.username}, {"password", *cfg.password}, {"source", "web"}}.dump();
    const auto answer = http::perform(auth);
    if (answer.status != 200) {
        throw std::runtime_error(
            std::format("auth failed: HTTP {} {}", answer.status, answer.body.substr(0, 300)));
    }
    const auto body = Json::parse(answer.body, nullptr, false);
    const auto token = find_token(body);
    if (!token)
        throw std::runtime_error(
            std::format("auth response carried no token: {}", answer.body.substr(0, 300)));

    auto get = request(cfg, http::Method::get, base(cfg) + cfg.org_path);
    get.headers = {{"x-auth-token", *token}};
    const auto org_answer = http::perform(get);
    if (org_answer.status != 200) {
        throw std::runtime_error(std::format("{} failed: HTTP {} {}", cfg.org_path,
                                             org_answer.status, org_answer.body.substr(0, 300)));
    }
    Json org = Json::parse(org_answer.body, nullptr, false);
    if (org.contains("data") && org["data"].is_object()) org = org["data"];

    Provisioning p{
        .auth_token = *token,
        .mqtt_username = text(org, "mqttUsername").value_or(""),
        .mqtt_password = text(org, "mqttPassword").value_or(""),
        .mqtt_addr = text(org, "mqttAddr"),
        .organization_id = text(org, "organizationId"),
        .organization_name = text(org, "companyName"),
        .binding_code = text(org, "bindingCode"),
    };
    if (p.mqtt_username.empty()) {
        throw std::runtime_error(std::format("{} returned no mqttUsername: {}", cfg.org_path,
                                             org.dump().substr(0, 300)));
    }
    logger()->info("provisioned: mqtt user {}, organization {} ({}), addr {}", p.mqtt_username,
                   p.organization_id.value_or("-"), p.organization_name.value_or("-"),
                   p.mqtt_addr.value_or("-"));
    return p;
}

void apply(Config& cfg, const Provisioning& p) {
    cfg.broker.username = p.mqtt_username;
    cfg.broker.password = p.mqtt_password;
    if (p.organization_id && !cfg.organization.organization_id)
        cfg.organization.organization_id = p.organization_id;
    if (p.binding_code && !cfg.organization.binding_code)
        cfg.organization.binding_code = p.binding_code;
}

}  // namespace docksim
