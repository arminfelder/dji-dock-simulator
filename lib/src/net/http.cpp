#include "net/http.hpp"

#include <docksim/version.hpp>

#include <cpr/cpr.h>
#include <curl/curl.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <format>
#include <mutex>
#include <utility>

namespace docksim::http {

namespace {

// cpr does not call curl_global_init. Call it once, before a thread makes a curl handle.
void global_init() {
    static std::once_flag once;
    std::call_once(once, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
}

std::string lower(std::string_view s) {
    std::string out(s);
    std::ranges::transform(out, out.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

std::string_view name(Method method) noexcept {
    switch (method) {
        case Method::get:
            return "GET";
        case Method::post:
            return "POST";
        case Method::put:
            return "PUT";
        case Method::del:
            return "DELETE";
    }
    std::unreachable();
}

cpr::Response send(cpr::Session& session, Method method) {
    switch (method) {
        case Method::get:
            return session.Get();
        case Method::post:
            return session.Post();
        case Method::put:
            return session.Put();
        case Method::del:
            return session.Delete();
    }
    std::unreachable();
}

}  // namespace

std::optional<std::string> Response::header(std::string_view name) const {
    const auto it = headers.find(lower(name));
    if (it == headers.end()) return std::nullopt;
    return it->second;
}

Response perform(const Request& request) {
    global_init();
    cpr::Session session;
    session.SetUrl(cpr::Url{request.url});
    session.SetUserAgent(cpr::UserAgent{std::format("dock-sim/{}", docksim::version())});
    session.SetTimeout(cpr::Timeout{request.timeout});
    session.SetRedirect(cpr::Redirect{request.follow_redirects});
    session.SetVerifySsl(cpr::VerifySsl{request.verify_tls});
    if (request.ca_file) session.SetSslOptions(cpr::Ssl(cpr::ssl::CaInfo{*request.ca_file}));

    cpr::Header headers;
    for (const auto& [name, value] : request.headers) headers[name] = value;
    session.SetHeader(headers);
    if (request.method == Method::post || request.method == Method::put)
        session.SetBody(cpr::Body{request.body});

    if (request.stop.stop_possible()) {
        session.SetProgressCallback(cpr::ProgressCallback{
            [&stop = request.stop](cpr::cpr_pf_arg_t, cpr::cpr_pf_arg_t, cpr::cpr_pf_arg_t,
                                   cpr::cpr_pf_arg_t,
                                   std::intptr_t) { return !stop.stop_requested(); }});
    }

    auto reply = send(session, request.method);
    if (reply.error) {
        throw TransportError(
            std::format("{} {}: {}", name(request.method), request.url, reply.error.message));
    }
    Response response{.status = reply.status_code,
                      .body = std::move(reply.text),
                      .effective_url = reply.url.str()};
    for (const auto& [name, value] : reply.header) response.headers[lower(name)] = value;
    return response;
}

std::vector<std::byte> as_bytes(std::string_view body) {
    std::vector<std::byte> out(body.size());
    std::ranges::transform(body, out.begin(), [](char c) { return static_cast<std::byte>(c); });
    return out;
}

}  // namespace docksim::http
