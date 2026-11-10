#include "net/url.hpp"

#include <curl/curl.h>

#include <format>
#include <stdexcept>

namespace docksim::url {

namespace {

// Accept schemes that this curl build cannot fetch (it is HTTP only).
constexpr unsigned parse_flags = CURLU_NON_SUPPORT_SCHEME;

CURLUPart curl_part(Part part) {
    switch (part) {
        case Part::scheme:
            return CURLUPART_SCHEME;
        case Part::user:
            return CURLUPART_USER;
        case Part::password:
            return CURLUPART_PASSWORD;
        case Part::host:
            return CURLUPART_HOST;
        case Part::port:
            return CURLUPART_PORT;
        case Part::path:
            return CURLUPART_PATH;
        case Part::query:
            return CURLUPART_QUERY;
        case Part::fragment:
            return CURLUPART_FRAGMENT;
    }
    throw std::invalid_argument("unknown URL part");
}

}  // namespace

void Url::Free::operator()(Curl_URL* u) const noexcept {
    curl_url_cleanup(u);
}

std::optional<Url> Url::parse(std::string_view text) {
    Url out(curl_url());
    if (!out.handle_) throw std::bad_alloc();
    if (curl_url_set(out.handle_.get(), CURLUPART_URL, std::string(text).c_str(), parse_flags) !=
        CURLUE_OK) {
        return std::nullopt;
    }
    return out;
}

Url::Url(const Url& other) : handle_(curl_url_dup(other.handle_.get())) {
    if (!handle_) throw std::bad_alloc();
}

Url& Url::operator=(const Url& other) {
    if (this != &other) *this = Url(other);
    return *this;
}

std::optional<Url> Url::resolve(std::string_view reference) const {
    Url out(*this);
    if (curl_url_set(out.handle_.get(), CURLUPART_URL, std::string(reference).c_str(),
                     parse_flags) != CURLUE_OK) {
        return std::nullopt;
    }
    return out;
}

std::optional<std::string> Url::get(Part part) const {
    char* value = nullptr;
    if (curl_url_get(handle_.get(), curl_part(part), &value, 0) != CURLUE_OK) return std::nullopt;
    std::string out(value);
    curl_free(value);
    return out;
}

void Url::set(Part part, std::optional<std::string_view> value) {
    const std::string text(value.value_or(""));
    const auto rc = curl_url_set(handle_.get(), curl_part(part), value ? text.c_str() : nullptr, 0);
    if (rc != CURLUE_OK)
        throw std::invalid_argument(std::format("URL part {}: {}", text, curl_url_strerror(rc)));
}

std::string Url::href() const {
    char* value = nullptr;
    if (curl_url_get(handle_.get(), CURLUPART_URL, &value, 0) != CURLUE_OK) return {};
    std::string out(value);
    curl_free(value);
    return out;
}

std::optional<std::string> Url::query_param(std::string_view name) const {
    const auto query = get(Part::query);
    if (!query) return std::nullopt;
    std::string_view rest = *query;
    while (!rest.empty()) {
        const auto amp = rest.find('&');
        const auto pair = rest.substr(0, amp);
        const auto eq = pair.find('=');
        if (pair.substr(0, eq) == name) {
            return eq == std::string_view::npos ? std::string{} : std::string(pair.substr(eq + 1));
        }
        if (amp == std::string_view::npos) break;
        rest.remove_prefix(amp + 1);
    }
    return std::nullopt;
}

}  // namespace docksim::url
