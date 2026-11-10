#include "cloud/redact.hpp"

#include "net/url.hpp"

#include <algorithm>
#include <array>
#include <cctype>

namespace docksim::redact {

namespace {

bool is_secret(std::string_view key) {
    static constexpr std::array<std::string_view, 6> words{"token",    "signature", "credential",
                                                           "password", "secret",    "key"};
    std::string lower(key);
    std::ranges::transform(lower, lower.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return std::ranges::any_of(words, [&](auto w) { return lower.find(w) != std::string::npos; });
}

}  // namespace

namespace {

// key=value pairs with the values of secret keys replaced by ***.
std::string mask_query(std::string_view query) {
    std::string out;
    std::size_t pos = 0;
    bool first = true;
    while (pos <= query.size()) {
        auto amp = query.find('&', pos);
        if (amp == std::string_view::npos) amp = query.size();
        const std::string_view pair = query.substr(pos, amp - pos);
        if (!pair.empty()) {
            const auto eq = pair.find('=');
            const std::string_view key = pair.substr(0, eq);
            if (!first) out += '&';
            first = false;
            out += key;
            out += '=';
            if (is_secret(key)) {
                out += "***";
            } else if (eq != std::string_view::npos) {
                out += pair.substr(eq + 1);
            }
        }
        pos = amp + 1;
    }
    return out;
}

// For text that is not an absolute URL: mask the query, drop the fragment.
std::string mask_raw(std::string_view text) {
    const auto hash = text.find('#');
    const auto q = text.find('?');
    if (q == std::string_view::npos || q > hash) return std::string(text.substr(0, hash));
    return std::string(text.substr(0, q + 1)) +
           mask_query(text.substr(q + 1, hash == std::string_view::npos ? hash : hash - q - 1));
}

}  // namespace

std::string safe_url(std::string_view text) {
    auto parsed = url::Url::parse(text);
    if (!parsed) return mask_raw(text);
    if (parsed->get(url::Part::password)) parsed->set(url::Part::password, "***");
    if (const auto query = parsed->get(url::Part::query))
        parsed->set(url::Part::query, mask_query(*query));
    parsed->set(url::Part::fragment, std::nullopt);
    return parsed->href();
}

}  // namespace docksim::redact
