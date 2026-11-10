#include "config/yaml.hpp"

#include "util/numeric.hpp"

#include <ryml.hpp>
#include <ryml_std.hpp>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <limits>
#include <regex>
#include <string>

namespace docksim::yaml {

namespace {

std::string lower(std::string_view s) {
    std::string out(s);
    std::ranges::transform(out, out.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

Json plain_scalar(std::string_view text) {
    static const std::regex int_re{R"([-+]?(0|[1-9][0-9_]*))"};
    static const std::regex float_re{
        R"([-+]?([0-9][0-9_]*)?\.[0-9_]*([eE][-+]?[0-9]+)?|[-+]?[0-9][0-9_]*[eE][-+]?[0-9]+)"};

    std::string s(text);
    const std::string l = lower(s);
    if (s.empty() || s == "~" || l == "null") return nullptr;
    if (l == "true" || l == "yes" || l == "on") return true;
    if (l == "false" || l == "no" || l == "off") return false;
    if (l == ".inf" || l == "+.inf") return std::numeric_limits<double>::infinity();
    if (l == "-.inf") return -std::numeric_limits<double>::infinity();
    if (l == ".nan") return std::numeric_limits<double>::quiet_NaN();

    std::string digits = s;
    std::erase(digits, '_');
    if (std::regex_match(s, int_re)) {
        std::int64_t v = 0;
        const char* first = digits.data() + (digits.front() == '+' ? 1 : 0);
        const auto [ptr, ec] = std::from_chars(first, digits.data() + digits.size(), v);
        if (ec == std::errc{} && ptr == digits.data() + digits.size()) return v;
    }
    if (s != "." && std::regex_match(s, float_re)) {
        if (auto v = num::parse_double(digits)) return *v;
    }
    return s;
}

Json convert(ryml::ConstNodeRef node) {
    if (node.is_map()) {
        Json out = Json::object();
        for (const auto child : node.children()) {
            const auto key = child.key();
            out[std::string(key.str, key.len)] = convert(child);
        }
        return out;
    }
    if (node.is_seq()) {
        Json out = Json::array();
        for (const auto child : node.children()) out.push_back(convert(child));
        return out;
    }
    if (!node.has_val()) return nullptr;
    const auto val = node.val();
    const std::string_view text(val.str, val.len);
    if (node.is_val_quoted()) return std::string(text);
    if (val.str == nullptr) return nullptr;
    return plain_scalar(text);
}

}  // namespace

Json parse(std::string_view text) {
    const std::string copy(text);
    const ryml::Tree tree = ryml::parse_in_arena(ryml::to_csubstr(copy));
    const auto root = tree.crootref();
    if (root.is_stream()) {
        if (root.num_children() == 0) return nullptr;
        return convert(root.first_child());
    }
    return convert(root);
}

}  // namespace docksim::yaml
