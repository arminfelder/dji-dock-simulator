#include "util/base64.hpp"

#include <openssl/evp.h>

namespace docksim::base64 {

namespace {

bool in_alphabet(char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '+' ||
           c == '/';
}

}  // namespace

std::string encode(std::span<const std::uint8_t> data) {
    std::string out((data.size() + 2) / 3 * 4 + 1, '\0');
    const int n = EVP_EncodeBlock(reinterpret_cast<unsigned char*>(out.data()), data.data(),
                                  static_cast<int>(data.size()));
    out.resize(static_cast<std::size_t>(n));
    return out;
}

std::optional<std::vector<std::uint8_t>> decode_lenient(std::string_view text) {
    std::string clean;
    clean.reserve(text.size() + 3);
    bool padded = false;
    for (const char c : text) {
        if (c == '=') {
            padded = true;
        } else if (in_alphabet(c)) {
            if (padded) return std::nullopt;
            clean += c;
        }
    }
    const std::size_t rest = clean.size() % 4;
    if (rest == 1) return std::nullopt;
    // EVP_DecodeBlock wants whole quads; the padding it then decodes as zero bytes is cut below.
    const std::size_t pad = rest == 0 ? 0 : 4 - rest;
    clean.append(pad, '=');
    std::vector<std::uint8_t> out(clean.size() / 4 * 3);
    const int n = EVP_DecodeBlock(out.data(), reinterpret_cast<const unsigned char*>(clean.data()),
                                  static_cast<int>(clean.size()));
    if (n < 0) return std::nullopt;
    out.resize(static_cast<std::size_t>(n) - pad);
    return out;
}

}  // namespace docksim::base64
