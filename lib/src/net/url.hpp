#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

struct Curl_URL;

// Absolute URLs over the libcurl URL API: RFC 3986 parsing and reference resolution.
// Any scheme is accepted (rtmp, rtsp, webrtc), not only the schemes that this curl build can fetch.
namespace docksim::url {

enum class Part : std::uint8_t { scheme, user, password, host, port, path, query, fragment };

class Url {
public:
    // Empty if text is not an absolute URL.
    [[nodiscard]] static std::optional<Url> parse(std::string_view text);

    Url(const Url& other);
    Url& operator=(const Url& other);
    Url(Url&&) noexcept = default;
    Url& operator=(Url&&) noexcept = default;
    ~Url() = default;

    // The reference (absolute, "/path", "path", "?query", "//host/path") resolved against this URL.
    [[nodiscard]] std::optional<Url> resolve(std::string_view reference) const;

    // Empty if the URL has no such part. Values stay percent-encoded.
    [[nodiscard]] std::optional<std::string> get(Part part) const;
    // An empty value removes the part.
    void set(Part part, std::optional<std::string_view> value);
    [[nodiscard]] std::string href() const;

    // The still-encoded value of the first query parameter with this name.
    [[nodiscard]] std::optional<std::string> query_param(std::string_view name) const;

private:
    struct Free {
        void operator()(Curl_URL* u) const noexcept;
    };
    explicit Url(Curl_URL* handle) : handle_(handle) {}
    std::unique_ptr<Curl_URL, Free> handle_;
};

}  // namespace docksim::url
