#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <string_view>
#include <vector>

// Blocking HTTP over cpr (libcurl). Call it from a worker thread, never from the
// executor that runs the dock.
namespace docksim::http {

// The methods the simulator sends. del is DELETE.
enum class Method : std::uint8_t { get, post, put, del };

struct Request {
    Method method = Method::get;
    std::string url;
    std::vector<std::pair<std::string, std::string>> headers;
    std::string body;
    bool verify_tls = true;
    std::optional<std::string> ca_file;
    std::chrono::milliseconds timeout{30'000};
    bool follow_redirects = true;
    // A stop request ends the transfer within about one second.
    std::stop_token stop;
};

struct Response {
    long status = 0;
    std::string body;
    // Header names in lower case. The last value wins.
    std::map<std::string, std::string> headers;
    std::string effective_url;

    [[nodiscard]] std::optional<std::string> header(std::string_view name) const;
};

// The transfer failed below HTTP: DNS, connect, TLS, timeout.
class TransportError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Throws TransportError. An HTTP error status is a normal response.
[[nodiscard]] Response perform(const Request& request);

[[nodiscard]] std::vector<std::byte> as_bytes(std::string_view body);

}  // namespace docksim::http
