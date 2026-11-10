#pragma once

#include "image/image.hpp"
#include "net/http.hpp"

#include <docksim/config.hpp>

#include <chrono>
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

// WHIP publisher (WebRTC-HTTP ingestion) on libdatachannel. The cloud gives a
// WHIP URL when the media server is SRS or MediaMTX.
namespace docksim::live {

// RTP clock of video, Hz (RFC 3551).
inline constexpr int video_clock_rate = 90000;
inline constexpr auto whip_timeout = std::chrono::seconds{15};
// The cloud waits for the live_start_push reply. Keep this below its timeout.
inline constexpr auto whip_ready_timeout = std::chrono::seconds{5};

struct HttpReply {
    long status = 0;
    std::string body;
    // Header names in lower case.
    std::map<std::string, std::string> headers;
};

// One HTTP request without redirect following: method, url, body, headers.
using HttpCall =
    std::function<HttpReply(http::Method method, const std::string& url, const std::string& body,
                            const std::vector<std::pair<std::string, std::string>>& headers)>;

// HTTP over curl, TLS verified.
[[nodiscard]] HttpCall curl_http();

// Point the ICE candidates of an answer at an address the network carries.
// SRS announces the host of the API request and the word "default", neither of
// which is the firewall address that forwards the media port.
[[nodiscard]] std::string rewrite_candidates(const std::string& sdp,
                                             const std::optional<std::string>& host,
                                             std::optional<int> port);

// https://host/rtc/v1/whip/?app=live&stream=NAME&streamToken=T becomes the API
// https://host/rtc/v1/publish/ and the stream webrtc://host/live/NAME?streamToken=T.
[[nodiscard]] std::pair<std::string, std::string> srs_publish_urls(const std::string& whip_url);

// Resolve location against base, as a browser would.
[[nodiscard]] std::string join_url(const std::string& base, const std::string& location);

// An Annex B access unit without its SEI NAL units. NVENC always writes a
// picture timing SEI before each frame; openh264 writes none.
[[nodiscard]] std::vector<std::byte> without_sei(std::span<const std::byte> annex_b);

// The newest frame to send; called at the stream frame rate.
using FrameSource = std::function<std::shared_ptr<const image::RgbImage>()>;

class WhipSession {
public:
    struct Settings {
        std::string video_id;
        std::string url;
        int width = 1280;
        int height = 720;
        int fps = 25;
        std::int64_t bitrate = 6'000'000;
        VideoEncoder encoder = VideoEncoder::software;
        std::optional<std::string> candidate_host;
        std::optional<int> candidate_port;
        // Local address for ICE; empty: every interface.
        std::optional<std::string> bind_address;
    };

    WhipSession(Settings settings, FrameSource frames, HttpCall http = curl_http());
    WhipSession(const WhipSession&) = delete;
    WhipSession& operator=(const WhipSession&) = delete;
    WhipSession(WhipSession&&) = delete;
    WhipSession& operator=(WhipSession&&) = delete;
    // Sends DELETE for the resource and closes the connection.
    ~WhipSession();

    // Offer, signal, answer. Throws std::runtime_error with the reason.
    void start();
    void stop();

    [[nodiscard]] bool connected() const;
    [[nodiscard]] std::int64_t frames_sent() const;
    [[nodiscard]] const std::optional<std::string>& resource() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace docksim::live
