#pragma once

#include "live/feed.hpp"
#include "live/streamer.hpp"
#include "live/whip.hpp"

#include <docksim/config.hpp>

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

// Live stream push: one feed and one push per active video_id. RTMP goes out
// as FLV, RTSP through the RTSP muxer, WebRTC through WHIP.
namespace docksim::live {

// A rendered feed for a video_id; null for a video_id without a camera, which
// then shows the test pattern.
using FeedFactory = std::function<std::unique_ptr<Feed>(const std::string& video_id)>;

class LiveStreamer final : public Streamer {
public:
    LiveStreamer(const LiveConfig& cfg, FeedFactory feeds, HttpCall http = curl_http());
    LiveStreamer(const LiveStreamer&) = delete;
    LiveStreamer& operator=(const LiveStreamer&) = delete;
    LiveStreamer(LiveStreamer&&) = delete;
    LiveStreamer& operator=(LiveStreamer&&) = delete;
    ~LiveStreamer() override;

    code::Result start(const std::string& video_id, const std::string& url,
                       code::UrlType type) override;
    code::Result stop(const std::string& video_id) override;
    void set_lens(const std::string& video_id, const std::string& lens) override;
    void stop_all() override;

private:
    struct Push;
    std::unique_ptr<Feed> open_feed(const std::string& video_id);

    const LiveConfig* cfg_;
    FeedFactory feeds_;
    HttpCall http_;
    std::mutex mutex_;
    std::map<std::string, std::unique_ptr<Push>> pushes_;
};

// The feed factory of one dock: the aircraft camera and the dock camera.
[[nodiscard]] FeedFactory camera_feeds(const Config& cfg, sim::Cameras& cameras);

}  // namespace docksim::live
