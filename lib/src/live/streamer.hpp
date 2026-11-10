#pragma once

#include "protocol/codes.hpp"

#include <string>

namespace docksim::live {

// Pushes the video of one video_id to a media server. Calls block; the live
// service runs them on a worker thread.
class Streamer {
public:
    Streamer() = default;
    Streamer(const Streamer&) = delete;
    Streamer& operator=(const Streamer&) = delete;
    Streamer(Streamer&&) = delete;
    Streamer& operator=(Streamer&&) = delete;
    virtual ~Streamer() = default;

    virtual code::Result start(const std::string& video_id, const std::string& url,
                               code::UrlType type) = 0;
    virtual code::Result stop(const std::string& video_id) = 0;
    virtual void set_lens(const std::string& video_id, const std::string& lens) = 0;
    virtual void stop_all() = 0;
};

// No live video support in this build. Every start answers LIVE_UNSUPPORTED.
class NullStreamer final : public Streamer {
public:
    code::Result start(const std::string& /*video_id*/, const std::string& /*url*/,
                       code::UrlType /*type*/) override {
        return code::Result::live_unsupported;
    }
    code::Result stop(const std::string& /*video_id*/) override { return code::Result::ok; }
    void set_lens(const std::string& /*video_id*/, const std::string& /*lens*/) override {}
    void stop_all() override {}
};

}  // namespace docksim::live
