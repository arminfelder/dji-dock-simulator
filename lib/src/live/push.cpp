#include "live/push.hpp"

#include "cloud/redact.hpp"
#include "live/av.hpp"
#include "log/log.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <tuple>
#include <utility>

namespace docksim::live {

namespace {

using Clock = std::chrono::steady_clock;

const auto& logger() {
    static const auto log = log::get("dock_sim.live");
    return log;
}

}  // namespace

// One active stream: the feed (or the pattern), and the thread or session that sends it.
struct LiveStreamer::Push {
    std::string video_id;
    std::unique_ptr<Feed> feed;
    std::unique_ptr<WhipSession> whip;
    std::thread pump;
    std::mutex mutex;
    std::condition_variable wake;
    bool stopping = false;
    std::int64_t pattern_frame = 0;

    Push() = default;
    Push(const Push&) = delete;
    Push& operator=(const Push&) = delete;
    Push(Push&&) = delete;
    Push& operator=(Push&&) = delete;
    ~Push() { stop(); }

    std::shared_ptr<const image::RgbImage> frame(int width, int height) {
        if (feed) return feed->latest();
        return std::make_shared<const image::RgbImage>(
            pattern(width, height, pattern_frame++, video_id));
    }

    void stop() {
        {
            const std::scoped_lock lock(mutex);
            stopping = true;
        }
        wake.notify_all();
        if (pump.joinable()) pump.join();
        if (whip) whip->stop();
        if (feed) feed->stop();
    }
};

LiveStreamer::LiveStreamer(const LiveConfig& cfg, FeedFactory feeds, HttpCall http)
    : cfg_(&cfg), feeds_(std::move(feeds)), http_(std::move(http)) {}

LiveStreamer::~LiveStreamer() {
    stop_all();
}

std::unique_ptr<Feed> LiveStreamer::open_feed(const std::string& video_id) {
    if (cfg_->source != LiveSource::render || !feeds_) return nullptr;
    auto feed = feeds_(video_id);
    if (feed) feed->start();
    return feed;
}

code::Result LiveStreamer::start(const std::string& video_id, const std::string& url,
                                 code::UrlType type) {
    if (!cfg_->enabled) {
        logger()->warn("live is disabled, push of {} not started", video_id);
        return code::Result::ok;
    }
    const char* muxer = type == code::UrlType::rtmp   ? "flv"
                        : type == code::UrlType::rtsp ? "rtsp"
                                                      : nullptr;
    if (muxer == nullptr && type != code::UrlType::webrtc) {
        logger()->error("unsupported live url_type {}", static_cast<int>(type));
        return code::Result::live_unsupported;
    }
    std::ignore = stop(video_id);
    auto push = std::make_unique<Push>();
    push->video_id = video_id;
    push->feed = open_feed(video_id);
    const auto [width, height] = cfg_->frame();
    const int fps = cfg_->fps;
    logger()->info("start push {} -> {} ({})", video_id, redact::safe_url(url),
                   push->feed ? "rendered" : "pattern");

    if (type == code::UrlType::webrtc) {
        auto* raw = push.get();
        push->whip = std::make_unique<WhipSession>(
            WhipSession::Settings{.video_id = video_id,
                                  .url = url,
                                  .width = width,
                                  .height = height,
                                  .fps = fps,
                                  .bitrate = cfg_->bitrate_bps(),
                                  .encoder = cfg_->encoder,
                                  .candidate_host = cfg_->candidate_host,
                                  .candidate_port = cfg_->candidate_port},
            [raw, width, height] { return raw->frame(width, height); }, http_);
        try {
            push->whip->start();
        } catch (const std::exception& e) {
            logger()->error("whip push {} failed: {}", video_id, e.what());
            push->stop();
            return code::Result::live_no_stream;
        }
    } else {
        // Connect in the calling thread, so that a refused push is answered at once.
        std::shared_ptr<MuxPush> mux;
        try {
            mux = std::make_shared<MuxPush>(
                MuxPush::Settings{.url = url,
                                  .muxer = muxer,
                                  .video = {.width = width,
                                            .height = height,
                                            .fps = fps,
                                            .bitrate = cfg_->bitrate_bps(),
                                            .wanted = cfg_->encoder,
                                            .global_header = false},
                                  .audio = true});
        } catch (const std::exception& e) {
            logger()->error("push {} failed: {}", video_id, e.what());
            push->stop();
            return code::Result::live_no_stream;
        }
        auto* raw = push.get();
        push->pump = std::thread([raw, mux, width, height, fps] {
            // The newest frame at the stream rate; a slow render repeats the last frame.
            const auto start = Clock::now();
            for (std::int64_t n = 0;; ++n) {
                {
                    std::unique_lock lock(raw->mutex);
                    const auto due =
                        start + std::chrono::duration_cast<Clock::duration>(
                                    std::chrono::duration<double>(static_cast<double>(n) / fps));
                    if (raw->wake.wait_until(lock, due, [raw] { return raw->stopping; })) break;
                }
                try {
                    mux->push(*raw->frame(width, height));
                } catch (const std::exception& e) {
                    logger()->error("push {} stopped: {}", raw->video_id, e.what());
                    // Nothing reads the feed now. Stop it, so that it does not render for nothing.
                    // Push::stop joins this thread before it stops the feed again.
                    if (raw->feed) raw->feed->stop();
                    return;
                }
            }
            try {
                mux->finish();
            } catch (const std::exception& e) {
                logger()->warn("push {}: {}", raw->video_id, e.what());
            }
        });
    }
    std::unique_ptr<Push> replaced;
    {
        const std::scoped_lock lock(mutex_);
        replaced = std::exchange(pushes_[video_id], std::move(push));
    }
    if (replaced) {
        replaced->stop();
        logger()->info("stop push {} (replaced)", video_id);
    }
    return code::Result::ok;
}

code::Result LiveStreamer::stop(const std::string& video_id) {
    std::unique_ptr<Push> push;
    {
        const std::scoped_lock lock(mutex_);
        const auto it = pushes_.find(video_id);
        if (it == pushes_.end()) return code::Result::ok;
        push = std::move(it->second);
        pushes_.erase(it);
    }
    push->stop();
    logger()->info("stop push {}", video_id);
    return code::Result::ok;
}

void LiveStreamer::set_lens(const std::string& video_id, const std::string& lens) {
    const std::scoped_lock lock(mutex_);
    const auto it = pushes_.find(video_id);
    if (it == pushes_.end() || !it->second->feed) return;
    it->second->feed->set_lens(lens);
    logger()->info("live {}: lens {}", video_id, lens);
}

void LiveStreamer::stop_all() {
    std::vector<std::string> ids;
    {
        const std::scoped_lock lock(mutex_);
        for (const auto& [id, _] : pushes_) ids.push_back(id);
    }
    for (const auto& id : ids) std::ignore = stop(id);
}

FeedFactory camera_feeds(const Config& cfg, sim::Cameras& cameras) {
    return [&cfg, &cameras](const std::string& video_id) -> std::unique_ptr<Feed> {
        // video_id: sn/payload_index/lens-index.
        const auto first = video_id.find('/');
        const auto sn = video_id.substr(0, first);
        const auto& device = cfg.device;
        if (sn != device.drone_sn && sn != device.dock_sn) return nullptr;
        const bool drone = sn == device.drone_sn;
        auto& renderer = cameras.renderer();
        const auto [width, height] = cfg.live.frame();
        FeedOptions options{
            .name = video_id,
            .width = width,
            .height = height,
            .fps = cfg.live.fps,
            .march_scale = cfg.live.march_scale.value_or(4),
            .supersample = renderer.gpu() ? cfg.live.supersample : 1,
            .overlay = cfg.live.overlay,
            .snapshot_dir = cfg.live.snapshot_dir,
        };
        auto feed = std::make_unique<Feed>(
            std::move(options),
            [&renderer](const render::Pose& pose, int w, int h, double dfov,
                        const render::FrameOptions& o) {
                return renderer.render(pose, w, h, dfov, o);
            },
            [&cameras, drone](const std::string& lens) {
                return drone ? cameras.drone_view(lens) : cameras.dock_view();
            });
        if (drone) {
            const auto second = video_id.find('/', first + 1);
            if (second != std::string::npos) {
                const auto lens = video_id.substr(second + 1);
                feed->set_lens(lens.substr(0, lens.find('-')));
            }
        }
        return feed;
    };
}

}  // namespace docksim::live
