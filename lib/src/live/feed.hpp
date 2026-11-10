#pragma once

#include "image/image.hpp"
#include "render/renderer.hpp"
#include "sim/views.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

// Live camera feed. A thread renders the current camera view as fast as it
// can, up to the stream frame rate. The push side reads the latest frame at a
// fixed rate; if the render is slower, the push repeats the last frame.
namespace docksim::live {

// A still view renders again after this many seconds, so that the sun keeps moving.
inline constexpr double still_refresh_s = 30.0;
// Seconds between two statistics lines per feed.
inline constexpr double stats_interval_s = 30.0;
// A frame slower than this repeats the last one more than once at 25 fps.
inline constexpr double slow_frame_s = 0.080;

// Render and send rates of one feed over the last stats interval.
class FeedStats {
public:
    explicit FeedStats(double now) { reset(now); }
    void reset(double now);
    void add_render(double seconds);
    // A summary line once per interval, then a new interval starts.
    [[nodiscard]] std::optional<std::string> report(double now);

    int renders = 0;
    int sent = 0;

private:
    double start_ = 0.0;
    double render_s_ = 0.0;
    double render_max_s_ = 0.0;
};

using RenderFn = std::function<image::RgbImage(const render::Pose& pose, int width, int height,
                                               double dfov, const render::FrameOptions& options)>;
using ViewFn = std::function<sim::View(const std::string& lens)>;

struct FeedOptions {
    std::string name;
    int width = 1280;
    int height = 720;
    int fps = 25;
    int march_scale = 4;
    // Only on a GPU; on the CPU it would cost s * s the render time.
    int supersample = 1;
    bool overlay = true;
    std::optional<std::string> snapshot_dir;
};

class Feed {
public:
    Feed(FeedOptions options, RenderFn render, ViewFn view);
    Feed(const Feed&) = delete;
    Feed& operator=(const Feed&) = delete;
    Feed(Feed&&) = delete;
    Feed& operator=(Feed&&) = delete;
    ~Feed();

    void start();
    void stop();

    // The newest frame. Each call counts as one frame sent to the stream.
    [[nodiscard]] std::shared_ptr<const image::RgbImage> latest();
    void set_lens(std::string lens);
    [[nodiscard]] std::string lens() const;
    [[nodiscard]] int rendered() const noexcept { return rendered_; }
    [[nodiscard]] const FeedOptions& options() const noexcept { return options_; }

private:
    void run();
    [[nodiscard]] image::RgbImage render(const sim::View& view);
    [[nodiscard]] image::RgbImage decorate(const image::RgbImage& frame,
                                           const sim::View& view) const;
    void snapshot(const image::RgbImage& frame) const;

    FeedOptions options_;
    RenderFn render_;
    ViewFn view_;
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    bool stopping_ = false;
    std::string lens_ = "wide";
    std::shared_ptr<const image::RgbImage> latest_;
    FeedStats stats_;
    std::atomic<int> rendered_ = 0;
    std::thread thread_;
};

// White-hot look. Not physical: dark, low-vegetation surfaces show as warm.
[[nodiscard]] image::RgbImage thermal(const image::RgbImage& frame);
// Dark sensor noise, for a camera with nothing to show.
[[nodiscard]] image::RgbImage dark(int width, int height);
// Colour bars with a running frame counter, for the pattern source.
[[nodiscard]] image::RgbImage pattern(int width, int height, std::int64_t frame,
                                      const std::string& label);

}  // namespace docksim::live
