#include "live/feed.hpp"

#include "log/log.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <random>

namespace docksim::live {

namespace {

using Clock = std::chrono::steady_clock;

const auto& logger() {
    static const auto log = log::get("dock_sim.feed");
    return log;
}

double seconds_now() {
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

// Identity of a view without its time; empty for a moving aircraft.
std::optional<std::string> still_key(const sim::View& view) {
    if (!view.pose) return "idle:" + view.idle_text;
    if (!view.still) return std::nullopt;
    const auto& p = *view.pose;
    return std::format("{:.7f}:{:.7f}:{:.3f}:{:.3f}:{:.3f}:{:.3f}:{}",
                       units::in_degrees(p.position.lat), units::in_degrees(p.position.lon),
                       units::metres_above_home(p.height), units::in_degrees(p.heading),
                       units::in_degrees(p.gimbal_pitch), view.dfov, view.lens);
}

std::string upper(std::string s) {
    std::ranges::transform(s, s.begin(),
                           [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}

}  // namespace

void FeedStats::reset(double now) {
    start_ = now;
    renders = 0;
    render_s_ = 0.0;
    render_max_s_ = 0.0;
    sent = 0;
}

void FeedStats::add_render(double seconds) {
    ++renders;
    render_s_ += seconds;
    render_max_s_ = std::max(render_max_s_, seconds);
}

std::optional<std::string> FeedStats::report(double now) {
    const double span = now - start_;
    if (span < stats_interval_s) return std::nullopt;
    const double avg_ms = renders > 0 ? render_s_ / renders * 1000 : 0.0;
    auto line = std::format("rendered {:.1f} fps (avg {:.1f} ms, max {:.1f} ms), sent {:.1f} fps",
                            renders / span, avg_ms, render_max_s_ * 1000, sent / span);
    reset(now);
    return line;
}

Feed::Feed(FeedOptions options, RenderFn render, ViewFn view)
    : options_(std::move(options)),
      render_(std::move(render)),
      view_(std::move(view)),
      latest_(std::make_shared<const image::RgbImage>(dark(options_.width, options_.height))),
      stats_(seconds_now()) {}

Feed::~Feed() {
    stop();
}

void Feed::start() {
    if (!thread_.joinable()) thread_ = std::thread([this] { run(); });
}

void Feed::stop() {
    {
        const std::scoped_lock lock(mutex_);
        stopping_ = true;
    }
    wake_.notify_all();
    if (thread_.joinable()) thread_.join();
}

std::shared_ptr<const image::RgbImage> Feed::latest() {
    const std::scoped_lock lock(mutex_);
    ++stats_.sent;
    return latest_;
}

void Feed::set_lens(std::string lens) {
    const std::scoped_lock lock(mutex_);
    lens_ = std::move(lens);
}

std::string Feed::lens() const {
    const std::scoped_lock lock(mutex_);
    return lens_;
}

void Feed::run() {
    const auto interval = std::chrono::duration<double>(1.0 / std::max(1, options_.fps));
    std::optional<std::string> last_key;
    auto last_render = Clock::time_point{};
    std::optional<image::RgbImage> base;
    while (true) {
        const auto started = Clock::now();
        image::RgbImage frame;
        try {
            const auto view = view_(lens());
            const auto key = still_key(view);
            if (!base || !key || key != last_key ||
                std::chrono::duration<double>(started - last_render).count() > still_refresh_s) {
                base = render(view);
                last_key = key;
                last_render = started;
                ++rendered_;
                const std::scoped_lock lock(mutex_);
                stats_.add_render(std::chrono::duration<double>(Clock::now() - started).count());
            }
            image::RgbImage shown = *base;
            if (view.rain && view.pose && view.lens != "ir")
                render::apply_rain(shown, *view.rain, seconds_now());
            frame = options_.overlay ? decorate(shown, view) : std::move(shown);
        } catch (const std::exception& e) {
            logger()->warn("feed {}: render failed: {}", options_.name, e.what());
            frame = dark(options_.width, options_.height);
            std::unique_lock lock(mutex_);
            if (wake_.wait_for(lock, std::chrono::seconds{1}, [this] { return stopping_; })) return;
        }
        std::optional<std::string> report;
        {
            const std::scoped_lock lock(mutex_);
            latest_ = std::make_shared<const image::RgbImage>(frame);
            report = stats_.report(seconds_now());
        }
        if (report) {
            logger()->info("live {}: {}", options_.name, *report);
            snapshot(frame);
        }
        std::unique_lock lock(mutex_);
        const auto wait =
            std::chrono::duration_cast<Clock::duration>(interval) - (Clock::now() - started);
        if (wake_.wait_for(lock, std::max(wait, Clock::duration::zero()),
                           [this] { return stopping_; }))
            return;
    }
}

image::RgbImage Feed::render(const sim::View& view) {
    if (!view.pose) return dark(options_.width, options_.height);
    std::map<std::string, double> phases;
    render::FrameOptions frame_options;
    // A live frame never waits for a cold tile or Overpass fetch.
    frame_options.wait_for_data = false;
    frame_options.supersample = options_.supersample;
    frame_options.march_scale = options_.march_scale;
    frame_options.profile = &phases;
    const auto started = Clock::now();
    auto frame = render_(*view.pose, options_.width, options_.height, view.dfov, frame_options);
    const double took = std::chrono::duration<double>(Clock::now() - started).count();
    if (took > slow_frame_s) {
        std::string parts;
        for (const auto& [k, v] : phases)
            parts += std::format("{}{} {:.0f}", parts.empty() ? "" : ", ", k, v * 1000);
        logger()->warn("live {}: slow frame {:.0f} ms ({})", options_.name, took * 1000, parts);
    }
    if (view.lens == "ir") frame = thermal(frame);
    return frame;
}

image::RgbImage Feed::decorate(const image::RgbImage& frame, const sim::View& view) const {
    image::RgbImage out = frame;
    const float size = static_cast<float>(std::max(14, options_.height / 36));
    const auto when = view.pose ? view.pose->time : std::chrono::system_clock::now();
    std::vector<std::string> lines{
        options_.name,
        std::format("{:%Y-%m-%d %H:%M:%S} UTC", std::chrono::floor<std::chrono::seconds>(when))};
    if (view.pose) {
        const auto& p = *view.pose;
        lines.push_back(
            std::format("{:.6f} {:.6f}  H {:5.1f} m  HDG {:05.1f}  GIMBAL {:+.0f}  {}",
                        units::in_degrees(p.position.lat), units::in_degrees(p.position.lon),
                        units::metres_above_home(p.height), units::in_degrees(p.heading),
                        units::in_degrees(p.gimbal_pitch), upper(view.lens)));
    }
    if (!view.idle_text.empty()) lines.push_back(view.idle_text);
    int y = 12;
    for (const auto& line : lines) {
        const auto box = image::measure_text(line, size);
        image::fill_rectangle(out, 12 - 6, y - 4, 12 + box.width + 6, y + box.height + 4,
                              {0, 0, 0});
        image::draw_text(out, 12, y, line, size, {255, 255, 255});
        y += box.height + 14;
    }
    return out;
}

void Feed::snapshot(const image::RgbImage& frame) const {
    if (!options_.snapshot_dir) return;
    try {
        std::filesystem::path folder = *options_.snapshot_dir;
        if (const char* home = std::getenv("HOME");
            home != nullptr && options_.snapshot_dir->starts_with("~")) {
            folder = std::filesystem::path(home) /
                     options_.snapshot_dir->substr(options_.snapshot_dir->find_first_not_of("~/"));
        }
        std::filesystem::create_directories(folder);
        std::string name = options_.name;
        std::ranges::replace(name, '/', '_');
        const auto stamp =
            std::format("{:%Y%m%d-%H%M%S}",
                        std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now()));
        const auto path = folder / std::format("{}-{}.png", name, stamp);
        const auto png = image::encode_png(frame);
        std::ofstream(path, std::ios::binary)
            .write(reinterpret_cast<const char*>(png.data()),
                   static_cast<std::streamsize>(png.size()));
        logger()->info("live {}: snapshot {}", options_.name, path.string());
    } catch (const std::exception& e) {
        logger()->warn("live {}: snapshot failed: {}", options_.name, e.what());
    }
}

image::RgbImage thermal(const image::RgbImage& frame) {
    const auto px = frame.bytes();
    const std::size_t n = px.size() / 3;
    std::vector<float> heat(n);
    for (std::size_t i = 0; i < n; ++i) {
        const float r = px[3 * i];
        const float g = px[3 * i + 1];
        const float b = px[3 * i + 2];
        const float luma = 0.30F * r + 0.59F * g + 0.11F * b;
        const float vegetation = std::clamp((g - r) / 40.0F, 0.0F, 1.0F);
        heat[i] = 255.0F - luma * 0.7F - vegetation * 60.0F;
    }
    std::vector<float> sorted = heat;
    const auto pick = [&](double q) {
        const auto k = static_cast<std::size_t>(q * static_cast<double>(n - 1));
        std::ranges::nth_element(sorted, sorted.begin() + static_cast<std::ptrdiff_t>(k));
        return sorted[k];
    };
    const float lo = n > 0 ? pick(0.02) : 0.0F;
    const float hi = n > 0 ? pick(0.98) : 1.0F;
    image::RgbImage out(frame.width(), frame.height());
    auto o = out.bytes();
    for (std::size_t i = 0; i < n; ++i) {
        const auto v = static_cast<std::uint8_t>(
            std::clamp((heat[i] - lo) / std::max(hi - lo, 1.0F) * 255.0F, 0.0F, 255.0F));
        o[3 * i] = o[3 * i + 1] = o[3 * i + 2] = v;
    }
    return out;
}

image::RgbImage dark(int width, int height) {
    thread_local std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> level(8, 19);
    image::RgbImage out(width, height);
    for (auto& b : out.bytes()) b = static_cast<std::uint8_t>(level(rng));
    return out;
}

image::RgbImage pattern(int width, int height, std::int64_t frame, const std::string& label) {
    static constexpr std::array<image::Rgb, 7> bars{{{192, 192, 192},
                                                     {192, 192, 0},
                                                     {0, 192, 192},
                                                     {0, 192, 0},
                                                     {192, 0, 192},
                                                     {192, 0, 0},
                                                     {0, 0, 192}}};
    image::RgbImage out(width, height);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x)
            out.set(x, y, bars[static_cast<std::size_t>(x * 7 / std::max(width, 1))]);
    }
    // A moving bar shows that the stream is live.
    const int sweep = static_cast<int>((frame * 8) % std::max(width, 1));
    image::fill_rectangle(out, sweep, height * 3 / 4, sweep + 16, height - 1, {255, 255, 255});
    const float size = static_cast<float>(std::max(14, height / 30));
    const auto text = std::format("{}\nframe {}", label, frame);
    const auto box = image::measure_text(text, size);
    image::fill_rectangle(out, 14, 16, 26 + box.width, 24 + box.height, {0, 0, 0}, 160);
    image::draw_text(out, 20, 20, text, size, {255, 255, 255});
    return out;
}

}  // namespace docksim::live
