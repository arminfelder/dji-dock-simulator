#pragma once

#include "image/image.hpp"
#include "raster/grid.hpp"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

// XYZ map tiles: a cached HTTP client and stitched mosaics. The disk layout
// is <cache_dir>/<sha1(url)[:12]>/<z>/<x>/<y>.tile, plus a <y>.missing marker
// for tiles the server does not have. The Python simulator uses the same
// layout, so both share one cache.
namespace docksim::render::geodata {

using Bytes = std::vector<std::byte>;

// Decoded tiles kept in memory per source.
inline constexpr std::size_t memory_tiles = 384;
inline constexpr int fetch_workers = 8;
// A "no tile here" marker is trusted this long, then the tile is asked for
// again. A server can answer 404 during maintenance.
inline constexpr auto missing_ttl = std::chrono::hours{7 * 24};
// A live frame starts the next build when it comes this close to the mosaic
// edge, in pixels. It must be less than build_pad.
inline constexpr double prefetch_margin = 128.0;
inline constexpr double build_pad = 256.0;

// status 0 means the transfer failed (DNS, connect, timeout).
struct FetchResult {
    long status = 0;
    Bytes body;
};
// Blocking GET of one URL. Called from worker threads.
using Fetcher = std::function<FetchResult(const std::string& url)>;

// libcurl GET with the dock-sim user agent.
[[nodiscard]] Fetcher http_fetcher(std::chrono::milliseconds timeout = std::chrono::seconds{20});

// Replaces {z}, {x} and {y} in an XYZ template.
[[nodiscard]] std::string format_url(std::string_view url_template, int z, int x, int y);

// "~/..." to the home directory.
[[nodiscard]] std::filesystem::path expand_user(std::string_view path);

// One XYZ tile URL with a disk cache. A missing tile is nullptr. Thread-safe.
class TileSource {
public:
    TileSource(std::string url_template, std::string_view cache_dir, Fetcher fetch = {});

    // Throws std::runtime_error when the server fails with anything but 204,
    // 400 or 404 ("no tile here"), or the transfer fails.
    [[nodiscard]] std::shared_ptr<const image::RgbImage> get(int z, int x, int y);

    [[nodiscard]] const std::filesystem::path& root() const noexcept { return root_; }
    [[nodiscard]] const std::string& url_template() const noexcept { return url_; }

private:
    [[nodiscard]] std::shared_ptr<const image::RgbImage> load(int z, int x, int y);

    using Key = std::tuple<int, int, int>;
    std::string url_;
    std::filesystem::path root_;
    Fetcher fetch_;
    std::mutex mutex_;
    std::list<Key> order_;
    std::map<Key, std::pair<std::shared_ptr<const image::RgbImage>, std::list<Key>::iterator>>
        memory_;
};

// Global pixel bounds (x0, y0, x1, y1) at one zoom.
struct Bounds {
    double x0;
    double y0;
    double x1;
    double y1;

    [[nodiscard]] Bounds grown(double by) const noexcept {
        return {.x0 = x0 - by, .y0 = y0 - by, .x1 = x1 + by, .y1 = y1 + by};
    }
};

// Stitched tiles at one zoom. Pixel (0, 0) is global pixel (x0, y0).
struct Mosaic {
    int zoom = 0;
    std::int64_t x0 = 0;
    std::int64_t y0 = 0;
    // RGB8, width * height * 3. Empty after decode for a DEM.
    std::vector<std::uint8_t> rgb;
    raster::Grid2D<std::uint8_t> valid;
    // Decoded values, for example DEM metres. Empty for imagery.
    raster::Grid2D<float> values;
    // Highest decoded value. Rays above it cannot hit the terrain.
    float top = 0.0F;

    [[nodiscard]] int width() const noexcept { return valid.width(); }
    [[nodiscard]] int height() const noexcept { return valid.height(); }
    [[nodiscard]] bool covers(const Bounds& b) const noexcept;
    [[nodiscard]] double overlap(const Bounds& b) const noexcept;
    // Bilinear sample of values at a global pixel position; empty without data.
    [[nodiscard]] std::optional<float> sample(double px, double py) const noexcept;
};

// Stitch the tiles that cover the bounds. Missing tiles stay invalid.
[[nodiscard]] Mosaic build_mosaic(TileSource& source, int zoom, const Bounds& bounds);

// Turns the stitched RGB into values, for example a Terrarium decode.
using Decode = std::function<raster::Grid2D<float>(const Mosaic&)>;
using BuiltSink = std::function<void(const std::shared_ptr<const Mosaic>&)>;

// Recent mosaics per zoom of one source. Frames of a moving camera reuse
// them. With wait false a frame never builds: it gets a cached mosaic, and a
// background build starts before the camera leaves it. Several cameras share
// one cache, so each zoom keeps per_zoom mosaics. Thread-safe.
class MosaicCache {
public:
    static constexpr std::size_t per_zoom = 6;

    explicit MosaicCache(std::shared_ptr<TileSource> source, Decode decode = {});
    MosaicCache(const MosaicCache&) = delete;
    MosaicCache& operator=(const MosaicCache&) = delete;
    MosaicCache(MosaicCache&&) = delete;
    MosaicCache& operator=(MosaicCache&&) = delete;
    // Waits for running background builds.
    ~MosaicCache();

    // A mosaic that covers bounds; without wait, the cached one where possible.
    [[nodiscard]] std::shared_ptr<const Mosaic> get(int zoom, const Bounds& bounds,
                                                    bool wait = true);
    [[nodiscard]] std::shared_ptr<const Mosaic> latest(int zoom) const;

    // Called with each new mosaic, in the building thread. The GPU path uploads here.
    void set_on_built(BuiltSink sink);

    [[nodiscard]] int builds() const;
    [[nodiscard]] double build_seconds() const;
    [[nodiscard]] bool idle() const;
    void wait_idle() const;
    [[nodiscard]] TileSource& source() noexcept { return *source_; }

private:
    std::shared_ptr<const Mosaic> build(int zoom, const Bounds& bounds);
    void prefetch(int zoom, const Bounds& bounds);

    std::shared_ptr<TileSource> source_;
    Decode decode_;
    BuiltSink on_built_;
    mutable std::mutex mutex_;
    mutable std::condition_variable idle_cv_;
    // Newest first.
    std::map<int, std::vector<std::shared_ptr<const Mosaic>>> last_;
    std::set<std::tuple<int, std::int64_t, std::int64_t>> pending_;
    int builds_ = 0;
    double build_s_ = 0.0;
};

}  // namespace docksim::render::geodata
