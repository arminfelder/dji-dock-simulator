#include "render/geodata/tiles.hpp"

#include "log/log.hpp"
#include "net/http.hpp"
#include "raster/mercator.hpp"

#include <asio/post.hpp>
#include <asio/thread_pool.hpp>
#include <openssl/sha.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <format>
#include <fstream>
#include <future>
#include <stdexcept>
#include <system_error>
#include <tuple>

namespace docksim::render::geodata {

namespace {

constexpr int tile = raster::tile_px;

asio::thread_pool& fetch_pool() {
    static asio::thread_pool pool(fetch_workers);
    return pool;
}

// Background builds for prefetch. Two: imagery and DEM can overlap.
asio::thread_pool& prefetch_pool() {
    static asio::thread_pool pool(2);
    return pool;
}

std::string sha1_prefix(std::string_view text, std::size_t chars) {
    std::array<unsigned char, SHA_DIGEST_LENGTH> digest{};
    SHA1(reinterpret_cast<const unsigned char*>(text.data()), text.size(), digest.data());
    std::string hex;
    for (const auto b : digest) hex += std::format("{:02x}", b);
    return hex.substr(0, chars);
}

std::optional<Bytes> read_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return std::nullopt;
    std::vector<char> chars{std::istreambuf_iterator<char>(in), {}};
    Bytes out(chars.size());
    std::ranges::transform(chars, out.begin(), [](char c) { return static_cast<std::byte>(c); });
    return out;
}

// Write to a temporary name and rename, so that a reader never sees half a file.
void write_file(const std::filesystem::path& path, std::span<const std::byte> data) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    auto tmp = path;
    // The cache directory is shared between processes; the pid keeps their temp files apart.
    tmp += std::format(".{}.{}.tmp", ::getpid(),
                       std::hash<std::thread::id>{}(std::this_thread::get_id()));
    {
        std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(data.data()),
                  static_cast<std::streamsize>(data.size()));
        if (!out) return;
    }
    std::filesystem::rename(tmp, path, ec);
    if (ec) std::filesystem::remove(tmp, ec);
}

const auto& logger() {
    static const auto log = log::get("dock_sim.render");
    return log;
}

}  // namespace

Fetcher http_fetcher(std::chrono::milliseconds timeout) {
    return [timeout](const std::string& url) -> FetchResult {
        try {
            const auto response = http::perform({.url = url, .timeout = timeout});
            return {.status = response.status, .body = http::as_bytes(response.body)};
        } catch (const http::TransportError& e) {
            logger()->warn("{}", e.what());
            return {};
        }
    };
}

std::string format_url(std::string_view url_template, int z, int x, int y) {
    std::string out;
    for (std::size_t i = 0; i < url_template.size(); ++i) {
        if (url_template[i] == '{' && i + 2 < url_template.size() && url_template[i + 2] == '}') {
            const char key = url_template[i + 1];
            if (key == 'z' || key == 'x' || key == 'y') {
                out += std::to_string(key == 'z' ? z : key == 'x' ? x : y);
                i += 2;
                continue;
            }
        }
        out += url_template[i];
    }
    return out;
}

std::filesystem::path expand_user(std::string_view path) {
    if (path.starts_with("~")) {
        const char* home = std::getenv("HOME");
        if (home != nullptr)
            return std::filesystem::path(home) /
                   std::filesystem::path(path.substr(path.size() > 1 ? 2 : 1));
    }
    return std::filesystem::path(path);
}

TileSource::TileSource(std::string url_template, std::string_view cache_dir, Fetcher fetch)
    : url_(std::move(url_template)),
      root_(expand_user(cache_dir) / sha1_prefix(url_, 12)),
      fetch_(fetch ? std::move(fetch) : http_fetcher()) {}

std::shared_ptr<const image::RgbImage> TileSource::get(int z, int x, int y) {
    const Key key{z, x, y};
    {
        const std::scoped_lock lock(mutex_);
        if (auto it = memory_.find(key); it != memory_.end()) {
            order_.splice(order_.begin(), order_, it->second.second);
            return it->second.first;
        }
    }
    auto tile_image = load(z, x, y);
    const std::scoped_lock lock(mutex_);
    if (!memory_.contains(key)) {
        order_.push_front(key);
        memory_.emplace(key, std::pair{tile_image, order_.begin()});
        while (memory_.size() > memory_tiles) {
            memory_.erase(order_.back());
            order_.pop_back();
        }
    }
    return tile_image;
}

std::shared_ptr<const image::RgbImage> TileSource::load(int z, int x, int y) {
    const auto dir = root_ / std::to_string(z) / std::to_string(x);
    const auto path = dir / std::format("{}.tile", y);
    const auto missing = dir / std::format("{}.missing", y);
    std::error_code ec;
    if (std::filesystem::exists(missing, ec)) {
        const auto age = std::filesystem::file_time_type::clock::now() -
                         std::filesystem::last_write_time(missing, ec);
        if (!ec && age < missing_ttl) return nullptr;
        std::filesystem::remove(missing, ec);
    }
    auto blob = read_file(path);
    if (!blob) {
        const auto url = format_url(url_, z, x, y);
        auto response = fetch_(url);
        if (response.status == 204 || response.status == 400 || response.status == 404) {
            std::filesystem::create_directories(dir, ec);
            // An empty marker file: the server has no tile here.
            const std::ofstream marker(missing);
            return nullptr;
        }
        if (response.status / 100 != 2) {
            throw std::runtime_error(std::format("tile {}: HTTP {}", url, response.status));
        }
        write_file(path, response.body);
        blob = std::move(response.body);
    }
    auto decoded = image::decode(*blob);
    if (decoded.empty())
        throw std::runtime_error(std::format("tile {}/{}/{} is not an image", z, x, y));
    if (decoded.width() != tile || decoded.height() != tile)
        decoded = image::resize(decoded, tile, tile);
    return std::make_shared<const image::RgbImage>(std::move(decoded));
}

bool Mosaic::covers(const Bounds& b) const noexcept {
    return b.x0 >= static_cast<double>(x0) && b.y0 >= static_cast<double>(y0) &&
           b.x1 < static_cast<double>(x0 + width()) && b.y1 < static_cast<double>(y0 + height());
}

double Mosaic::overlap(const Bounds& b) const noexcept {
    const double dx =
        std::min(b.x1, static_cast<double>(x0 + width())) - std::max(b.x0, static_cast<double>(x0));
    const double dy = std::min(b.y1, static_cast<double>(y0 + height())) -
                      std::max(b.y0, static_cast<double>(y0));
    return std::max(dx, 0.0) * std::max(dy, 0.0);
}

std::optional<float> Mosaic::sample(double px, double py) const noexcept {
    const auto x = static_cast<float>(px - static_cast<double>(x0) - 0.5);
    const auto y = static_cast<float>(py - static_cast<double>(y0) - 0.5);
    const auto value = raster::bilinear(values, x, y);
    if (!value) return std::nullopt;
    const int ix = static_cast<int>(std::min(x, static_cast<float>(width()) - raster::edge));
    const int iy = static_cast<int>(std::min(y, static_cast<float>(height()) - raster::edge));
    if (valid(ix, iy) == 0 || valid(ix + 1, iy + 1) == 0) return std::nullopt;
    return value;
}

Mosaic build_mosaic(TileSource& source, int zoom, const Bounds& bounds) {
    const std::int64_t count = std::int64_t{1} << zoom;
    const auto tx0 =
        std::max<std::int64_t>(0, static_cast<std::int64_t>(std::floor(bounds.x0 / tile)));
    const auto ty0 =
        std::max<std::int64_t>(0, static_cast<std::int64_t>(std::floor(bounds.y0 / tile)));
    const auto tx1 =
        std::min<std::int64_t>(count - 1, static_cast<std::int64_t>(std::floor(bounds.x1 / tile)));
    const auto ty1 =
        std::min<std::int64_t>(count - 1, static_cast<std::int64_t>(std::floor(bounds.y1 / tile)));
    const auto cols = static_cast<int>(std::max<std::int64_t>(0, tx1 - tx0 + 1));
    const auto rows = static_cast<int>(std::max<std::int64_t>(0, ty1 - ty0 + 1));

    Mosaic m;
    m.zoom = zoom;
    m.x0 = tx0 * tile;
    m.y0 = ty0 * tile;
    m.valid = raster::Grid2D<std::uint8_t>(cols * tile, rows * tile, 0);
    m.rgb.assign(static_cast<std::size_t>(cols) * tile * static_cast<std::size_t>(rows) * tile * 3U,
                 0);

    std::vector<std::future<std::shared_ptr<const image::RgbImage>>> jobs;
    for (std::int64_t ty = ty0; ty <= ty1; ++ty) {
        for (std::int64_t tx = tx0; tx <= tx1; ++tx) {
            auto job =
                std::make_shared<std::packaged_task<std::shared_ptr<const image::RgbImage>()>>(
                    [&source, zoom, tx, ty] {
                        return source.get(zoom, static_cast<int>(tx), static_cast<int>(ty));
                    });
            jobs.push_back(job->get_future());
            asio::post(fetch_pool(), [job] { (*job)(); });
        }
    }
    const std::size_t stride = static_cast<std::size_t>(cols) * tile;
    std::size_t j = 0;
    std::exception_ptr failure;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c, ++j) {
            std::shared_ptr<const image::RgbImage> t;
            try {
                t = jobs[j].get();
            } catch (...) {
                if (!failure) failure = std::current_exception();
                continue;
            }
            if (!t) continue;
            for (int y = 0; y < tile; ++y) {
                const std::size_t row =
                    static_cast<std::size_t>(r) * static_cast<std::size_t>(tile) +
                    static_cast<std::size_t>(y);
                const auto src = t->bytes().subspan(
                    static_cast<std::size_t>(y) * static_cast<std::size_t>(tile) * 3U,
                    static_cast<std::size_t>(tile) * 3U);
                std::ranges::copy(
                    src,
                    m.rgb.begin() + static_cast<std::ptrdiff_t>(
                                        (row * stride + static_cast<std::size_t>(c) * tile) * 3U));
                std::fill_n(
                    m.valid.data().begin() + static_cast<std::ptrdiff_t>(
                                                 row * stride + static_cast<std::size_t>(c) * tile),
                    tile, std::uint8_t{1});
            }
        }
    }
    if (failure) std::rethrow_exception(failure);
    return m;
}

MosaicCache::MosaicCache(std::shared_ptr<TileSource> source, Decode decode)
    : source_(std::move(source)), decode_(std::move(decode)) {}

MosaicCache::~MosaicCache() {
    wait_idle();
}

void MosaicCache::set_on_built(BuiltSink sink) {
    const std::scoped_lock lock(mutex_);
    on_built_ = std::move(sink);
}

int MosaicCache::builds() const {
    const std::scoped_lock lock(mutex_);
    return builds_;
}

double MosaicCache::build_seconds() const {
    const std::scoped_lock lock(mutex_);
    return build_s_;
}

bool MosaicCache::idle() const {
    const std::scoped_lock lock(mutex_);
    return pending_.empty();
}

void MosaicCache::wait_idle() const {
    std::unique_lock lock(mutex_);
    idle_cv_.wait(lock, [this] { return pending_.empty(); });
}

std::shared_ptr<const Mosaic> MosaicCache::latest(int zoom) const {
    const std::scoped_lock lock(mutex_);
    const auto it = last_.find(zoom);
    return it == last_.end() || it->second.empty() ? nullptr : it->second.front();
}

std::shared_ptr<const Mosaic> MosaicCache::get(int zoom, const Bounds& bounds, bool wait) {
    std::vector<std::shared_ptr<const Mosaic>> recent;
    {
        const std::scoped_lock lock(mutex_);
        if (const auto it = last_.find(zoom); it != last_.end()) recent = it->second;
    }
    const auto covering =
        std::ranges::find_if(recent, [&](const auto& m) { return m->covers(bounds); });
    if (covering != recent.end()) {
        if (!wait && !(*covering)->covers(bounds.grown(prefetch_margin))) prefetch(zoom, bounds);
        return *covering;
    }
    if (!wait && !recent.empty()) {
        // The camera outran the prefetch. Keep the nearest old data rather than stall.
        prefetch(zoom, bounds);
        return *std::ranges::max_element(recent, {},
                                         [&](const auto& m) { return m->overlap(bounds); });
    }
    return build(zoom, bounds);
}

void MosaicCache::prefetch(int zoom, const Bounds& bounds) {
    // One build per zoom and area: two cameras far apart prefetch side by side.
    const std::tuple key{zoom,
                         static_cast<std::int64_t>(std::floor((bounds.x0 + bounds.x1) / 2 / tile)),
                         static_cast<std::int64_t>(std::floor((bounds.y0 + bounds.y1) / 2 / tile))};
    {
        const std::scoped_lock lock(mutex_);
        if (!pending_.insert(key).second) return;
    }
    asio::post(prefetch_pool(), [this, key, zoom, bounds] {
        try {
            std::ignore = build(zoom, bounds);
        } catch (const std::exception& e) {
            logger()->warn("prefetch of zoom {} failed: {}", zoom, e.what());
        }
        const std::scoped_lock lock(mutex_);
        pending_.erase(key);
        idle_cv_.notify_all();
    });
}

std::shared_ptr<const Mosaic> MosaicCache::build(int zoom, const Bounds& bounds) {
    const auto started = std::chrono::steady_clock::now();
    auto mosaic = std::make_shared<Mosaic>(build_mosaic(*source_, zoom, bounds.grown(build_pad)));
    if (decode_) {
        mosaic->values = decode_(*mosaic);
        mosaic->rgb.clear();
        mosaic->rgb.shrink_to_fit();
        const auto data = mosaic->values.data();
        mosaic->top = data.empty() ? 0.0F : *std::ranges::max_element(data);
    }
    std::shared_ptr<const Mosaic> done = std::move(mosaic);
    BuiltSink sink;
    {
        const std::scoped_lock lock(mutex_);
        sink = on_built_;
    }
    if (sink) sink(done);
    const std::scoped_lock lock(mutex_);
    auto& recent = last_[zoom];
    recent.insert(recent.begin(), done);
    if (recent.size() > per_zoom) recent.resize(per_zoom);
    ++builds_;
    build_s_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    return done;
}

}  // namespace docksim::render::geodata
