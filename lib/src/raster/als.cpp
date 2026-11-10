#include "raster/als.hpp"

#include "log/log.hpp"
#include "net/http.hpp"
#include "render/geodata/tiles.hpp"

#include <miniz.h>
#include <unistd.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <format>
#include <fstream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <thread>

namespace docksim::raster::als {

namespace {

// EPSG:3035, ETRS89 / LAEA Europe, on GRS80.
constexpr double a_ = 6378137.0;
constexpr double f_ = 1 / 298.257222101;
constexpr double e2_ = 2 * f_ - f_ * f_;
const double e_ = std::sqrt(e2_);
constexpr double lat0_ = 52.0 * std::numbers::pi / 180.0;
constexpr double lon0_ = 10.0 * std::numbers::pi / 180.0;
constexpr double fe_ = 4321000.0;
constexpr double fn_ = 3210000.0;

double q(double phi) {
    const double s = std::sin(phi);
    return (1 - e2_) * (s / (1 - e2_ * s * s) - std::log((1 - e_ * s) / (1 + e_ * s)) / (2 * e_));
}

struct Constants {
    double qp, rq, beta0, d;
};

const Constants& constants() {
    static const Constants c = [] {
        const double qp = q(std::numbers::pi / 2);
        const double rq = a_ * std::sqrt(qp / 2);
        const double beta0 = std::asin(q(lat0_) / qp);
        const double d = a_ * std::cos(lat0_) /
                         std::sqrt(1 - e2_ * std::sin(lat0_) * std::sin(lat0_)) /
                         (rq * std::cos(beta0));
        return Constants{.qp = qp, .rq = rq, .beta0 = beta0, .d = d};
    }();
    return c;
}

const auto& logger() {
    static const auto log = log::get("dock_sim.als");
    return log;
}

template <class T>
T read_le(std::span<const std::byte> b, std::size_t at) {
    // Written so that a huge offset from the file cannot wrap around.
    if (at > b.size() || b.size() - at < sizeof(T))
        throw std::runtime_error("BigTIFF header is truncated");
    T v{};
    std::memcpy(&v, b.data() + at, sizeof(T));
    if constexpr (std::endian::native == std::endian::big) v = std::byteswap(v);
    return v;
}

std::optional<Bytes> read_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return std::nullopt;
    std::vector<char> chars{std::istreambuf_iterator<char>(in), {}};
    Bytes out(chars.size());
    std::memcpy(out.data(), chars.data(), chars.size());
    return out;
}

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

constexpr std::uint32_t index_magic = 0x31584944;  // "DIX1"

void save_index(const std::filesystem::path& path, const FileIndex& index) {
    Bytes out;
    const auto put = [&](const auto& v) {
        const auto* p = reinterpret_cast<const std::byte*>(&v);
        out.insert(out.end(), p, p + sizeof(v));
    };
    put(index_magic);
    put(index.origin_x);
    put(index.origin_y);
    put(static_cast<std::int64_t>(index.width));
    const std::uint64_t entries = index.offsets.size();
    put(entries);
    for (const auto v : index.offsets) put(v);
    for (const auto v : index.counts) put(v);
    write_file(path, out);
}

std::optional<FileIndex> load_index(const std::filesystem::path& path) {
    const auto data = read_file(path);
    if (!data) return std::nullopt;
    try {
        std::size_t at = 0;
        const auto take = [&]<class T>(T) {
            const T v = read_le<T>(*data, at);
            at += sizeof(T);
            return v;
        };
        if (take(std::uint32_t{}) != index_magic) return std::nullopt;
        FileIndex index;
        index.origin_x = take(double{});
        index.origin_y = take(double{});
        index.width = static_cast<int>(take(std::int64_t{}));
        const auto n = take(std::uint64_t{});
        if (n > (data->size() - at) / 16) return std::nullopt;
        index.offsets.resize(n);
        index.counts.resize(n);
        for (auto& v : index.offsets) v = take(std::uint64_t{});
        for (auto& v : index.counts) v = take(std::uint64_t{});
        return index;
    } catch (const std::runtime_error&) {
        return std::nullopt;
    }
}

}  // namespace

std::pair<double, double> laea(double lat, double lon) noexcept {
    const auto& c = constants();
    const double phi = lat * std::numbers::pi / 180.0;
    const double lam = lon * std::numbers::pi / 180.0;
    const double beta = std::asin(q(phi) / c.qp);
    const double dl = lam - lon0_;
    const double b = c.rq * std::sqrt(2 / (1 + std::sin(c.beta0) * std::sin(beta) +
                                           std::cos(c.beta0) * std::cos(beta) * std::cos(dl)));
    const double x = fe_ + b * c.d * std::cos(beta) * std::sin(dl);
    const double y = fn_ + (b / c.d) * (std::cos(c.beta0) * std::sin(beta) -
                                        std::sin(c.beta0) * std::cos(beta) * std::cos(dl));
    return {x, y};
}

std::vector<float> unpredict_float(std::span<const std::uint8_t> raw, int width, int height) {
    const auto w = static_cast<std::size_t>(width);
    const auto row_bytes = w * 4;
    if (raw.size() < row_bytes * static_cast<std::size_t>(height))
        throw std::runtime_error("ALS block is too short");
    std::vector<float> out(w * static_cast<std::size_t>(height));
    std::vector<std::uint8_t> row(row_bytes);
    for (int y = 0; y < height; ++y) {
        const auto* src = raw.data() + static_cast<std::size_t>(y) * row_bytes;
        std::uint8_t acc = 0;
        for (std::size_t i = 0; i < row_bytes; ++i) {
            acc = static_cast<std::uint8_t>(acc + src[i]);
            row[i] = acc;
        }
        for (std::size_t x = 0; x < w; ++x) {
            // Plane k holds byte k (most significant first) of every pixel.
            const std::uint32_t bits =
                (std::uint32_t{row[x]} << 24U) | (std::uint32_t{row[w + x]} << 16U) |
                (std::uint32_t{row[2 * w + x]} << 8U) | std::uint32_t{row[3 * w + x]};
            out[static_cast<std::size_t>(y) * w + x] = std::bit_cast<float>(bits);
        }
    }
    return out;
}

std::map<int, std::pair<std::uint64_t, std::uint64_t>> bigtiff_tags(
    std::span<const std::byte> head) {
    if (head.size() < 16 || head[0] != std::byte{'I'} || head[1] != std::byte{'I'} ||
        head[2] != std::byte{'+'} || head[3] != std::byte{0}) {
        throw std::runtime_error("not a little-endian BigTIFF");
    }
    const auto ifd = read_le<std::uint64_t>(head, 8);
    const auto n = read_le<std::uint64_t>(head, ifd);
    std::map<int, std::pair<std::uint64_t, std::uint64_t>> tags;
    for (std::uint64_t k = 0; k < n; ++k) {
        const std::size_t entry = ifd + 8 + 20 * k;
        const auto tag = read_le<std::uint16_t>(head, entry);
        const auto type = read_le<std::uint16_t>(head, entry + 2);
        const auto count = read_le<std::uint64_t>(head, entry + 4);
        std::uint64_t unit = 8;
        switch (type) {
            case 1:
            case 2:
                unit = 1;
                break;
            case 3:
                unit = 2;
                break;
            case 4:
                unit = 4;
                break;
            default:
                break;
        }
        std::uint64_t value = 0;
        if (unit * count <= 8) {
            if (type == 3) {
                value = read_le<std::uint16_t>(head, entry + 12);
            } else if (type == 4) {
                value = read_le<std::uint32_t>(head, entry + 12);
            } else {
                value = read_le<std::uint64_t>(head, entry + 12);
            }
        } else {
            value = read_le<std::uint64_t>(head, entry + 12);
        }
        tags[tag] = {value, count};
    }
    return tags;
}

RangeFetcher http_range_fetcher() {
    return
        [](const std::string& url, std::uint64_t start, std::uint64_t end) -> std::optional<Bytes> {
            const auto response = http::perform({
                .url = url,
                .headers = {{"Range", std::format("bytes={}-{}", start, end)}},
                .timeout = std::chrono::seconds{30},
            });
            if (response.status == 404) return std::nullopt;
            if (response.status / 100 != 2)
                throw std::runtime_error(std::format("ALS {}: HTTP {}", url, response.status));
            return http::as_bytes(response.body);
        };
}

Layer::Layer(std::string kind, std::string_view cache_dir, std::string url_template,
             RangeFetcher fetch)
    : kind_(std::move(kind)),
      url_(std::move(url_template)),
      fetch_(fetch ? std::move(fetch) : http_range_fetcher()) {
    std::string lower = kind_;
    std::ranges::transform(lower, lower.begin(),
                           [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    root_ = render::geodata::expand_user(cache_dir) / "als" / lower;
}

std::optional<Bytes> Layer::get(const std::string& name, std::uint64_t start, std::uint64_t end) {
    // name is N<northing>E<easting>.
    const auto e = name.find('E');
    std::string url = url_;
    const auto replace = [&](std::string_view key, std::string_view value) {
        for (auto at = url.find(key); at != std::string::npos;
             at = url.find(key, at + value.size())) {
            url.replace(at, key.size(), value);
        }
    };
    replace("{kind}", kind_);
    replace("{n}", name.substr(1, e - 1));
    replace("{e}", name.substr(e + 1));
    return fetch_(url, start, end);
}

std::shared_ptr<const FileIndex> Layer::fetch_index(const std::string& name) {
    const auto head = get(name, 0, 65535);
    if (!head) return nullptr;
    const auto tags = bigtiff_tags(*head);
    const auto need = [&](int tag) {
        const auto it = tags.find(tag);
        if (it == tags.end()) throw std::runtime_error(std::format("BigTIFF has no tag {}", tag));
        return it->second;
    };
    const auto [off_ptr, off_n] = need(324);
    const auto [cnt_ptr, cnt_n] = need(325);
    // Every tile needs an offset and a byte count. The tables come from a remote file, so their
    // size is checked before it goes into any arithmetic.
    constexpr std::uint64_t max_tiles = std::uint64_t{1} << 24;
    if (off_n != cnt_n || off_n == 0 || off_n > max_tiles) {
        throw std::runtime_error(
            std::format("BigTIFF tile tables do not match: {} offsets, {} counts", off_n, cnt_n));
    }
    if (off_ptr > std::numeric_limits<std::uint64_t>::max() - 8 * off_n ||
        cnt_ptr > std::numeric_limits<std::uint64_t>::max() - 8 * cnt_n) {
        throw std::runtime_error("BigTIFF tile tables lie outside the file");
    }
    const auto lo = std::min(off_ptr, cnt_ptr);
    const auto hi = std::max(off_ptr + 8 * off_n, cnt_ptr + 8 * cnt_n);
    const auto tables = get(name, lo, hi - 1);
    const auto geo_ptr = need(33922).first;
    const auto geo = get(name, geo_ptr, geo_ptr + 47);
    if (!tables || !geo) return nullptr;
    auto index = std::make_shared<FileIndex>();
    for (std::uint64_t i = 0; i < off_n; ++i)
        index->offsets.push_back(read_le<std::uint64_t>(*tables, off_ptr - lo + 8 * i));
    for (std::uint64_t i = 0; i < cnt_n; ++i)
        index->counts.push_back(read_le<std::uint64_t>(*tables, cnt_ptr - lo + 8 * i));
    // ModelTiepoint: i, j, k, x, y, z.
    index->origin_x = read_le<double>(*geo, 24);
    index->origin_y = read_le<double>(*geo, 32);
    index->width = static_cast<int>(need(256).first);
    return index;
}

std::shared_ptr<const FileIndex> Layer::file_index(const std::string& name) {
    {
        const std::scoped_lock lock(mutex_);
        if (const auto it = index_.find(name); it != index_.end()) return it->second;
    }
    const auto path = root_ / (name + ".idx.bin");
    std::shared_ptr<const FileIndex> index;
    try {
        if (auto cached = load_index(path)) {
            index = std::make_shared<const FileIndex>(std::move(*cached));
        } else {
            index = fetch_index(name);
            if (index) save_index(path, *index);
        }
    } catch (const std::exception& e) {
        logger()->warn("ALS {} {}: no index ({})", kind_, name, e.what());
        index = nullptr;
    }
    const std::scoped_lock lock(mutex_);
    index_[name] = index;
    return index;
}

std::shared_ptr<const std::vector<float>> Layer::block_data(const std::string& name,
                                                            const FileIndex& index, int number) {
    const BlockKey key{name, number};
    {
        const std::scoped_lock lock(mutex_);
        if (const auto it = blocks_.find(key); it != blocks_.end()) {
            order_.splice(order_.begin(), order_, it->second.second);
            return it->second.first;
        }
    }
    const auto path = root_ / name / std::format("{}.z", number);
    auto raw = read_file(path);
    if (!raw && static_cast<std::size_t>(number) < index.offsets.size()) {
        const auto start = index.offsets[static_cast<std::size_t>(number)];
        const auto count = index.counts[static_cast<std::size_t>(number)];
        raw = count > 0 ? get(name, start, start + count - 1) : Bytes{};
        if (raw) write_file(path, *raw);
    }
    std::shared_ptr<const std::vector<float>> data;
    if (raw && !raw->empty()) {
        std::vector<std::uint8_t> plain(static_cast<std::size_t>(block) * block * 4);
        mz_ulong size = plain.size();
        if (mz_uncompress(plain.data(), &size, reinterpret_cast<const unsigned char*>(raw->data()),
                          raw->size()) != MZ_OK) {
            throw std::runtime_error(
                std::format("ALS {} block {}: bad deflate data", name, number));
        }
        auto values = unpredict_float(plain, block, block);
        for (auto& v : values) {
            if (v <= nodata + 1) v = std::numeric_limits<float>::quiet_NaN();
        }
        data = std::make_shared<const std::vector<float>>(std::move(values));
    }
    const std::scoped_lock lock(mutex_);
    if (!blocks_.contains(key)) {
        order_.push_front(key);
        blocks_.emplace(key, std::pair{data, order_.begin()});
        while (blocks_.size() > memory_blocks) {
            blocks_.erase(order_.back());
            order_.pop_back();
        }
    }
    return data;
}

Window Layer::window(double x0, double y0, double x1, double y1) {
    const auto col0 = static_cast<std::int64_t>(std::floor(x0));
    const auto col1 = static_cast<std::int64_t>(std::ceil(x1));
    const auto row0 = static_cast<std::int64_t>(std::floor(-y1));
    const auto row1 = static_cast<std::int64_t>(std::ceil(-y0));
    const auto w = static_cast<int>(col1 - col0 + 1);
    const auto h = static_cast<int>(row1 - row0 + 1);
    Window out{
        .heights = Grid2D<float>(w, h, std::numeric_limits<float>::quiet_NaN()), .x0 = 0, .y0 = 0};
    const auto gx0 = col0;
    const auto gy0 = -row0;
    for (auto fy = static_cast<std::int64_t>(std::floor(y0 / file_m));
         fy <= static_cast<std::int64_t>(std::floor(y1 / file_m)); ++fy) {
        for (auto fx = static_cast<std::int64_t>(std::floor(x0 / file_m));
             fx <= static_cast<std::int64_t>(std::floor(x1 / file_m)); ++fx) {
            const auto name = std::format("N{}E{}", fy * file_m, fx * file_m);
            const auto index = file_index(name);
            if (!index) continue;
            // Pixel column c of the file covers easting origin_x + c .. + 1.
            const auto c0 =
                static_cast<std::int64_t>(std::floor(static_cast<double>(gx0) - index->origin_x));
            const auto r0 =
                static_cast<std::int64_t>(std::floor(index->origin_y - static_cast<double>(gy0)));
            const std::int64_t across = index->blocks_across();
            const auto floor_div = [](std::int64_t a, std::int64_t b) {
                return a >= 0 ? a / b : -((-a + b - 1) / b);
            };
            for (auto br = std::max<std::int64_t>(0, floor_div(r0, block));
                 br < std::min(across, floor_div(r0 + h, block) + 1); ++br) {
                for (auto bc = std::max<std::int64_t>(0, floor_div(c0, block));
                     bc < std::min(across, floor_div(c0 + w, block) + 1); ++bc) {
                    const auto data = block_data(name, *index, static_cast<int>(br * across + bc));
                    if (!data) continue;
                    const auto top = br * block - r0;
                    const auto left = bc * block - c0;
                    for (auto y = std::max<std::int64_t>(0, top);
                         y < std::min<std::int64_t>(h, top + block); ++y) {
                        for (auto x = std::max<std::int64_t>(0, left);
                             x < std::min<std::int64_t>(w, left + block); ++x) {
                            out.heights(static_cast<int>(x), static_cast<int>(y)) =
                                (*data)[static_cast<std::size_t>((y - top) * block + (x - left))];
                        }
                    }
                }
            }
        }
    }
    out.x0 = static_cast<double>(gx0) + 0.5;
    out.y0 = static_cast<double>(gy0) - 0.5;
    return out;
}

const Grid2D<float>& HeightGrid::peaks() const {
    std::call_once(peaks_once_, [this] {
        const int w = heights_.width();
        const int h = heights_.height();
        peaks_ = Grid2D<float>(w, h, std::numeric_limits<float>::quiet_NaN());
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                float best = -std::numeric_limits<float>::infinity();
                bool any = false;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        const float v =
                            heights_(std::clamp(x + dx, 0, w - 1), std::clamp(y + dy, 0, h - 1));
                        if (std::isnan(v)) continue;
                        any = true;
                        best = std::max(best, v);
                    }
                }
                if (any) peaks_(x, y) = best;
            }
        }
    });
    return peaks_;
}

float HeightGrid::sample(double lat, double lon, bool peak) const {
    const auto [x, y] = laea(lat, lon);
    const double gx = x - x0_;
    const double gy = y0_ - y;
    const auto& grid = peak ? peaks() : heights_;
    const int w = grid.width();
    const int h = grid.height();
    const auto ix = static_cast<std::int64_t>(std::floor(gx));
    const auto iy = static_cast<std::int64_t>(std::floor(gy));
    if (ix < 0 || iy < 0 || ix >= w - 1 || iy >= h - 1)
        return std::numeric_limits<float>::quiet_NaN();
    const auto fx = static_cast<float>(gx - static_cast<double>(ix));
    const auto fy = static_cast<float>(gy - static_cast<double>(iy));
    const int cx = static_cast<int>(ix);
    const int cy = static_cast<int>(iy);
    return grid(cx, cy) * (1 - fx) * (1 - fy) + grid(cx + 1, cy) * fx * (1 - fy) +
           grid(cx, cy + 1) * (1 - fx) * fy + grid(cx + 1, cy + 1) * fx * fy;
}

HeightAboveGround::HeightAboveGround(std::string_view cache_dir, const std::string& url_template,
                                     const RangeFetcher& fetch)
    : dsm_("DSM", cache_dir, url_template, fetch), dtm_("DTM", cache_dir, url_template, fetch) {}

std::shared_ptr<const HeightGrid> HeightAboveGround::area(double lat0, double lon0, double lat1,
                                                          double lon1, double margin_m) {
    double xmin = std::numeric_limits<double>::infinity();
    double ymin = xmin;
    double xmax = -xmin;
    double ymax = -xmin;
    for (const auto& [lat, lon] :
         {std::pair{lat0, lon0}, {lat0, lon1}, {lat1, lon0}, {lat1, lon1}}) {
        const auto [x, y] = laea(lat, lon);
        xmin = std::min(xmin, x);
        xmax = std::max(xmax, x);
        ymin = std::min(ymin, y);
        ymax = std::max(ymax, y);
    }
    auto dsm = dsm_.window(xmin - margin_m, ymin - margin_m, xmax + margin_m, ymax + margin_m);
    const auto dtm =
        dtm_.window(xmin - margin_m, ymin - margin_m, xmax + margin_m, ymax + margin_m);
    bool any = false;
    auto out = dsm.heights.data();
    const auto ground = dtm.heights.data();
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] -= ground[i];
        any = any || std::isfinite(out[i]);
    }
    if (!any) return nullptr;
    return std::make_shared<const HeightGrid>(std::move(dsm.heights), dsm.x0, dsm.y0);
}

}  // namespace docksim::raster::als
