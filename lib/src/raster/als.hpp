#pragma once

#include "raster/grid.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// Laser-scan height models (ALS) of Austria from the BEV, read block by block.
// DSM (roofs, trees) and DTM (terrain) at 1 m, CC BY 4.0, in 50 km BigTIFF
// files of up to 7.5 GB, tiled in 256 x 256 blocks, each deflate-compressed
// with the floating point predictor. A small area needs only a few range
// requests. Heights above the ground are DSM minus DTM, which avoids a datum
// mismatch with the Terrarium DEM.
namespace docksim::raster::als {

inline constexpr std::string_view bev_url =
    "https://data.bev.gv.at/download/ALS/{kind}/20210401/ALS_{kind}_CRS3035RES50000mN{n}E{e}.tif";
inline constexpr int file_m = 50'000;
inline constexpr int block = 256;
inline constexpr float nodata = -9999.0F;
// Decoded blocks kept in memory per layer, 256 KB each.
inline constexpr std::size_t memory_blocks = 256;

using Bytes = std::vector<std::byte>;

// EPSG:3035 easting and northing of lat/lon (ellipsoidal formulas, EPSG guidance 7-2).
[[nodiscard]] std::pair<double, double> laea(double lat, double lon) noexcept;

// Undo the TIFF floating point predictor (3) of a float32 block. Each row
// holds the bytes of all pixels by significance (most significant first),
// each byte stored as the difference to the one before it in the row.
[[nodiscard]] std::vector<float> unpredict_float(std::span<const std::uint8_t> raw, int width,
                                                 int height);

// Tag -> (value or pointer, count) of the first IFD of a little-endian BigTIFF.
// Throws std::runtime_error.
[[nodiscard]] std::map<int, std::pair<std::uint64_t, std::uint64_t>> bigtiff_tags(
    std::span<const std::byte> head);

struct FileIndex {
    double origin_x = 0;
    double origin_y = 0;
    int width = 0;
    std::vector<std::uint64_t> offsets;
    std::vector<std::uint64_t> counts;

    [[nodiscard]] int blocks_across() const noexcept { return (width + block - 1) / block; }
};

// Bytes start..end (inclusive) of a URL. Empty if the file does not exist (404).
// Throws on any other failure. Called from worker threads.
using RangeFetcher = std::function<std::optional<Bytes>(const std::string& url, std::uint64_t start,
                                                        std::uint64_t end)>;

[[nodiscard]] RangeFetcher http_range_fetcher();

// Heights over EPSG:3035 bounds, NaN without data. Rows run north to south,
// pixels are 1 m; (x0, y0) is the centre of pixel (0, 0).
struct Window {
    Grid2D<float> heights;
    double x0 = 0;
    double y0 = 0;
};

// One ALS model (DSM or DTM): blocks from the BEV files, cached on disk under
// <cache>/als/<kind>/. The block files <name>/<n>.z are the raw deflate data,
// the same files as the Python simulator. The index is <name>.idx.bin.
class Layer {
public:
    Layer(std::string kind, std::string_view cache_dir,
          std::string url_template = std::string(bev_url), RangeFetcher fetch = {});

    [[nodiscard]] Window window(double x0, double y0, double x1, double y1);

private:
    [[nodiscard]] std::shared_ptr<const FileIndex> file_index(const std::string& name);
    [[nodiscard]] std::shared_ptr<const std::vector<float>> block_data(const std::string& name,
                                                                       const FileIndex& index,
                                                                       int number);
    [[nodiscard]] std::optional<Bytes> get(const std::string& name, std::uint64_t start,
                                           std::uint64_t end);
    [[nodiscard]] std::shared_ptr<const FileIndex> fetch_index(const std::string& name);

    std::string kind_;
    std::string url_;
    std::filesystem::path root_;
    RangeFetcher fetch_;
    std::mutex mutex_;
    std::map<std::string, std::shared_ptr<const FileIndex>> index_;
    using BlockKey = std::pair<std::string, int>;
    std::list<BlockKey> order_;
    std::map<BlockKey,
             std::pair<std::shared_ptr<const std::vector<float>>, std::list<BlockKey>::iterator>>
        blocks_;
};

// Heights on a 1 m EPSG:3035 grid; (x0, y0) is the centre of pixel (0, 0), rows run south.
class HeightGrid {
public:
    HeightGrid(Grid2D<float> heights, double x0, double y0)
        : heights_(std::move(heights)), x0_(x0), y0_(y0) {}

    // Bilinear height at lat/lon; NaN outside or without data. peak takes the
    // highest of the 3 x 3 pixels around each corner instead: at a roof edge
    // the bilinear value mixes in the ground.
    [[nodiscard]] float sample(double lat, double lon, bool peak = false) const;
    // The highest of each 3 x 3 neighbourhood, computed once. NaN where all are NaN.
    [[nodiscard]] const Grid2D<float>& peaks() const;

    [[nodiscard]] const Grid2D<float>& heights() const noexcept { return heights_; }
    [[nodiscard]] double x0() const noexcept { return x0_; }
    [[nodiscard]] double y0() const noexcept { return y0_; }

private:
    Grid2D<float> heights_;
    double x0_;
    double y0_;
    mutable std::once_flag peaks_once_;
    mutable Grid2D<float> peaks_;
};

// DSM minus DTM around an area: roofs and trees in metres above the ground.
class HeightAboveGround {
public:
    explicit HeightAboveGround(std::string_view cache_dir,
                               const std::string& url_template = std::string(bev_url),
                               const RangeFetcher& fetch = {});

    [[nodiscard]] std::shared_ptr<const HeightGrid> area(double lat0, double lon0, double lat1,
                                                         double lon1, double margin_m = 4.0);

private:
    Layer dsm_;
    Layer dtm_;
};

}  // namespace docksim::raster::als
