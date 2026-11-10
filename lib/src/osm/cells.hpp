#pragma once

#include "osm/features.hpp"
#include "osm/footprints.hpp"
#include "osm/overpass.hpp"

#include <docksim/config.hpp>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stop_token>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

// Footprints per 0.01 x 0.01 degree cell, cached in memory and on disk. The
// disk layout is <cache_dir>/<building_cache_dir>/<cy>_<cx>.json with a list of
// {id, tags, outer, inner}.
namespace docksim::osm {

// Change the name when the cached content changes: then every cell is fetched again.
// v2: the split rings of multipolygons are joined.
inline constexpr std::string_view building_cache_dir = "osm-buildings-v2";

inline constexpr double cell_deg = 0.01;
// A cached cell older than this is fetched again, so that new buildings
// appear. If no server answers, the old cell stays in use.
inline constexpr std::chrono::seconds max_cell_age{90LL * 24 * 3600};
// A cell that no server delivered is not asked for again before this.
inline constexpr std::chrono::seconds failed_retry{300};
// prefetch asks for at most this many cells per side in one request. A dense
// city cell is about 0.5 MB, so 4 x 4 stays far below the declared maxsize.
inline constexpr int fetch_block_cells = 4;

using CellKey = std::pair<int, int>;
using Cell = std::shared_ptr<const std::vector<Footprint>>;
using FeatureCell = std::shared_ptr<const std::vector<Feature>>;

// The cell that holds a point: (floor(lat / cell_deg), floor(lon / cell_deg)).
[[nodiscard]] CellKey cell_of(double lat, double lon);

// Expand a leading ~ to $HOME.
[[nodiscard]] std::filesystem::path expand_user(const std::string& path);

class CellStore {
public:
    // cache_dir as in media.cache_dir; the cells go into cache_dir/building_cache_dir.
    CellStore(const std::filesystem::path& cache_dir, std::shared_ptr<BuildingSource> source,
              FootprintOptions options);

    // Footprints of one cell. A cell that no server delivers is tried again
    // after failed_retry. Thread-safe; the network is used outside the lock.
    // Throws Stopped when stop is requested during a fetch.
    [[nodiscard]] Cell footprints(int cy, int cx, const std::stop_token& stop = {});
    // Entrances and shops of one cell, in cache_dir/osm-features with the same
    // retry rules. Empty, never missing, when no server answers.
    [[nodiscard]] FeatureCell features(int cy, int cx, const std::stop_token& stop = {});
    // The features of a cell without waiting for the network: null if neither
    // memory nor disk has them. Then a background thread fetches them for a later call.
    [[nodiscard]] FeatureCell cached_features(int cy, int cx);
    // Fetches the cells from (cy0, cx0) to (cy1, cx1) whose buildings the disk
    // lacks or holds stale, buildings and features together, in blocks of
    // fetch_block_cells. A block that no server delivers is halved until one
    // cell is left; that cell is marked failed. Later footprints and features
    // calls read the cache. Throws Stopped when stop is requested.
    void prefetch(int cy0, int cx0, int cy1, int cx1, const std::stop_token& stop = {});

    [[nodiscard]] std::filesystem::path path(int cy, int cx) const;

private:
    [[nodiscard]] static std::optional<std::vector<Json>> read(const std::filesystem::path& file);
    [[nodiscard]] static bool stale(const std::filesystem::path& file);
    static void write(const std::filesystem::path& file, const std::vector<Json>& raw);
    [[nodiscard]] std::filesystem::path feature_path(int cy, int cx) const;
    // Fetches the claimed cells inside the box. Halves the box when it fails.
    void fetch_block(const std::set<CellKey>& claimed, int cy0, int cx0, int cy1, int cx1,
                     const std::stop_token& stop);

    std::filesystem::path root_;
    std::shared_ptr<BuildingSource> source_;
    FootprintOptions options_;
    std::mutex mutex_;
    std::map<CellKey, Cell> cells_;
    std::map<CellKey, std::chrono::steady_clock::time_point> failed_;
    std::filesystem::path feature_root_;
    std::map<CellKey, FeatureCell> feature_cells_;
    std::map<CellKey, std::chrono::steady_clock::time_point> feature_failed_;
    // Cells that one thread fetches now. Other callers wait on fetched_, not on a second request.
    std::set<CellKey> fetching_;
    std::set<CellKey> feature_fetching_;
    std::condition_variable_any fetched_;
    // Cells waiting for the background fetch. The worker starts on the first request.
    std::mutex queue_mutex_;
    std::condition_variable_any queue_changed_;
    std::deque<CellKey> queue_;
    std::set<CellKey> queued_;
    // Last member: it stops and joins before the state it uses goes away.
    std::jthread feature_worker_;
};

// The cell's elements with the extras that belong to it: geometry that OSM
// lacks, from lib/resources/osm/*.json. add_inner adds inner rings to an
// element by id; elements are new parts, kept in the cell of their first
// vertex. An element with "requires": id is added only where that element exists.
[[nodiscard]] std::vector<Json> with_extras(std::vector<Json> raw, int cy, int cx);

// A CellStore on the configured cache and Overpass mirrors.
[[nodiscard]] std::shared_ptr<CellStore> make_cell_store(const MediaConfig& media);

}  // namespace docksim::osm
