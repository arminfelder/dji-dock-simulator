#include "osm/cells.hpp"

#include "log/log.hpp"

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <format>
#include <fstream>
#include <map>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <stop_token>
#include <tuple>

namespace docksim::osm {

namespace {

const auto& logger() {
    static const auto log = log::get("dock_sim.osm");
    return log;
}

constexpr char flakturm_augarten[] = {
#embed "../../resources/osm/flakturm_augarten.json"
    , 0};

struct Extras {
    std::map<std::int64_t, std::vector<Json>> add_inner;
    std::vector<Json> elements;
};

const Extras& extras() {
    static const Extras parsed = [] {
        Extras out;
        for (const char* text : {flakturm_augarten}) {
            const auto doc = Json::parse(text);
            for (const auto& [key, rings] : doc.value("add_inner", Json::object()).items()) {
                auto& list = out.add_inner[std::stoll(key)];
                for (const auto& r : rings) list.push_back(r);
            }
            for (const auto& e : doc.value("elements", Json::array())) out.elements.push_back(e);
        }
        return out;
    }();
    return parsed;
}

std::int64_t id_of(const Json& item) {
    const auto it = item.find("id");
    return it != item.end() && it->is_number() ? it->get<std::int64_t>() : 0;
}

FeatureCell to_features(const std::vector<Json>& raw) {
    std::vector<Feature> found;
    for (const auto& item : raw) {
        if (const auto f = feature_from_cache(item)) found.push_back(*f);
    }
    return std::make_shared<const std::vector<Feature>>(std::move(found));
}

// Waits while another thread fetches key. Returns the value when it is ready.
// Returns nullopt when the caller has claimed key and must fetch it.
template <class Value>
std::optional<Value> ready_or_claim(std::unique_lock<std::mutex>& lock,
                                    std::condition_variable_any& fetched,
                                    std::set<CellKey>& fetching,
                                    const std::map<CellKey, Value>& ready, const CellKey& key,
                                    const std::stop_token& stop) {
    for (;;) {
        if (const auto it = ready.find(key); it != ready.end()) return it->second;
        if (fetching.insert(key).second) return std::nullopt;
        if (!fetched.wait(lock, stop, [&] { return !fetching.contains(key); })) throw Stopped{};
    }
}

class FetchClaim {
public:
    FetchClaim(std::mutex& mutex, std::condition_variable_any& fetched, std::set<CellKey>& fetching,
               CellKey key)
        : mutex_(mutex), fetched_(fetched), fetching_(fetching), key_(key) {}
    FetchClaim(const FetchClaim&) = delete;
    FetchClaim& operator=(const FetchClaim&) = delete;
    FetchClaim(FetchClaim&&) = delete;
    FetchClaim& operator=(FetchClaim&&) = delete;
    ~FetchClaim() {
        {
            const std::scoped_lock lock(mutex_);
            fetching_.erase(key_);
        }
        fetched_.notify_all();
    }

private:
    std::mutex& mutex_;
    std::condition_variable_any& fetched_;
    std::set<CellKey>& fetching_;
    CellKey key_;
};

}  // namespace

CellKey cell_of(double lat, double lon) {
    return {static_cast<int>(std::floor(lat / cell_deg)),
            static_cast<int>(std::floor(lon / cell_deg))};
}

std::filesystem::path expand_user(const std::string& path) {
    if (path == "~" || path.starts_with("~/")) {
        const char* home = std::getenv("HOME");
        return std::filesystem::path(home != nullptr ? home : "") /
               path.substr(path.size() > 1 ? 2 : 1);
    }
    return path;
}

std::vector<Json> with_extras(std::vector<Json> raw, int cy, int cx) {
    const auto& ex = extras();
    if (ex.add_inner.empty() && ex.elements.empty()) return raw;
    std::set<std::int64_t> present;
    for (auto& item : raw) {
        present.insert(id_of(item));
        if (const auto it = ex.add_inner.find(id_of(item)); it != ex.add_inner.end()) {
            if (!item.contains("inner") || !item["inner"].is_array()) item["inner"] = Json::array();
            for (const auto& ring : it->second) item["inner"].push_back(ring);
        }
    }
    for (const auto& element : ex.elements) {
        const auto& first = element["outer"][0][0];
        if (cell_of(first[0].get<double>(), first[1].get<double>()) != CellKey{cy, cx}) continue;
        // A pit floor without the block around it would stand as a free pole.
        if (element.contains("requires") &&
            !present.contains(element["requires"].get<std::int64_t>()))
            continue;
        Json copy = element;
        copy.erase("requires");
        raw.push_back(std::move(copy));
    }
    return raw;
}

CellStore::CellStore(const std::filesystem::path& cache_dir, std::shared_ptr<BuildingSource> source,
                     FootprintOptions options)
    : root_(cache_dir / building_cache_dir),
      source_(std::move(source)),
      options_(std::move(options)),
      feature_root_(cache_dir / "osm-features") {}

std::filesystem::path CellStore::path(int cy, int cx) const {
    return root_ / std::format("{}_{}.json", cy, cx);
}

std::filesystem::path CellStore::feature_path(int cy, int cx) const {
    return feature_root_ / std::format("{}_{}.json", cy, cx);
}

void CellStore::prefetch(int cy0, int cx0, int cy1, int cx1, const std::stop_token& stop) {
    const auto now = std::chrono::steady_clock::now();
    // Only missing buildings start a request. Features come with it; a cell
    // that lacks only features keeps the lazy per-cell fetch.
    std::vector<CellKey> wanted;
    for (int cy = cy0; cy <= cy1; ++cy) {
        for (int cx = cx0; cx <= cx1; ++cx) {
            if (stale(path(cy, cx))) wanted.emplace_back(cy, cx);
        }
    }
    std::set<CellKey> claimed;
    {
        const std::scoped_lock lock(mutex_);
        for (const auto& key : wanted) {
            if (fetching_.contains(key) || feature_fetching_.contains(key)) continue;
            if (const auto f = failed_.find(key); f != failed_.end() && now < f->second) continue;
            fetching_.insert(key);
            feature_fetching_.insert(key);
            claimed.insert(key);
        }
    }
    if (claimed.empty()) return;
    class Release {
    public:
        Release(CellStore& store, const std::set<CellKey>& keys) : store_(store), keys_(keys) {}
        Release(const Release&) = delete;
        Release& operator=(const Release&) = delete;
        Release(Release&&) = delete;
        Release& operator=(Release&&) = delete;
        ~Release() {
            {
                const std::scoped_lock lock(store_.mutex_);
                for (const auto& key : keys_) {
                    store_.fetching_.erase(key);
                    store_.feature_fetching_.erase(key);
                }
            }
            store_.fetched_.notify_all();
        }

    private:
        CellStore& store_;
        const std::set<CellKey>& keys_;
    };
    const Release release(*this, claimed);
    for (int by = cy0; by <= cy1; by += fetch_block_cells) {
        for (int bx = cx0; bx <= cx1; bx += fetch_block_cells) {
            fetch_block(claimed, by, bx, std::min(by + fetch_block_cells - 1, cy1),
                        std::min(bx + fetch_block_cells - 1, cx1), stop);
        }
    }
}

void CellStore::fetch_block(const std::set<CellKey>& claimed, int cy0, int cx0, int cy1, int cx1,
                            const std::stop_token& stop) {
    // Shrink the box to the claimed cells in it.
    int lo_y = cy1 + 1;
    int lo_x = cx1 + 1;
    int hi_y = cy0 - 1;
    int hi_x = cx0 - 1;
    for (const auto& [cy, cx] : claimed) {
        if (cy < cy0 || cy > cy1 || cx < cx0 || cx > cx1) continue;
        lo_y = std::min(lo_y, cy);
        lo_x = std::min(lo_x, cx);
        hi_y = std::max(hi_y, cy);
        hi_x = std::max(hi_x, cx);
    }
    if (hi_y < lo_y) return;
    const auto inside = [&](const CellKey& key) {
        return key.first >= lo_y && key.first <= hi_y && key.second >= lo_x &&
               key.second <= hi_x && claimed.contains(key);
    };

    const auto found = source_->area(lo_y * cell_deg, lo_x * cell_deg, (hi_y + 1) * cell_deg,
                                     (hi_x + 1) * cell_deg, stop);
    if (!found) {
        if (hi_y == lo_y && hi_x == lo_x) {
            {
                const std::scoped_lock lock(mutex_);
                const auto until = std::chrono::steady_clock::now() + failed_retry;
                failed_[{lo_y, lo_x}] = until;
                feature_failed_[{lo_y, lo_x}] = until;
            }
            logger()->error("no overpass server answered for cell {},{}; buildings missing", lo_y,
                            lo_x);
            return;
        }
        logger()->info("overpass: cells {},{} to {},{} failed, trying halves", lo_y, lo_x, hi_y,
                       hi_x);
        if (hi_y - lo_y >= hi_x - lo_x) {
            const int mid = lo_y + ((hi_y - lo_y) / 2);
            fetch_block(claimed, lo_y, lo_x, mid, hi_x, stop);
            fetch_block(claimed, mid + 1, lo_x, hi_y, hi_x, stop);
        } else {
            const int mid = lo_x + ((hi_x - lo_x) / 2);
            fetch_block(claimed, lo_y, lo_x, hi_y, mid, stop);
            fetch_block(claimed, lo_y, mid + 1, hi_y, hi_x, stop);
        }
        return;
    }

    // A building goes into every cell its outline box touches, as a query per
    // cell would return it. A feature is a point and goes into one cell.
    std::map<CellKey, std::vector<Json>> buildings;
    std::map<CellKey, std::vector<Json>> features;
    for (int cy = lo_y; cy <= hi_y; ++cy) {
        for (int cx = lo_x; cx <= hi_x; ++cx) {
            if (!inside({cy, cx})) continue;
            buildings[{cy, cx}];
            features[{cy, cx}];
        }
    }
    for (const auto& item : found->buildings) {
        double south = 90;
        double west = 180;
        double north = -90;
        double east = -180;
        for (const auto& ring : item.value("outer", Json::array())) {
            for (const auto& p : ring) {
                const double lat = p[0].get<double>();
                const double lon = p[1].get<double>();
                south = std::min(south, lat);
                north = std::max(north, lat);
                west = std::min(west, lon);
                east = std::max(east, lon);
            }
        }
        // An element without an outline has no cell. It draws nothing.
        if (north < south) continue;
        const auto [y0, x0] = cell_of(south, west);
        const auto [y1, x1] = cell_of(north, east);
        for (int cy = y0; cy <= y1; ++cy) {
            for (int cx = x0; cx <= x1; ++cx) {
                if (inside({cy, cx})) buildings[{cy, cx}].push_back(item);
            }
        }
    }
    if (found->features) {
        for (const auto& item : *found->features) {
            const auto key = cell_of(item["lat"].get<double>(), item["lon"].get<double>());
            if (inside(key)) features[key].push_back(item);
        }
    }
    for (const auto& [key, raw] : buildings) write(path(key.first, key.second), raw);
    if (found->features) {
        for (const auto& [key, raw] : features) write(feature_path(key.first, key.second), raw);
    } else {
        const std::scoped_lock lock(mutex_);
        const auto until = std::chrono::steady_clock::now() + failed_retry;
        for (const auto& [key, raw] : features) feature_failed_[key] = until;
    }
    logger()->debug("overpass: cells {},{} to {},{} in one request, {} buildings", lo_y, lo_x,
                    hi_y, hi_x, found->buildings.size());
}

bool CellStore::stale(const std::filesystem::path& file) {
    std::error_code ec;
    const auto written = std::filesystem::last_write_time(file, ec);
    if (ec) return true;
    return std::filesystem::file_time_type::clock::now() - written > max_cell_age;
}

std::optional<std::vector<Json>> CellStore::read(const std::filesystem::path& file) {
    const std::ifstream in(file);
    if (!in) return std::nullopt;
    std::ostringstream text;
    text << in.rdbuf();
    const auto data = Json::parse(text.str(), nullptr, false);
    if (data.is_discarded()) {
        logger()->warn("cache file {} is unreadable, fetching again", file.string());
        return std::nullopt;
    }
    if (!data.is_array()) return std::nullopt;
    return std::vector<Json>(data.begin(), data.end());
}

void CellStore::write(const std::filesystem::path& file, const std::vector<Json>& raw) {
    // Write and rename, so that a crash never leaves a half file behind.
    std::filesystem::create_directories(file.parent_path());
    const auto tmp = file.parent_path() /
                     std::format(".{}.{}.tmp", file.stem().string(), std::random_device{}());
    {
        std::ofstream out(tmp);
        out << Json(raw).dump();
        if (!out) throw std::runtime_error(std::format("cannot write {}", tmp.string()));
    }
    std::filesystem::rename(tmp, file);
}

Cell CellStore::footprints(int cy, int cx, const std::stop_token& stop) {
    const CellKey key{cy, cx};
    const auto now = std::chrono::steady_clock::now();
    bool recently_failed = false;
    {
        std::unique_lock lock(mutex_);
        if (auto ready = ready_or_claim(lock, fetched_, fetching_, cells_, key, stop))
            return *ready;
        const auto f = failed_.find(key);
        recently_failed = f != failed_.end() && now < f->second;
    }
    const FetchClaim claim(mutex_, fetched_, fetching_, key);
    const auto file = path(cy, cx);
    const auto cached = read(file);
    if (recently_failed && !cached) return std::make_shared<const std::vector<Footprint>>();
    std::vector<Json> raw = cached.value_or(std::vector<Json>{});
    if (!recently_failed && (!cached || stale(file))) {
        const double south = cy * cell_deg;
        const double west = cx * cell_deg;
        auto fresh = source_->buildings(south, west, south + cell_deg, west + cell_deg, stop);
        if (fresh) {
            write(file, *fresh);
            raw = std::move(*fresh);
        } else {
            const std::scoped_lock lock(mutex_);
            failed_[key] = now + failed_retry;
            if (cached) {
                logger()->warn("cell {},{} is stale and no server answered; using the old one", cy,
                               cx);
            } else {
                logger()->error("no overpass server answered for cell {},{}; buildings missing", cy,
                                cx);
                return std::make_shared<const std::vector<Footprint>>();
            }
        }
    }
    std::vector<Footprint> cell;
    for (const auto& item : with_extras(std::move(raw), cy, cx))
        cell.push_back(from_cache(item, options_));
    auto shared = std::make_shared<const std::vector<Footprint>>(std::move(cell));
    const std::scoped_lock lock(mutex_);
    return cells_.try_emplace(key, std::move(shared)).first->second;
}

FeatureCell CellStore::features(int cy, int cx, const std::stop_token& stop) {
    const CellKey key{cy, cx};
    const auto now = std::chrono::steady_clock::now();
    bool recently_failed = false;
    {
        std::unique_lock lock(mutex_);
        if (auto ready =
                ready_or_claim(lock, fetched_, feature_fetching_, feature_cells_, key, stop))
            return *ready;
        const auto f = feature_failed_.find(key);
        recently_failed = f != feature_failed_.end() && now < f->second;
    }
    const FetchClaim claim(mutex_, fetched_, feature_fetching_, key);
    const auto file = feature_path(cy, cx);
    const auto cached = read(file);
    std::vector<Json> raw = cached.value_or(std::vector<Json>{});
    bool known = cached.has_value();
    if (!recently_failed && (!cached || stale(file))) {
        const double south = cy * cell_deg;
        const double west = cx * cell_deg;
        if (auto fresh = source_->features(south, west, south + cell_deg, west + cell_deg, stop)) {
            write(file, *fresh);
            raw = std::move(*fresh);
            known = true;
        } else {
            const std::scoped_lock lock(mutex_);
            feature_failed_[key] = now + failed_retry;
            logger()->warn("no overpass server answered for the entrances and shops of cell {},{}",
                           cy, cx);
        }
    }
    auto shared = to_features(raw);
    // A cell without data is not kept, so that the retry after failed_retry can fill it.
    if (!known) return shared;
    const std::scoped_lock lock(mutex_);
    return feature_cells_.try_emplace(key, std::move(shared)).first->second;
}

FeatureCell CellStore::cached_features(int cy, int cx) {
    const CellKey key{cy, cx};
    {
        const std::scoped_lock lock(mutex_);
        if (const auto it = feature_cells_.find(key); it != feature_cells_.end()) return it->second;
    }
    const auto file = feature_path(cy, cx);
    const auto cached = read(file);
    if (cached && !stale(file)) return features(cy, cx);
    // A stale cell is used as it is until the background fetch replaces it.
    FeatureCell old = cached ? to_features(*cached) : nullptr;
    const std::scoped_lock lock(queue_mutex_);
    if (queued_.insert(key).second) queue_.push_back(key);
    if (!feature_worker_.joinable()) {
        feature_worker_ = std::jthread([this](const std::stop_token& stop) {
            for (;;) {
                CellKey next;
                {
                    std::unique_lock wait(queue_mutex_);
                    if (!queue_changed_.wait(wait, stop, [this] { return !queue_.empty(); }))
                        return;
                    next = queue_.front();
                    queue_.pop_front();
                }
                try {
                    std::ignore = features(next.first, next.second, stop);
                } catch (const Stopped&) {
                    return;
                } catch (const std::exception& e) {
                    logger()->warn("entrances and shops of cell {},{}: {}", next.first, next.second,
                                   e.what());
                }
                const std::scoped_lock done(queue_mutex_);
                queued_.erase(next);
            }
        });
    }
    queue_changed_.notify_one();
    return old;
}

std::shared_ptr<CellStore> make_cell_store(const MediaConfig& media) {
    return std::make_shared<CellStore>(expand_user(media.cache_dir),
                                       std::make_shared<OverpassClient>(media.overpass_urls),
                                       FootprintOptions{
                                           .default_height = media.building_default_height,
                                           .level_height = media.building_level_height,
                                           .overrides = media.building_heights,
                                           .default_style = media.building_default_style,
                                       });
}

}  // namespace docksim::osm
