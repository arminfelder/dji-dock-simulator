#include "osm/provider.hpp"

#include "log/log.hpp"
#include "raster/mercator.hpp"

#include <chrono>
#include <cmath>

namespace docksim::osm {

namespace {

const auto& logger() {
    static const auto log = log::get("dock_sim.osm");
    return log;
}

}  // namespace

GridRing::GridRing(std::shared_ptr<CellStore> cells, double radius_m, double resolution_m,
                   std::string name, bool detail)
    : cells_(std::move(cells)),
      radius_m_(radius_m),
      resolution_m_(resolution_m),
      name_(std::move(name)),
      detail_(detail) {}

GridRing::~GridRing() {
    // A far ring can take minutes of Overpass fetches; do not wait for it.
    loader_.request_stop();
    if (loader_.joinable()) loader_.join();
}

void GridRing::set_on_ready(GridReady callback) {
    const std::scoped_lock lock(mutex_);
    on_ready_ = std::move(callback);
}

std::shared_ptr<const LocalGrid> GridRing::grid(double lat, double lon, bool wait) {
    std::shared_ptr<const LocalGrid> current;
    {
        const std::scoped_lock lock(mutex_);
        current = grid_;
    }
    if (current && current->contains(lat, lon, radius_m_ * rebuild_margin)) return current;
    if (wait) {
        load(lat, lon);
        const std::scoped_lock lock(mutex_);
        return grid_;
    }
    {
        const std::scoped_lock lock(mutex_);
        if (!loading_) {
            loading_ = true;
            if (loader_.joinable()) loader_.join();
            loader_ = std::jthread([this, lat, lon](const std::stop_token& stop) {
                load(lat, lon, stop);
                const std::scoped_lock done(mutex_);
                loading_ = false;
            });
        }
    }
    return current && current->contains(lat, lon, 0.0) ? current : nullptr;
}

std::vector<Footprint> GridRing::footprints(double lat, double lon,
                                            const std::stop_token& stop) const {
    const raster::LocalFrame frame{.lat0 = lat, .lon0 = lon};
    const double half_lat = radius_m_ / frame.m_per_deg_lat();
    const double half_lon = radius_m_ / frame.m_per_deg_lon();
    std::vector<Footprint> found;
    const auto [cy0, cx0] = cell_of(lat - half_lat, lon - half_lon);
    const auto [cy1, cx1] = cell_of(lat + half_lat, lon + half_lon);
    cells_->prefetch(cy0, cx0, cy1, cx1, stop);
    for (int cy = cy0; cy <= cy1; ++cy) {
        for (int cx = cx0; cx <= cx1; ++cx) {
            if (stop.stop_requested()) throw Stopped();
            const auto cell = cells_->footprints(cy, cx, stop);
            found.insert(found.end(), cell->begin(), cell->end());
        }
    }
    return found;
}

void GridRing::load(double lat, double lon, const std::stop_token& stop) {
    using clock = std::chrono::steady_clock;
    const auto started = clock::now();
    std::uint64_t ticket = 0;
    {
        const std::scoped_lock lock(mutex_);
        ticket = ++requested_;
    }
    std::shared_ptr<const LocalGrid> built;
    std::size_t count = 0;
    clock::time_point fetched;
    try {
        const auto items = footprints(lat, lon, stop);
        count = items.size();
        fetched = clock::now();
        built = std::make_shared<const LocalGrid>(
            rasterize(items, lat, lon, radius_m_, resolution_m_, detail_));
    } catch (const Stopped&) {
        return;
    } catch (const std::exception& e) {
        logger()->warn("{} building grid failed, rendering without it: {}", name_, e.what());
        return;
    }
    GridReady ready;
    {
        const std::scoped_lock lock(mutex_);
        if (ticket < published_) {
            logger()->debug("{} building grid for an older position dropped", name_);
            return;
        }
        published_ = ticket;
        grid_ = built;
        ready = on_ready_;
    }
    if (ready) ready(built);
    const auto seconds = [](auto d) { return std::chrono::duration<double>(d).count(); };
    logger()->info("{} building grid ready: {} footprints, cells {:.1f} s, raster {:.1f} s", name_,
                   count, seconds(fetched - started), seconds(clock::now() - fetched));
}

BuildingProvider::BuildingProvider(std::shared_ptr<CellStore> cells, double radius_m,
                                   double far_radius_m, double far_resolution_m)
    : near_(std::make_unique<GridRing>(cells, radius_m, near_resolution_m, "near", true)) {
    if (far_radius_m > radius_m) {
        far_ = std::make_unique<GridRing>(std::move(cells), far_radius_m, far_resolution_m, "far",
                                          false);
    }
}

std::optional<Buildings> BuildingProvider::grid(double lat, double lon, bool wait) {
    auto near = near_->grid(lat, lon, wait);
    auto far = far_ ? far_->grid(lat, lon, false) : nullptr;
    if (!near && !far) return std::nullopt;
    return Buildings{.near = std::move(near), .far = std::move(far)};
}

void BuildingProvider::set_on_ready(const GridReady& callback) {
    near_->set_on_ready(callback);
    if (far_) far_->set_on_ready(callback);
}

std::unique_ptr<BuildingProvider> make_provider(const MediaConfig& media,
                                                std::shared_ptr<CellStore> cells) {
    if (!media.buildings) return nullptr;
    if (!cells) cells = make_cell_store(media);
    return std::make_unique<BuildingProvider>(std::move(cells), media.building_radius,
                                              media.building_far_radius,
                                              media.building_far_resolution);
}

}  // namespace docksim::osm
