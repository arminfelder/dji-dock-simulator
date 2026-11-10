#pragma once

#include "osm/cells.hpp"
#include "osm/grid.hpp"

#include <docksim/config.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>

// Building grids around a moving camera: a fine near ring and a coarse far ring.
namespace docksim::osm {

// Near grid cell size, metres. Matches the finite-difference step of the surface normal.
inline constexpr double near_resolution_m = 0.5;
// Rebuild when the camera is closer than this share of the radius to the edge.
inline constexpr double rebuild_margin = 0.4;

using GridReady = std::function<void(const std::shared_ptr<const LocalGrid>&)>;

// One grid of radius_m around the camera, rebuilt as the camera moves.
class GridRing {
public:
    GridRing(std::shared_ptr<CellStore> cells, double radius_m, double resolution_m,
             std::string name, bool detail);
    GridRing(const GridRing&) = delete;
    GridRing& operator=(const GridRing&) = delete;
    GridRing(GridRing&&) = delete;
    GridRing& operator=(GridRing&&) = delete;
    // Waits for a running background load.
    ~GridRing();

    // The grid for a camera at lat/lon. With wait, a missing grid is built
    // now; a cold Overpass fetch can take minutes. Without wait, the build runs
    // in the background, and the caller gets the old grid while the camera is
    // still on it, else null. Thread-safe.
    [[nodiscard]] std::shared_ptr<const LocalGrid> grid(double lat, double lon, bool wait);

    // Called with each new grid, in the loading thread. A GPU renderer uploads here.
    void set_on_ready(GridReady callback);

    [[nodiscard]] double radius_m() const noexcept { return radius_m_; }

private:
    void load(double lat, double lon, const std::stop_token& stop = {});
    [[nodiscard]] std::vector<Footprint> footprints(double lat, double lon,
                                                    const std::stop_token& stop) const;

    std::shared_ptr<CellStore> cells_;
    double radius_m_;
    double resolution_m_;
    std::string name_;
    // Roof shapes and building parts. Off for the far ring: geometric LoD.
    bool detail_;
    std::mutex mutex_;
    std::shared_ptr<const LocalGrid> grid_;
    GridReady on_ready_;
    std::jthread loader_;
    bool loading_ = false;
    // A blocking load and the background load can run at once. Each takes a ticket when it starts;
    // a load publishes only if no newer one has published, so an old position never wins.
    std::uint64_t requested_ = 0;
    std::uint64_t published_ = 0;
};

class BuildingProvider {
public:
    // The far ring exists only if far_radius_m > radius_m.
    BuildingProvider(std::shared_ptr<CellStore> cells, double radius_m, double far_radius_m = 0.0,
                     double far_resolution_m = 3.0);

    // Near and far grids for a camera at lat/lon. wait applies to the near
    // grid only; the far grid covers many more Overpass cells and always loads
    // in the background. Empty while neither grid is ready.
    [[nodiscard]] std::optional<Buildings> grid(double lat, double lon, bool wait);

    void set_on_ready(const GridReady& callback);

    [[nodiscard]] GridRing& near() { return *near_; }
    [[nodiscard]] GridRing* far() { return far_.get(); }

private:
    std::unique_ptr<GridRing> near_;
    std::unique_ptr<GridRing> far_;
};

// From media.buildings, building_radius, building_far_radius, building_far_resolution.
// Null if media.buildings is off.
[[nodiscard]] std::unique_ptr<BuildingProvider> make_provider(
    const MediaConfig& media, std::shared_ptr<CellStore> cells = nullptr);

}  // namespace docksim::osm
