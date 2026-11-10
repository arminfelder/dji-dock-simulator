#pragma once

#include "osm/cells.hpp"
#include "raster/als.hpp"
#include "raster/mercator.hpp"
#include "render/geodata/terrain.hpp"
#include "render/rt/pieces.hpp"
#include "render/rt/slots.hpp"
#include "render/rt/vk.hpp"

#include <asio/thread_pool.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <vector>

// The ray tracing scene around a moving camera: terrain tiles and building
// cells. Each becomes one bottom-level structure, built in the background. While
// terrain tiles load, the old set stays on screen, so no holes appear. Building
// and tree pieces join the set when they are ready.
namespace docksim::render::rt {

// A terrain tile splits into its four children while the camera is closer than
// split x its width. The finest zoom is the DEM's finest.
inline constexpr double split = 1.5;
inline constexpr int coarse_zoom = 10;
// Built pieces kept for reuse when the camera comes back.
inline constexpr std::size_t max_pieces = 600;
// Triangle records the pool holds at first, 48 bytes each; it doubles when full.
inline constexpr std::size_t pool_triangles = 1'000'000;
// Instance custom indices have 24 bits; record slots must stay below.
inline constexpr std::size_t max_triangles = (1U << 24U) - 1U;
inline constexpr int build_workers = 3;

// One complete scene: the top level and the record buffer of its triangles.
// The top level holds only GPU addresses of its bottom levels; blases keeps
// them alive as long as this set lives.
struct SceneSet {
    std::set<PieceKey> keys;
    std::shared_ptr<vk::Accel> tlas;
    std::shared_ptr<vk::Buffer> records;
    std::size_t triangles = 0;
    std::vector<std::shared_ptr<vk::Accel>> blases;
};

struct SceneSources {
    std::shared_ptr<geodata::Terrain> terrain;
    // Null: no buildings.
    std::shared_ptr<osm::CellStore> cells;
    // Null: OSM roofs only.
    std::shared_ptr<raster::als::HeightAboveGround> scan;
    double building_radius = 1000.0;
    double far_radius = 3000.0;
    double scan_radius = 1500.0;
};

class Scene {
public:
    Scene(std::shared_ptr<vk::Context> ctx, SceneSources sources);
    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;
    Scene(Scene&&) = delete;
    Scene& operator=(Scene&&) = delete;
    ~Scene();

    // Positions count from here. A new origin drops every built piece.
    void set_origin(double lat, double lon, float z0);
    [[nodiscard]] std::optional<raster::LocalFrame> origin() const;

    // Roof height where the dock stands; 0 on open ground. Without wait, 0
    // until the cell has loaded in the background.
    [[nodiscard]] float home_roof(bool wait);

    // The scene for a camera at lat/lon; with wait, missing pieces are built
    // now. Null while no piece has geometry.
    [[nodiscard]] std::shared_ptr<const SceneSet> frame(double lat, double lon, double reach,
                                                        bool wait);

    [[nodiscard]] std::vector<PieceKey> wanted(double lat, double lon, double reach) const;

private:
    struct Piece {
        std::shared_ptr<vk::Accel> blas;
        std::size_t triangles = 0;
        // First triangle in the record pool; none without triangles.
        std::optional<std::size_t> slot;
    };

    void build(const PieceKey& key);
    void request(const std::vector<PieceKey>& keys);
    void request_home();
    // A pool slot; evicts unused pieces, then grows the pool. Call with the lock.
    std::size_t place(std::size_t triangles);
    bool evict();
    [[nodiscard]] std::shared_ptr<const SceneSet> assemble(const std::set<PieceKey>& keys);
    void tiles(double lat, double lon, double reach, std::vector<PieceKey>& out) const;
    void split_tile(const TileKey& key, double lat, double lon, double reach,
                    std::vector<PieceKey>& out) const;
    void cells(double lat, double lon, std::vector<PieceKey>& out) const;

    std::shared_ptr<vk::Context> ctx_;
    SceneSources sources_;
    int max_zoom_;
    mutable std::recursive_mutex mutex_;
    std::optional<raster::LocalFrame> origin_;
    float z0_ = 0.0F;
    // Bumped by every new origin; background builds for an old origin are dropped.
    std::uint64_t generation_ = 0;
    std::map<PieceKey, Piece> pieces_;
    std::vector<PieceKey> order_;
    std::set<PieceKey> pending_;
    std::set<PieceKey> wanted_;
    std::shared_ptr<const SceneSet> current_;
    std::optional<float> home_roof_;
    bool home_pending_ = false;
    std::shared_ptr<vk::Buffer> pool_;
    Slots slots_{pool_triangles};
    asio::thread_pool workers_{build_workers};
};

}  // namespace docksim::render::rt
