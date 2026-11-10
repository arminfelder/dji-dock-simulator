#include "render/rt/scene.hpp"

#include "log/log.hpp"

#include <asio/post.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <tuple>

namespace docksim::render::rt {

namespace {

const auto& logger() {
    static const auto log = log::get("dock_sim.render");
    return log;
}

constexpr std::size_t record_bytes = record_size * sizeof(float);

std::string describe(const PieceKey& key) {
    return std::visit(
        [](const auto& k) -> std::string {
            using K = std::decay_t<decltype(k)>;
            if constexpr (std::is_same_v<K, TileKey>)
                return std::format("tile {}/{}/{}", k.z, k.tx, k.ty);
            if constexpr (std::is_same_v<K, CellKey>)
                return std::format("cell {}_{}{}", k.cy, k.cx, k.near ? " near" : "");
            if constexpr (std::is_same_v<K, TreesKey>)
                return std::format("trees {}_{}", k.cy, k.cx);
        },
        key);
}

std::shared_ptr<vk::Buffer> new_pool(vk::Context& ctx, std::size_t triangles) {
    return std::make_shared<vk::Buffer>(ctx, triangles * record_bytes,
                                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, vk::Memory::device);
}

}  // namespace

Scene::Scene(std::shared_ptr<vk::Context> ctx, SceneSources sources)
    : ctx_(std::move(ctx)),
      sources_(std::move(sources)),
      max_zoom_(std::max(sources_.terrain->max_zoom(), sources_.terrain->near_zoom())),
      pool_(new_pool(*ctx_, pool_triangles)) {
    sources_.far_radius = std::max(sources_.far_radius, sources_.building_radius);
}

Scene::~Scene() {
    {
        const std::scoped_lock lock(mutex_);
        ++generation_;
    }
    workers_.stop();
    workers_.join();
}

void Scene::set_origin(double lat, double lon, float z0) {
    const std::scoped_lock lock(mutex_);
    const raster::LocalFrame origin{.lat0 = lat, .lon0 = lon};
    if (origin_ && origin_->lat0 == lat && origin_->lon0 == lon && z0_ == z0) return;
    origin_ = origin;
    z0_ = z0;
    ++generation_;
    pieces_.clear();
    order_.clear();
    pending_.clear();
    current_.reset();
    home_roof_.reset();
    home_pending_ = false;
    pool_ = new_pool(*ctx_, pool_triangles);
    slots_ = Slots(pool_triangles);
}

std::optional<raster::LocalFrame> Scene::origin() const {
    const std::scoped_lock lock(mutex_);
    return origin_;
}

float Scene::home_roof(bool wait) {
    std::optional<raster::LocalFrame> origin;
    float z0 = 0.0F;
    {
        const std::scoped_lock lock(mutex_);
        if (home_roof_) return *home_roof_;
        if (!origin_ || !sources_.cells) return 0.0F;
        origin = origin_;
        z0 = z0_;
    }
    if (!wait) {
        request_home();
        return 0.0F;
    }
    const float roof =
        rt::home_roof(*sources_.terrain, *sources_.cells, *origin, z0, sources_.scan.get());
    const std::scoped_lock lock(mutex_);
    if (origin_ && origin_->lat0 == origin->lat0 && origin_->lon0 == origin->lon0)
        home_roof_ = roof;
    return roof;
}

void Scene::request_home() {
    const std::scoped_lock lock(mutex_);
    if (home_pending_) return;
    home_pending_ = true;
    asio::post(workers_, [this] {
        try {
            std::ignore = home_roof(true);
        } catch (const std::exception& e) {
            logger()->warn("ray tracing home roof failed: {}", e.what());
        }
        const std::scoped_lock relock(mutex_);
        home_pending_ = false;
    });
}

std::shared_ptr<const SceneSet> Scene::frame(double lat, double lon, double reach, bool wait) {
    const auto want = wanted(lat, lon, reach);
    std::vector<PieceKey> missing;
    {
        const std::scoped_lock lock(mutex_);
        wanted_ = std::set<PieceKey>(want.begin(), want.end());
        for (const auto& k : want) {
            if (!pieces_.contains(k)) missing.push_back(k);
        }
    }
    if (!missing.empty() && wait) {
        for (const auto& k : missing) build(k);
        missing.clear();
    } else if (!missing.empty()) {
        request(missing);
    }
    const std::scoped_lock lock(mutex_);
    std::set<PieceKey> ready;
    for (const auto& k : want) {
        if (pieces_.contains(k)) ready.insert(k);
    }
    if (current_ && current_->keys == ready) return current_;
    // A missing tile leaves a hole in the ground; a missing building cell does not.
    const bool holes = std::ranges::any_of(
        missing, [](const auto& k) { return std::holds_alternative<TileKey>(k); });
    if (holes && current_) return current_;
    // Building cells without buildings have no geometry; a scene needs some.
    if (std::ranges::none_of(ready, [&](const auto& k) { return pieces_.at(k).blas != nullptr; }))
        return nullptr;
    current_ = assemble(ready);
    return current_;
}

std::vector<PieceKey> Scene::wanted(double lat, double lon, double reach) const {
    std::vector<PieceKey> out;
    tiles(lat, lon, reach, out);
    cells(lat, lon, out);
    return out;
}

void Scene::tiles(double lat, double lon, double reach, std::vector<PieceKey>& out) const {
    const auto [px, py] = raster::global_pixel(lat, lon, coarse_zoom);
    const double span = reach / raster::tile_metres(lat, coarse_zoom) + 1;
    const auto t = static_cast<double>(raster::tile_px);
    const int tx0 = static_cast<int>(px / t - span);
    const int tx1 = static_cast<int>(px / t + span);
    const int ty0 = static_cast<int>(py / t - span);
    const int ty1 = static_cast<int>(py / t + span);
    for (int ty = ty0; ty <= ty1; ++ty) {
        for (int tx = tx0; tx <= tx1; ++tx)
            split_tile({.z = coarse_zoom, .tx = tx, .ty = ty}, lat, lon, reach, out);
    }
}

void Scene::split_tile(const TileKey& key, double lat, double lon, double reach,
                       std::vector<PieceKey>& out) const {
    const auto [px, py] = raster::global_pixel(lat, lon, key.z);
    const auto t = static_cast<double>(raster::tile_px);
    const double cx =
        std::clamp(px / t, static_cast<double>(key.tx), static_cast<double>(key.tx + 1));
    const double cy =
        std::clamp(py / t, static_cast<double>(key.ty), static_cast<double>(key.ty + 1));
    const double width = raster::tile_metres(lat, key.z);
    const double distance = std::hypot(cx - px / t, cy - py / t) * width;
    if (distance > reach) return;
    if (key.z < max_zoom_ && distance < split * width) {
        for (int dy = 0; dy < 2; ++dy) {
            for (int dx = 0; dx < 2; ++dx)
                split_tile({.z = key.z + 1, .tx = 2 * key.tx + dx, .ty = 2 * key.ty + dy}, lat, lon,
                           reach, out);
        }
        return;
    }
    out.emplace_back(key);
}

void Scene::cells(double lat, double lon, std::vector<PieceKey>& out) const {
    if (!sources_.cells) return;
    const raster::LocalFrame frame{.lat0 = lat, .lon0 = lon};
    const double half_lat = sources_.far_radius / frame.m_per_deg_lat();
    const double half_lon = sources_.far_radius / frame.m_per_deg_lon();
    const auto cell = [](double v) { return static_cast<int>(std::floor(v / osm::cell_deg)); };
    const double half_cell = osm::cell_deg * frame.m_per_deg_lat() / 2;
    for (int cy = cell(lat - half_lat); cy <= cell(lat + half_lat); ++cy) {
        for (int cx = cell(lon - half_lon); cx <= cell(lon + half_lon); ++cx) {
            const double c_lat = (cy + 0.5) * osm::cell_deg;
            const double c_lon = (cx + 0.5) * osm::cell_deg;
            const double d = std::hypot((c_lat - lat) * frame.m_per_deg_lat(),
                                        (c_lon - lon) * frame.m_per_deg_lon());
            if (d > sources_.far_radius + osm::cell_deg * frame.m_per_deg_lat()) continue;
            // Building parts cost few triangles; every cell keeps them. Near
            // cells get doors and shopfronts, and their roofs and trees from
            // the laser scan where there is one.
            const bool near = d < sources_.scan_radius + half_cell;
            out.emplace_back(CellKey{.cy = cy, .cx = cx, .near = near});
            if (near && sources_.scan != nullptr) out.emplace_back(TreesKey{.cy = cy, .cx = cx});
        }
    }
}

void Scene::request(const std::vector<PieceKey>& keys) {
    const std::scoped_lock lock(mutex_);
    for (const auto& key : keys) {
        if (!pending_.insert(key).second) continue;
        asio::post(workers_, [this, key] {
            try {
                build(key);
            } catch (const std::exception& e) {
                logger()->warn("ray tracing piece {} failed: {}", describe(key), e.what());
            }
            const std::scoped_lock relock(mutex_);
            pending_.erase(key);
        });
    }
}

void Scene::build(const PieceKey& key) {
    std::optional<raster::LocalFrame> origin;
    float z0 = 0.0F;
    std::uint64_t generation = 0;
    {
        const std::scoped_lock lock(mutex_);
        if (!origin_ || pieces_.contains(key)) return;
        origin = origin_;
        z0 = z0_;
        generation = generation_;
    }
    const auto started = std::chrono::steady_clock::now();
    Mesh mesh =
        build_piece(key, *sources_.terrain, sources_.cells.get(), *origin, z0, sources_.scan.get());
    curve(mesh, *origin);
    Piece piece{.blas = nullptr, .triangles = mesh.triangles(), .slot = std::nullopt};
    if (!mesh.empty()) {
        piece.blas = std::make_shared<vk::Accel>(
            ctx_->build_blas(std::span(mesh.vertices.front().data(), mesh.vertices.size() * 3),
                             std::span(mesh.indices.front().data(), mesh.indices.size() * 3)));
    }
    const std::scoped_lock lock(mutex_);
    if (generation != generation_ || pieces_.contains(key)) return;
    if (!mesh.empty()) {
        const std::size_t slot = place(mesh.triangles());
        ctx_->upload(*pool_, std::as_bytes(std::span(mesh.records)), slot * record_bytes);
        piece.slot = slot;
    }
    pieces_.emplace(key, std::move(piece));
    order_.push_back(key);
    while (pieces_.size() > max_pieces && evict()) {
    }
    logger()->debug(
        "ray tracing piece {}: {} triangles, {:.2f} s", describe(key), mesh.triangles(),
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
}

std::size_t Scene::place(std::size_t triangles) {
    while (true) {
        if (const auto slot = slots_.alloc(triangles)) return *slot;
        if (evict()) continue;
        const std::size_t capacity =
            std::min(max_triangles, std::max(2 * slots_.capacity(), slots_.capacity() + triangles));
        if (capacity <= slots_.capacity())
            throw std::runtime_error("ray tracing record pool is full");
        // A larger buffer with the old contents. Scene sets on the old buffer keep it.
        auto grown = new_pool(*ctx_, capacity);
        ctx_->copy(*pool_, *grown, slots_.capacity() * record_bytes);
        pool_ = std::move(grown);
        slots_.grow(capacity);
    }
}

bool Scene::evict() {
    // The oldest piece that neither the current scene nor the latest frame uses.
    for (auto it = order_.begin(); it != order_.end(); ++it) {
        const auto key = *it;
        if ((current_ && current_->keys.contains(key)) || wanted_.contains(key)) continue;
        const auto& piece = pieces_.at(key);
        if (piece.slot) slots_.release(*piece.slot, piece.triangles);
        pieces_.erase(key);
        order_.erase(it);
        return true;
    }
    return false;
}

std::shared_ptr<const SceneSet> Scene::assemble(const std::set<PieceKey>& keys) {
    auto set = std::make_shared<SceneSet>();
    set->keys = keys;
    set->records = pool_;
    std::vector<vk::Instance> instances;
    for (const auto& key : keys) {
        const auto& piece = pieces_.at(key);
        set->triangles += piece.triangles;
        if (!piece.blas) continue;
        set->blases.push_back(piece.blas);
        instances.push_back({piece.blas.get(), static_cast<std::uint32_t>(piece.slot.value())});
    }
    set->tlas = std::make_shared<vk::Accel>(ctx_->build_tlas(instances));
    logger()->debug("ray tracing scene: {} pieces, {} triangles", instances.size(), set->triangles);
    return set;
}

}  // namespace docksim::render::rt
