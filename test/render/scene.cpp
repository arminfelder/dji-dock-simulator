#include "scene.hpp"

#include "image/image.hpp"
#include "osm/cells.hpp"
#include "osm/overpass.hpp"

#ifdef DOCKSIM_WITH_VULKAN_RT
#include "render/rt/frame.hpp"
#endif
#include "util/json.hpp"
#include "util/numeric.hpp"

#include <cmath>
#include <format>
#include <fstream>
#include <numbers>
#include <random>
#include <stdexcept>

namespace docksim::test {

namespace {

constexpr double radians(double d) {
    return d * std::numbers::pi / 180.0;
}

struct TileCoord {
    int z;
    int x;
    int y;
};

// Latitude and longitude of the pixel centre (col, row) of one XYZ tile.
std::pair<double, double> pixel_latlon(const TileCoord& t, int col, int row) {
    const double n = 256.0 * std::exp2(t.z);
    const double px = (t.x * 256.0 + col + 0.5) / n;
    const double py = (t.y * 256.0 + row + 0.5) / n;
    return {std::atan(std::sinh(std::numbers::pi * (1 - 2 * py))) * 180.0 / std::numbers::pi,
            px * 360.0 - 180.0};
}

image::RgbImage imagery_tile(const TileCoord& t) {
    image::RgbImage img(256, 256);
    for (int row = 0; row < 256; ++row) {
        for (int col = 0; col < 256; ++col) {
            const auto [lat, lon] = pixel_latlon(t, col, row);
            const double u = (lon - home_lon) * 75'000.0;
            const double v = (lat - home_lat) * 111'000.0;
            const double field = num::floor_mod(std::floor(u / 25) + std::floor(v / 25), 2.0);
            const bool road = num::floor_mod(u, 120.0) < 6 || num::floor_mod(v, 160.0) < 5;
            image::Rgb rgb{static_cast<std::uint8_t>(std::clamp(
                               70 + 50 * field + num::floor_mod(0.02 * u, 30.0), 0.0, 255.0)),
                           static_cast<std::uint8_t>(110 + 30 * field),
                           static_cast<std::uint8_t>(60 + 20 * (1 - field))};
            if (road) rgb = {150, 150, 145};
            img.set(col, row, rgb);
        }
    }
    return img;
}

image::RgbImage dem_tile(const TileCoord& t) {
    image::RgbImage img(256, 256);
    for (int row = 0; row < 256; ++row) {
        for (int col = 0; col < 256; ++col) {
            const auto [lat, lon] = pixel_latlon(t, col, row);
            const double value = elevation(lat, lon) + 32768.0;
            const double r = std::floor(value / 256);
            const double g = std::floor(value - r * 256);
            const double b = std::floor((value - r * 256 - g) * 256);
            img.set(col, row,
                    {static_cast<std::uint8_t>(r), static_cast<std::uint8_t>(g),
                     static_cast<std::uint8_t>(b)});
        }
    }
    return img;
}

}  // namespace

render::geodata::FetchResult fake_get(const std::string& url) {
    // https://<host>/<z>/<x>/<y>.png
    const auto host_start = url.find("://") + 3;
    const auto path = url.find('/', host_start);
    const auto host = url.substr(host_start, path - host_start);
    TileCoord t{};
    if (std::sscanf(url.c_str() + path, "/%d/%d/%d.png", &t.z, &t.x, &t.y) != 3)
        return {.status = 404, .body = {}};
    return {.status = 200,
            .body = image::encode_png(host == "imagery.test" ? imagery_tile(t) : dem_tile(t))};
}

namespace {

class NoOverpass final : public osm::BuildingSource {
public:
    std::optional<std::vector<Json>> buildings(double /*south*/, double /*west*/, double /*north*/,
                                               double /*east*/,
                                               const std::stop_token& /*stop*/) override {
        throw std::logic_error(
            "render test tried to reach Overpass; write the cell in write_cells");
    }
};

struct Building {
    std::int64_t id;
    // South-west corner as metres east/north of home, size east/north.
    double e0;
    double n0;
    double width;
    double depth;
    Json tags;
};

const std::vector<Building>& buildings() {
    static const std::vector<Building> list{
        {101, 30, 20, 25, 15, {{"building", "yes"}, {"building:levels", "4"}}},
        {102, -60, 40, 18, 30, {{"building", "yes"}, {"height", "22 m"}}},
        {103, 10, -70, 40, 12, {{"building", "yes"}, {"roof:shape", "gabled"}}},
        {104, -40, -40, 12, 12, {{"building", "carport"}}},
        {105,
         60,
         -30,
         16,
         22,
         {{"building", "house"}, {"building:levels", "2"}, {"roof:shape", "hipped"}}},
        // An outline with a tower part: near the camera the part replaces the outline.
        {106, -70, -90, 30, 30, {{"building", "yes"}, {"height", "30"}}},
        {107, -60, -80, 10, 10, {{"building:part", "yes"}, {"height", "30"}}},
        {108, -70, -90, 30, 30, {{"building:part", "yes"}, {"height", "8"}}},
        // Facade styles: a weathered concrete flak tower without windows, and an office.
        {109,
         20,
         60,
         22,
         22,
         {{"building", "bunker"},
          {"building:material", "concrete"},
          {"historic", "ruins"},
          {"height", "40"}}},
        {110, -30, 15, 20, 14, {{"building", "office"}, {"building:levels", "6"}}},
    };
    return list;
}

Json ring(const Building& b) {
    constexpr double m_lat = 111'195.0;
    const double m_lon = m_lat * std::cos(radians(home_lat));
    const std::array<std::pair<double, double>, 5> corners{{{b.e0, b.n0},
                                                            {b.e0 + b.width, b.n0},
                                                            {b.e0 + b.width, b.n0 + b.depth},
                                                            {b.e0, b.n0 + b.depth},
                                                            {b.e0, b.n0}}};
    Json out = Json::array();
    for (const auto& [e, n] : corners)
        out.push_back(Json::array({home_lat + n / m_lat, home_lon + e / m_lon}));
    return out;
}

}  // namespace

void write_cells(const std::filesystem::path& cache, double radius_m) {
    const auto root = cache / osm::building_cache_dir;
    std::filesystem::create_directories(root);
    const double half_lat = radius_m / 111'195.0;
    const double half_lon = half_lat / std::cos(radians(home_lat));
    const auto cell = [](double v) { return static_cast<int>(std::floor(v / osm::cell_deg)); };
    for (int cy = cell(home_lat - half_lat); cy <= cell(home_lat + half_lat); ++cy) {
        for (int cx = cell(home_lon - half_lon); cx <= cell(home_lon + half_lon); ++cx) {
            Json items = Json::array();
            if (cy == cell(home_lat) && cx == cell(home_lon)) {
                for (const auto& b : buildings()) {
                    items.push_back({{"id", b.id},
                                     {"tags", b.tags},
                                     {"outer", Json::array({ring(b)})},
                                     {"inner", Json::array()}});
                }
            }
            std::ofstream(root / std::format("{}_{}.json", cy, cx)) << items.dump();
        }
    }
}

double elevation(double lat, double lon) {
    const double north = (lat - home_lat) * 111'000.0;
    const double east = (lon - home_lon) * 75'000.0;
    return 560.0 +
           40.0 * std::exp(-((north - 600) * (north - 600) + (east - 400) * (east - 400)) /
                           (300.0 * 300.0)) +
           0.01 * east;
}

OfflineScene::OfflineScene(double building_radius_m, [[maybe_unused]] bool ray_tracing) {
    std::random_device rd;
    cache_ = std::filesystem::temp_directory_path() / std::format("docksim-render-{:08x}", rd());
    std::filesystem::create_directories(cache_);
    // Cameras may stand up to 1 km from home; their building grid needs those cells too.
    write_cells(cache_, building_radius_m + 1000.0);

    auto cells = std::make_shared<osm::CellStore>(cache_, std::make_shared<NoOverpass>(),
                                                  osm::FootprintOptions{});
    render::Renderer::Sources s;
    s.imagery = std::make_shared<render::geodata::Imagery>("https://imagery.test/{z}/{x}/{y}.png",
                                                           19, cache_.string(), fake_get);
    s.terrain = std::make_shared<render::geodata::Terrain>("https://dem.test/{z}/{x}/{y}.png", 14,
                                                           cache_.string(), 0, 0.0, fake_get);
    s.buildings = std::make_shared<osm::BuildingProvider>(cells, building_radius_m);
    s.shadows = true;
    s.threads = 4;
#ifdef DOCKSIM_WITH_VULKAN_RT
    if (ray_tracing) {
        constexpr const char* lavapipe = "/usr/share/vulkan/icd.d/lvp_icd.json";
        if (std::getenv("VK_ICD_FILENAMES") == nullptr &&
            std::getenv("DOCKSIM_TEST_GPU") == nullptr && std::filesystem::exists(lavapipe)) {
            ::setenv("VK_ICD_FILENAMES", lavapipe, 1);
        }
        auto ctx = render::rt::vk::Context::open(std::nullopt, true);
        render::rt::SceneSources sources{.terrain = s.terrain,
                                         .cells = cells,
                                         .scan = nullptr,
                                         .building_radius = building_radius_m,
                                         .far_radius = building_radius_m,
                                         .scan_radius = 0.0};
        s.rt = std::make_shared<render::rt::FrameRenderer>(
            ctx, std::make_shared<render::rt::Scene>(ctx, std::move(sources)));
        s.buildings.reset();
    }
#endif
    renderer_ = std::make_unique<render::Renderer>(std::move(s));
}

OfflineScene::~OfflineScene() {
    renderer_.reset();
    std::error_code ec;
    std::filesystem::remove_all(cache_, ec);
}

sim::WallTime at(double hour) {
    const auto day = std::chrono::sys_days{std::chrono::year{2026} / 6 / 21};
    return day + std::chrono::duration_cast<std::chrono::seconds>(
                     std::chrono::duration<double>(hour * 3600.0));
}

render::Pose pose(double north_m, double height, double heading, double pitch, double hour) {
    return {.position = geo::point(home_lat + north_m / 111'195.0, home_lon),
            .height = units::above_home(height),
            .home = geo::point(home_lat, home_lon),
            .heading = heading * units::degree,
            .gimbal_pitch = pitch * units::degree,
            .time = at(hour)};
}

}  // namespace docksim::test
