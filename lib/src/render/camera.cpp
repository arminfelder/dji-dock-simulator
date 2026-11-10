#include "render/camera.hpp"

#include "device/geo.hpp"
#include "raster/mercator.hpp"

#include <cmath>
#include <list>
#include <mutex>
#include <tuple>

namespace docksim::render {

namespace {

// Ray-frame cache entries. Zoom changes the field of view, so each step is a new entry.
constexpr std::size_t frame_cache = 16;

}  // namespace

std::shared_ptr<const std::vector<Vec3>> camera_rays(int width, int height, double dfov_deg) {
    using Key = std::tuple<int, int, long>;
    static std::mutex mutex;
    static std::list<std::pair<Key, std::shared_ptr<const std::vector<Vec3>>>> cache;
    const Key key{width, height, std::lround(dfov_deg * 100.0)};
    {
        const std::scoped_lock lock(mutex);
        for (auto it = cache.begin(); it != cache.end(); ++it) {
            if (it->first == key) {
                cache.splice(cache.begin(), cache, it);
                return cache.front().second;
            }
        }
    }
    const double dfov = static_cast<double>(std::get<2>(key)) / 100.0;
    const double focal = std::hypot(width, height) / 2.0 / std::tan(geo::radians(dfov) / 2.0);
    auto rays = std::make_shared<std::vector<Vec3>>(static_cast<std::size_t>(width) *
                                                    static_cast<std::size_t>(height));
    for (int y = 0; y < height; ++y) {
        const double v = (y + 0.5 - height / 2.0) / focal;
        for (int x = 0; x < width; ++x) {
            const double u = (x + 0.5 - width / 2.0) / focal;
            const double n = std::sqrt(u * u + v * v + 1.0);
            (*rays)[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                    static_cast<std::size_t>(x)] = {
                static_cast<float>(u / n), static_cast<float>(v / n), static_cast<float>(1.0 / n)};
        }
    }
    const std::scoped_lock lock(mutex);
    cache.emplace_front(key, rays);
    if (cache.size() > frame_cache) cache.pop_back();
    return rays;
}

Rotation rotation(double heading_deg, double pitch_deg) {
    const double psi = geo::radians(heading_deg);
    const double theta = geo::radians(pitch_deg);
    const double fx = std::sin(psi) * std::cos(theta);
    const double fy = std::cos(psi) * std::cos(theta);
    const double fz = std::sin(theta);
    const double rx = std::cos(psi);
    const double ry = -std::sin(psi);
    const double rz = 0.0;
    // up = right x forward; the second row is down = -up.
    const double ux = ry * fz - rz * fy;
    const double uy = rz * fx - rx * fz;
    const double uz = rx * fy - ry * fx;
    const auto f = [](double v) { return static_cast<float>(v); };
    return {f(rx), f(ry), f(rz), f(-ux), f(-uy), f(-uz), f(fx), f(fy), f(fz)};
}

double reach(double max_ray_up, double height) {
    if (max_ray_up >= -0.1) return max_range_m;
    // Margins for terrain above the camera base and for the frame edge.
    return std::min(max_range_m, (height + 300.0) / std::tan(-std::asin(max_ray_up)) + 200.0);
}

double pixel_angle(int width, int height, double dfov_deg) {
    return 2.0 * std::tan(geo::radians(dfov_deg) / 2.0) / std::hypot(width, height);
}

std::pair<float, float> window_anchor(const Pose& pose) {
    const raster::LocalFrame frame{.lat0 = units::in_degrees(pose.position.lat),
                                   .lon0 = units::in_degrees(pose.position.lon)};
    const double east = (frame.lon0 - units::in_degrees(pose.home.lon)) * frame.m_per_deg_lon();
    const double north = (frame.lat0 - units::in_degrees(pose.home.lat)) * frame.m_per_deg_lat();
    return {static_cast<float>(east), static_cast<float>(north)};
}

}  // namespace docksim::render
