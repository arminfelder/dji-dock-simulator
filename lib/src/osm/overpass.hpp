#pragma once

#include "util/json.hpp"

#include <chrono>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <string_view>
#include <vector>

// Overpass API client: one query per map cell, mirrors, rate limit and
// retries. Public servers refuse a query whose declared timeout and maxsize do
// not fit their free capacity, and answer 504. A small declared maxsize is
// admitted where the default 512 MiB is refused. Rapid requests get 429.
namespace docksim::osm {

inline constexpr std::string_view overpass_user_agent = "dock-sim/0.1 (+building renderer)";
// One cell is about 0.5 MB, so 64 MiB leaves a wide margin. building:part gives
// stepped towers near the camera.
inline constexpr std::string_view overpass_query =
    "[out:json][timeout:25][maxsize:67108864];("
    "way[\"building\"]({s},{w},{n},{e});relation[\"building\"]({s},{w},{n},{e});"
    "way[\"building:part\"]({s},{w},{n},{e});relation[\"building:part\"]({s},{w},{n},{e});"
    ");out geom tags;";
// Seconds between two requests, so that the per-client rate limit is not hit.
inline constexpr double request_gap_s = 1.0;
inline constexpr int attempts = 3;
inline constexpr double retry_after_default_s = 10.0;
inline constexpr double busy_backoff_s = 5.0;
// A mirror that failed every attempt is skipped this long, unless all are down.
inline constexpr double mirror_cooldown_s = 120.0;

// The query with the box filled in, 4 decimals as the Python client wrote it.
[[nodiscard]] std::string format_query(std::string_view query, double south, double west,
                                       double north, double east);
[[nodiscard]] std::string format_query(double south, double west, double north, double east);
// overpass_query and the features_query union in one request: the buildings
// first, then the entrance and shop nodes.
[[nodiscard]] std::string area_query();

// Seconds from a Retry-After header: delta seconds or an HTTP date.
[[nodiscard]] double retry_after(
    std::optional<std::string_view> value,
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now());

struct PostReply {
    long status = 0;
    std::string body;
    // Header names in lower case.
    std::map<std::string, std::string> headers;
};

// The caller requested a stop while a fetch waited or ran.
struct Stopped {};

// POST of the form field data=<query> to url. Empty on a transport error;
// then problem names it. Throws Stopped when stop is requested.
using Post =
    std::function<std::optional<PostReply>(const std::string& url, const std::string& query,
                                           std::string& problem, const std::stop_token& stop)>;

// The default transport: libcurl, 60 s timeout, the dock-sim User-Agent.
[[nodiscard]] Post curl_post();

// Buildings and entrances/shops of one box, in cache form. Empty features: no
// server delivered them, but the buildings are valid.
struct Area {
    std::vector<Json> buildings;
    std::optional<std::vector<Json>> features;
};

// The source of raw building elements for one box, in cache form. Empty if no server answers.
// Throws Stopped when stop is requested.
class BuildingSource {
public:
    BuildingSource() = default;
    BuildingSource(const BuildingSource&) = delete;
    BuildingSource& operator=(const BuildingSource&) = delete;
    BuildingSource(BuildingSource&&) = delete;
    BuildingSource& operator=(BuildingSource&&) = delete;
    virtual ~BuildingSource() = default;

    [[nodiscard]] virtual std::optional<std::vector<Json>> buildings(
        double south, double west, double north, double east, const std::stop_token& stop) = 0;
    // Entrances and shops (osm/features.hpp) in cache form. A source without them has none.
    [[nodiscard]] virtual std::optional<std::vector<Json>> features(
        double /*south*/, double /*west*/, double /*north*/, double /*east*/,
        const std::stop_token& /*stop*/) {
        return std::vector<Json>{};
    }
    // Both kinds for one box. The default asks buildings, then features.
    [[nodiscard]] virtual std::optional<Area> area(double south, double west, double north,
                                                   double east, const std::stop_token& stop) {
        auto found = buildings(south, west, north, east, stop);
        if (!found) return std::nullopt;
        return Area{.buildings = std::move(*found),
                    .features = features(south, west, north, east, stop)};
    }
};

class OverpassClient final : public BuildingSource {
public:
    using Sleep = std::function<void(double seconds, const std::stop_token& stop)>;
    using Clock = std::function<double()>;

    // Empty sleep and clock use the real ones. Thread-safe.
    OverpassClient(std::vector<std::string> urls, Post post = curl_post(), Sleep sleep = {},
                   Clock clock = {});

    [[nodiscard]] std::optional<std::vector<Json>> buildings(double south, double west,
                                                             double north, double east,
                                                             const std::stop_token& stop) override;
    [[nodiscard]] std::optional<std::vector<Json>> features(double south, double west, double north,
                                                            double east,
                                                            const std::stop_token& stop) override;
    // One request for both kinds. Empty if no server answers.
    [[nodiscard]] std::optional<Area> area(double south, double west, double north, double east,
                                           const std::stop_token& stop) override;

private:
    // The elements of a query, tried on every mirror. Empty if none answers.
    [[nodiscard]] std::optional<Json> fetch(const std::string& query, double south, double west,
                                            const std::stop_token& stop);
    // Mirrors not cooling down, the last one that answered first, then in configured
    // order; all of them if every one is cooling down.
    [[nodiscard]] std::vector<std::string> mirrors();
    std::optional<PostReply> post(const std::string& url, const std::string& query,
                                  std::string& problem, const std::stop_token& stop);

    std::vector<std::string> urls_;
    Post post_;
    Sleep sleep_;
    Clock clock_;
    std::mutex gap_mutex_;
    std::mutex state_mutex_;
    double last_request_ = -request_gap_s;
    std::map<std::string, double> down_until_;
    std::string preferred_;
};

}  // namespace docksim::osm
