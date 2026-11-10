#pragma once

#include "protocol/codes.hpp"
#include "services/registry.hpp"
#include "sim/async.hpp"
#include "util/json.hpp"

#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace docksim::sim {
class DockContext;
}

// Custom flight areas (DJI 130.custom-flight-area.md) and offline maps. Both
// ask the cloud for a file list, download each file, check its SHA256 and
// report the sync as an event. The wire rules are in docs/protocol-quirks.md
// under Maintenance.
namespace docksim::services {

// The body of a file URL. Throws on any download error.
using Fetch = std::function<asio::awaitable<std::vector<std::byte>>(std::string url)>;

namespace areas {

enum class Kind : std::uint8_t { nfz, dfence };

struct Area {
    std::string id;
    Kind kind = Kind::nfz;
    bool enable = true;
    // A Point with subType Circle: centre and radius in metres.
    bool circle = false;
    double latitude = 0.0;
    double longitude = 0.0;
    double radius = 0.0;
    // A Polygon: the outer ring as (latitude, longitude).
    std::vector<std::pair<double, double>> ring;
};

// The features of a custom flight area file. Throws std::runtime_error.
[[nodiscard]] std::vector<Area> parse(std::string_view geojson);

[[nodiscard]] bool contains(const Area& area, double latitude, double longitude);
// Metres from the point to the edge of the area, inside or outside.
[[nodiscard]] double edge_distance(const Area& area, double latitude, double longitude);

}  // namespace areas

class FlightAreas {
public:
    FlightAreas(sim::DockContext& ctx, Fetch fetch) : ctx_(ctx), fetch_(std::move(fetch)) {}

    // flight_areas_update and flight_areas_delete.
    void register_services(Registry& registry);

    // flight_areas_get, then download, check and store every file.
    asio::awaitable<void> sync();
    void start_sync();

    [[nodiscard]] const std::vector<areas::Area>& areas() const { return areas_; }
    // Inside an enabled no-fly zone.
    [[nodiscard]] bool forbidden(double latitude, double longitude) const;
    // Inside an enabled no-fly zone, or outside every enabled geofence.
    [[nodiscard]] bool violated(double latitude, double longitude) const;

    // Call on each motion step of a flight. Sends flight_areas_drone_location at
    // its interval. True when the aircraft has just entered a no-fly zone or left a geofence.
    bool observe();

private:
    asio::awaitable<void> progress(std::string_view status, code::SyncReason reason, Json file);

    sim::DockContext& ctx_;
    Fetch fetch_;
    std::vector<areas::Area> areas_;
    sim::TaskGroup::Handle sync_;
    double last_report_ = -std::numeric_limits<double>::infinity();
    bool violated_ = false;
};

// offline_map_update, then offline_map_get, download and offline_map_sync_progress.
// The simulator stores the file names only; it does not use the map.
class OfflineMap {
public:
    OfflineMap(sim::DockContext& ctx, Fetch fetch) : ctx_(ctx), fetch_(std::move(fetch)) {}

    void register_services(Registry& registry);
    asio::awaitable<void> sync();

    [[nodiscard]] bool enabled() const { return enabled_; }
    [[nodiscard]] const std::vector<std::string>& files() const { return files_; }

private:
    asio::awaitable<void> progress(std::string_view status, code::SyncReason reason, Json file);

    sim::DockContext& ctx_;
    Fetch fetch_;
    bool enabled_ = false;
    std::vector<std::string> files_;
    sim::TaskGroup::Handle sync_;
};

}  // namespace docksim::services
