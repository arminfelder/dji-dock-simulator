#include "services/areas.hpp"

#include "device/geo.hpp"
#include "log/log.hpp"
#include "sim/context.hpp"
#include "util/digest.hpp"
#include "util/numeric.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <format>
#include <stdexcept>

namespace docksim::services {

namespace {

const auto& logger() {
    static const auto log = log::get("dock_sim.areas");
    return log;
}

std::string lower(std::string s) {
    std::ranges::transform(s, s.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

// The files of a flight_areas_get or offline_map_get reply. Empty if the reply has none.
std::optional<Json> reply_files(const Json& reply) {
    if (reply.contains("result") && reply["result"].is_number() && reply["result"] != 0)
        return std::nullopt;
    const Json* holder = &reply;
    if (reply.contains("output") && reply["output"].is_object()) holder = &reply["output"];
    if (!holder->contains("files")) return Json::array();
    const auto& files = (*holder)["files"];
    if (!files.is_array()) return std::nullopt;
    return files;
}

Json sync_file(const Json& file) {
    return {{"name", file.value("name", "")}, {"checksum", file.value("checksum", "")}};
}

// Download and check one listed file. Empty on success, else the sync reason.
struct Download {
    std::vector<std::byte> body;
    std::optional<code::SyncReason> failure;
};

asio::awaitable<Download> download(const Fetch& fetch, const Json& file, bool checksum_reason) {
    const auto url = file.value("url", "");
    const auto name = file.value("name", "");
    Download out;
    if (url.empty()) {
        logger()->error("{}: no url", name);
        out.failure = code::SyncReason::download_failed;
        co_return out;
    }
    std::string error;
    try {
        out.body = co_await fetch(url);
    } catch (const std::exception& e) {
        error = e.what();
    }
    if (!error.empty()) {
        logger()->error("download of {} failed: {}", name, error);
        out.failure = code::SyncReason::download_failed;
        co_return out;
    }
    const auto size = file.value("size", Json());
    if (size.is_number_integer() && size.get<std::int64_t>() > 0 &&
        static_cast<std::size_t>(size.get<std::int64_t>()) != out.body.size()) {
        logger()->error("{}: {} bytes, the list says {}", name, out.body.size(), size.dump());
        out.failure = code::SyncReason::download_failed;
        co_return out;
    }
    const auto expected = lower(file.value("checksum", ""));
    if (!expected.empty()) {
        const auto actual = digest::sha256_hex(out.body);
        if (actual != expected) {
            logger()->error("{}: sha256 {} is not the listed {}", name, actual, expected);
            out.failure = checksum_reason ? code::SyncReason::checksum_failed
                                          : code::SyncReason::download_failed;
        }
    }
    co_return out;
}

struct Plane {
    double x;
    double y;
};

// Metres east and north of the origin, on a plane that touches the earth there.
Plane to_plane(double lat0, double lon0, double latitude, double longitude) {
    return {
        .x = geo::radians(longitude - lon0) * std::cos(geo::radians(lat0)) * geo::earth_radius_m,
        .y = geo::radians(latitude - lat0) * geo::earth_radius_m,
    };
}

double segment_distance(Plane a, Plane b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    const double t = length2 > 0.0 ? std::clamp(-(a.x * dx + a.y * dy) / length2, 0.0, 1.0) : 0.0;
    return std::hypot(a.x + t * dx, a.y + t * dy);
}

double centre_distance(const areas::Area& area, double latitude, double longitude) {
    return units::in_metres(
        geo::haversine(geo::point(area.latitude, area.longitude), geo::point(latitude, longitude)));
}

areas::Area parse_feature(const Json& feature, std::size_t index) {
    if (!feature.is_object()) throw std::runtime_error("a feature must be an object");
    const Json properties = feature.value("properties", Json::object());
    if (!properties.is_object()) throw std::runtime_error("properties must be an object");
    areas::Area area;
    area.id = feature.contains("id") && feature["id"].is_string() ? feature["id"].get<std::string>()
                                                                  : std::format("area-{}", index);
    // The backend puts the type next to geometry; older files carry it in properties.
    auto kind = feature.value("geofence_type", Json());
    if (!kind.is_string()) kind = properties.value("type", Json("nfz"));
    if (kind == "nfz") {
        area.kind = areas::Kind::nfz;
    } else if (kind == "dfence") {
        area.kind = areas::Kind::dfence;
    } else {
        throw std::runtime_error(
            std::format("{}: type {} is not nfz or dfence", area.id, kind.dump()));
    }
    area.enable = properties.value("enable", true);
    const Json geometry = feature.value("geometry", Json::object());
    const auto type = geometry.value("type", "");
    const Json& coordinates = geometry.value("coordinates", Json());
    if (type == "Point") {
        if (!coordinates.is_array() || coordinates.size() < 2 || !coordinates[0].is_number() ||
            !coordinates[1].is_number())
            throw std::runtime_error(
                std::format("{}: a Point needs [longitude, latitude]", area.id));
        if (properties.value("subType", "") != "Circle")
            throw std::runtime_error(std::format("{}: a Point needs subType Circle", area.id));
        area.circle = true;
        area.longitude = coordinates[0].get<double>();
        area.latitude = coordinates[1].get<double>();
        const auto radius = properties.value("radius", Json());
        if (!radius.is_number() || radius.get<double>() <= 0.0)
            throw std::runtime_error(std::format("{}: a Circle needs a positive radius", area.id));
        area.radius = radius.get<double>();
    } else if (type == "Polygon") {
        if (!coordinates.is_array() || coordinates.empty() || !coordinates[0].is_array())
            throw std::runtime_error(std::format("{}: a Polygon needs a ring", area.id));
        for (const auto& position : coordinates[0]) {
            if (!position.is_array() || position.size() < 2 || !position[0].is_number() ||
                !position[1].is_number())
                throw std::runtime_error(std::format("{}: bad ring position", area.id));
            area.ring.emplace_back(position[1].get<double>(), position[0].get<double>());
        }
        if (area.ring.size() > 1 && area.ring.front() == area.ring.back()) area.ring.pop_back();
        if (area.ring.size() < 3)
            throw std::runtime_error(std::format("{}: a ring needs three positions", area.id));
    } else {
        throw std::runtime_error(std::format("{}: geometry {} is not Point or Polygon", area.id,
                                             type.empty() ? "<none>" : type));
    }
    return area;
}

}  // namespace

namespace areas {

std::vector<Area> parse(std::string_view geojson) {
    Json root;
    try {
        root = Json::parse(geojson);
    } catch (const Json::exception& e) {
        throw std::runtime_error(std::format("not JSON: {}", e.what()));
    }
    if (!root.is_object() || !root.contains("features") || !root["features"].is_array())
        throw std::runtime_error("no features list");
    std::vector<Area> out;
    for (std::size_t i = 0; i < root["features"].size(); ++i)
        out.push_back(parse_feature(root["features"][i], i));
    return out;
}

bool contains(const Area& area, double latitude, double longitude) {
    if (area.circle) return centre_distance(area, latitude, longitude) <= area.radius;
    bool inside = false;
    const auto n = area.ring.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const auto a = to_plane(latitude, longitude, area.ring[i].first, area.ring[i].second);
        const auto b = to_plane(latitude, longitude, area.ring[j].first, area.ring[j].second);
        if ((a.y > 0.0) != (b.y > 0.0) && 0.0 < (b.x - a.x) * (0.0 - a.y) / (b.y - a.y) + a.x)
            inside = !inside;
    }
    return inside;
}

double edge_distance(const Area& area, double latitude, double longitude) {
    if (area.circle) return std::abs(centre_distance(area, latitude, longitude) - area.radius);
    double best = std::numeric_limits<double>::infinity();
    const auto n = area.ring.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        best = std::min(
            best, segment_distance(
                      to_plane(latitude, longitude, area.ring[i].first, area.ring[i].second),
                      to_plane(latitude, longitude, area.ring[j].first, area.ring[j].second)));
    }
    return best;
}

}  // namespace areas

void FlightAreas::register_services(Registry& registry) {
    registry.add("flight_areas_update", [this](const Json&) -> asio::awaitable<Reply> {
        if (!sync_.done()) co_return result(code::Result::flight_areas_busy);
        start_sync();
        co_return ok();
    });
    // Not in the DJI Dock 2 thing model. The cloud-sdk FlightAreaMethodEnum has it; the
    // simulator drops every stored area.
    registry.add("flight_areas_delete", [this](const Json&) -> asio::awaitable<Reply> {
        areas_.clear();
        violated_ = false;
        co_return ok();
    });
}

void FlightAreas::start_sync() {
    if (sync_.done()) sync_ = ctx_.spawn(sync(), "flight-areas-sync");
}

asio::awaitable<void> FlightAreas::progress(std::string_view status, code::SyncReason reason,
                                            Json file) {
    // status values are the lowercase JsonValue names of the cloud enum.
    co_await ctx_.emit_event(
        "flight_areas_sync_progress",
        {{"result", 0}, {"status", status}, {"reason", reason}, {"file", std::move(file)}});
}

asio::awaitable<void> FlightAreas::sync() {
    Json reply;
    try {
        reply = co_await ctx_.request("flight_areas_get", Json::object());
    } catch (const transport::RequestTimeout& e) {
        logger()->error("flight_areas_get: {}", e.what());
        co_return;
    }
    const auto files = reply_files(reply);
    if (!files) {
        logger()->error("flight_areas_get: unusable reply {}", reply.dump());
        co_await progress("fail", code::SyncReason::parse_failed, Json::object());
        co_return;
    }
    logger()->info("flight areas: {} file(s)", files->size());
    if (files->empty()) {
        areas_.clear();
        violated_ = false;
        co_await progress("synchronizing", code::SyncReason::success, Json::object());
        co_await progress("synchronized", code::SyncReason::success, Json::object());
        co_return;
    }
    std::vector<areas::Area> loaded;
    Json last;
    for (const auto& file : *files) {
        last = sync_file(file);
        co_await progress("synchronizing", code::SyncReason::success, last);
        auto got = co_await download(fetch_, file, true);
        if (got.failure) {
            co_await progress("fail", *got.failure, last);
            co_return;
        }
        std::string error;
        try {
            auto parsed = areas::parse(
                std::string_view(reinterpret_cast<const char*>(got.body.data()), got.body.size()));
            std::ranges::move(parsed, std::back_inserter(loaded));
        } catch (const std::exception& e) {
            error = e.what();
        }
        if (!error.empty()) {
            logger()->error("{}: {}", file.value("name", ""), error);
            co_await progress("fail", code::SyncReason::load_failed, last);
            co_return;
        }
    }
    areas_ = std::move(loaded);
    violated_ = false;
    logger()->info("flight areas: {} area(s) stored", areas_.size());
    co_await progress("synchronized", code::SyncReason::success, last);
}

bool FlightAreas::forbidden(double latitude, double longitude) const {
    return std::ranges::any_of(areas_, [&](const areas::Area& a) {
        return a.enable && a.kind == areas::Kind::nfz && areas::contains(a, latitude, longitude);
    });
}

bool FlightAreas::violated(double latitude, double longitude) const {
    bool fenced = false;
    bool inside_fence = false;
    for (const auto& a : areas_) {
        if (!a.enable || a.kind != areas::Kind::dfence) continue;
        fenced = true;
        inside_fence = inside_fence || areas::contains(a, latitude, longitude);
    }
    return forbidden(latitude, longitude) || (fenced && !inside_fence);
}

bool FlightAreas::observe() {
    if (areas_.empty()) return false;
    const auto& drone = ctx_.drone;
    const double now = ctx_.clock().monotonic();
    if (now - last_report_ >= ctx_.cfg().maintenance.flight_area_report_interval) {
        last_report_ = now;
        Json locations = Json::array();
        for (const auto& a : areas_) {
            if (!a.enable) continue;
            locations.push_back({
                {"area_id", a.id},
                {"area_distance",
                 num::round_to(areas::edge_distance(a, drone.latitude, drone.longitude), 2)},
                {"is_in_area", areas::contains(a, drone.latitude, drone.longitude)},
            });
        }
        if (!locations.empty()) {
            ctx_.spawn(
                ctx_.emit_event("flight_areas_drone_location",
                                {{"drone_locations", std::move(locations)}}, {.need_reply = false}),
                "flight-areas-location");
        }
    }
    const bool now_violated = violated(drone.latitude, drone.longitude);
    const bool entered = now_violated && !violated_;
    violated_ = now_violated;
    return entered;
}

void OfflineMap::register_services(Registry& registry) {
    registry.add("offline_map_update", [this](const Json&) -> asio::awaitable<Reply> {
        if (!sync_.done()) co_return result(code::Result::offline_map_busy);
        sync_ = ctx_.spawn(sync(), "offline-map-sync");
        co_return ok();
    });
}

asio::awaitable<void> OfflineMap::progress(std::string_view status, code::SyncReason reason,
                                           Json file) {
    co_await ctx_.emit_event(
        "offline_map_sync_progress",
        {{"result", 0}, {"status", status}, {"reason", reason}, {"file", std::move(file)}});
}

asio::awaitable<void> OfflineMap::sync() {
    Json reply;
    try {
        reply = co_await ctx_.request("offline_map_get", Json::object());
    } catch (const transport::RequestTimeout& e) {
        logger()->error("offline_map_get: {}", e.what());
        co_return;
    }
    const auto files = reply_files(reply);
    if (!files) {
        logger()->error("offline_map_get: unusable reply {}", reply.dump());
        co_await progress("fail", code::SyncReason::parse_failed, Json::object());
        co_return;
    }
    const Json& output =
        reply.contains("output") && reply["output"].is_object() ? reply["output"] : reply;
    enabled_ = output.value("offline_map_enable", false);
    logger()->info("offline map: enable={} {} file(s)", enabled_, files->size());
    if (files->empty()) {
        files_.clear();
        co_await progress("synchronizing", code::SyncReason::success, Json::object());
        co_await progress("synchronized", code::SyncReason::success, Json::object());
        co_return;
    }
    std::vector<std::string> names;
    Json last;
    for (const auto& file : *files) {
        last = sync_file(file);
        co_await progress("synchronizing", code::SyncReason::success, last);
        // OfflineMapSyncReasonEnum has no checksum reason, so a bad checksum is a bad download.
        const auto got = co_await download(fetch_, file, false);
        if (got.failure) {
            co_await progress("fail", *got.failure, last);
            co_return;
        }
        names.push_back(file.value("name", ""));
    }
    files_ = std::move(names);
    co_await progress("synchronized", code::SyncReason::success, last);
}

}  // namespace docksim::services
