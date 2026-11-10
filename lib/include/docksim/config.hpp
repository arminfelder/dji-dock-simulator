#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Configuration schema. Every field has a default; a YAML file overrides any subset.
namespace docksim {

// Whole-machine version of the Dock 2 release 14.03.0915. It ships the dock
// main module v66.04.03.00 and the M3D main module v66.04.03.01.
inline constexpr const char* default_firmware = "14.03.0915";

enum class BrokerTransport : std::uint8_t { tcp, websockets };
enum class PhotoRenderer : std::uint8_t { imagery, synthetic };
enum class RenderDevice : std::uint8_t { cpu, gpu, automatic, rt };
enum class FacadeStyle : std::uint8_t { classic, modern };
enum class LiveSource : std::uint8_t { render, pattern };
enum class VideoEncoder : std::uint8_t { software, nvenc };
enum class LogLevel : std::uint8_t { debug, info, warning, error };

struct BrokerConfig {
    std::string host = "tcp.cloud.example.com";
    int port = 8883;
    bool tls = true;
    std::optional<std::string> ca_file;
    // Skip certificate and host name checks.
    bool insecure = false;
    int keepalive = 60;
    BrokerTransport transport = BrokerTransport::tcp;
    std::string ws_path = "/mqtt";
    std::optional<std::string> username;
    std::optional<std::string> password;
    // The real dock uses QoS 1 on every topic except osd, which is QoS 0.
    int qos = 1;

    friend bool operator==(const BrokerConfig&, const BrokerConfig&) = default;
};

// Backend HTTP API, for login and for wayline file download.
struct HttpConfig {
    // The backend serves its API under the /server path.
    std::string base_url = "https://cloud.example.com/server";
    std::string auth_path = "/auth";
    std::string org_path = "/org/current";
    std::optional<std::string> username;
    std::optional<std::string> password;
    bool verify_tls = true;
    std::optional<std::string> ca_file;

    friend bool operator==(const HttpConfig&, const HttpConfig&) = default;
};

// Binding to an organization of the cloud platform. --login fills these in.
struct OrganizationConfig {
    bool bind_on_start = true;
    std::optional<std::string> organization_id;
    std::optional<std::string> binding_code;
    std::string dock_callsign = "Simulated Dock";
    std::string drone_callsign = "Simulated Drone";

    friend bool operator==(const OrganizationConfig&, const OrganizationConfig&) = default;
};

struct DeviceConfig {
    std::string dock_sn = "SIMDOCK0000001";
    std::string drone_sn = "SIMDRONE0000001";
    // dock1, dock2 or dock3.
    std::string dock_model = "dock2";
    // m30, m30t, m3d, m3td, m4d or m4td.
    std::string drone_model = "m3d";
    std::string thing_version = "1.1.0";
    std::string firmware_version = default_firmware;
    std::string dock_firmware_version = default_firmware;
    std::string drone_firmware_version = default_firmware;
    std::string device_secret = "simulator";
    // PEM text of the RSA public key that signs app licenses. Empty: the license is not checked.
    std::optional<std::string> license_key;
    double latitude = 47.263;
    double longitude = 11.401;
    // Metres above the ellipsoid. The web UI matches missions to the dock by it.
    double altitude = 580.0;
    std::string channel_index = "A";

    friend bool operator==(const DeviceConfig&, const DeviceConfig&) = default;
};

// Seconds between messages. topo_interval 0 sends update_topo only at start.
struct TelemetryConfig {
    double dock_osd_interval = 1.0;
    double drone_osd_interval = 0.5;
    double state_interval = 30.0;
    double topo_interval = 0.0;

    friend bool operator==(const TelemetryConfig&, const TelemetryConfig&) = default;
};

struct MediaConfig {
    bool enabled = true;
    // Seconds between landing and the start of the upload. Until then the web
    // UI shows "Waiting to upload", and Force upload starts it at once.
    double upload_delay = 60.0;
    std::pair<int, int> image_size{2000, 1500};
    std::optional<std::string> endpoint_override;
    bool verify_tls = true;
    PhotoRenderer renderer = PhotoRenderer::imagery;
    // XYZ tile templates with {z}, {x} and {y}. The default covers Austria only.
    std::string imagery_url =
        "https://mapsneu.wien.gv.at/basemap/bmaporthofoto30cm/normal/google3857/{z}/{y}/{x}.jpeg";
    int imagery_max_zoom = 19;
    std::string imagery_attribution = "Datenquelle: basemap.at";
    // Terrarium encoding: metres = R * 256 + G + B / 256 - 32768.
    std::string dem_url = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png";
    int dem_zoom = 14;
    // A finer DEM within dem_near_radius metres of the camera. 0 turns it off.
    int dem_near_zoom = 15;
    double dem_near_radius = 1500.0;
    std::string cache_dir = "~/.cache/dock_sim/tiles";
    RenderDevice render_device = RenderDevice::automatic;
    // rt only: accept a software Vulkan device (Mesa lavapipe). Slow; for tests.
    bool rt_allow_cpu = false;
    // Part of the GPU device name, for example "NVIDIA", "Arc", "Radeon".
    std::optional<std::string> gpu_device;
    bool buildings = true;
    std::vector<std::string> overpass_urls = {
        "https://overpass-api.de/api/interpreter",
        // Full planet. Never add a regional instance (overpass.osm.ch): it answers
        // 200 with no elements outside its region, and that empty cell stays cached.
        "https://overpass.openstreetmap.fr/api/interpreter",
        "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
        "https://overpass.private.coffee/api/interpreter",
    };
    // Metres, for a footprint with no height and no building:levels tag.
    double building_default_height = 8.0;
    double building_level_height = 3.0;
    // Facades of residential buildings without a date tag. classic: 19th
    // century city block; modern: post-war apartment block.
    FacadeStyle building_default_style = FacadeStyle::classic;
    // rt only: roofs from the BEV laser scan (Austria, 1 m, CC BY 4.0).
    bool building_scan = true;
    double building_scan_radius = 1500.0;
    // Metres around the camera that show buildings at 0.5 m cells.
    double building_radius = 1000.0;
    // A coarse ring beyond building_radius, out to the skyline. 0 turns it off.
    double building_far_radius = 3000.0;
    double building_far_resolution = 3.0;
    // Height in metres per OSM way or relation id, for buildings that OSM tags wrongly.
    std::map<std::int64_t, double> building_heights;
    bool shadows = true;
    // Rays per pixel side for photos on the GPU.
    int supersample = 2;
    // Sun position. Empty is the real time. "HH:MM" is that time today in
    // lighting_timezone. A full ISO 8601 time sets the date too.
    std::optional<std::string> lighting_time;
    std::string lighting_timezone = "Europe/Vienna";
    // false lets the lighting clock run on from lighting_time. true holds it.
    bool lighting_frozen = false;

    friend bool operator==(const MediaConfig&, const MediaConfig&) = default;
};

struct LiveConfig {
    bool enabled = true;
    int fps = 25;
    // WIDTHxHEIGHT.
    std::string size = "1280x720";
    // For example 6M or 800k. Much below 4M at 1280x720 shows H.264 blocks.
    std::string bitrate = "6M";
    // Saves the raw rendered frame here every 30 s, for quality checks.
    std::optional<std::string> snapshot_dir;
    bool overlay = true;
    // The media server can announce an address that the network does not
    // carry. These two replace the address in the WHIP answer.
    std::optional<std::string> candidate_host;
    std::optional<int> candidate_port;
    LiveSource source = LiveSource::render;
    int render_workers = 8;
    // Terrain is ray-marched on a grid this many times smaller than the frame.
    // Empty picks 4 on the GPU and 16 on the CPU.
    std::optional<int> march_scale;
    // Rays per pixel side on the GPU. The CPU path ignores it.
    int supersample = 2;
    VideoEncoder encoder = VideoEncoder::software;

    [[nodiscard]] std::int64_t bitrate_bps() const;
    [[nodiscard]] std::pair<int, int> frame() const;

    friend bool operator==(const LiveConfig&, const LiveConfig&) = default;
};

// Flight kinematics. The simulator flies a straight line between waypoints.
struct FlightConfig {
    double speed = 10.0;
    double climb_rate = 4.0;
    double takeoff_altitude = 30.0;
    // Seconds per motion step. 0.04 s gives a new pose for every frame of a
    // 25 fps live feed.
    double tick = 0.04;
    double prepare_seconds = 6.0;
    double land_seconds = 8.0;

    friend bool operator==(const FlightConfig&, const FlightConfig&) = default;
};

// One HMS alarm that the dock reports again and again (event hms).
// Launch limits of the dock. DJI specification values, not read from the firmware.
struct WeatherConfig {
    // Metres per second. At this wind speed and more, the dock does not launch.
    double max_wind_takeoff = 12.0;
    // RainfallEnum (1 light, 2 moderate, 3 heavy). At this rainfall and more, the dock does not launch.
    int max_rainfall_takeoff = 3;

    friend bool operator==(const WeatherConfig&, const WeatherConfig&) = default;
};

// Instructor REST API and web UI.
struct ApiConfig {
    bool enabled = true;
    std::string bind = "127.0.0.1";
    // 0 takes any free port.
    int port = 8080;
    // Set: every /api request must send "Authorization: Bearer <token>".
    std::optional<std::string> token;
    // Directory of the built web UI. Empty: <executable>/../share/dock-sim/web.
    std::optional<std::string> web_root;

    friend bool operator==(const ApiConfig&, const ApiConfig&) = default;
};

struct HmsAlarm {
    // DJI HMS code, for example 0x16100083.
    std::string code;
    // 0 notification, 1 reminder, 2 warning.
    int level = 0;
    // 0 flight task, 1 device management, 2 media, 3 hms.
    int module = 3;
    // dock or drone.
    std::string device = "dock";
    bool in_the_sky = false;
    // The alarm goes away by itself when its cause ends, for example strong wind.
    bool imminent = false;
    // Fill the variables of the HMS text (hms.json of DJI).
    int component_index = 0;
    int sensor_index = 0;

    friend bool operator==(const HmsAlarm&, const HmsAlarm&) = default;
};

// Remote debug, firmware, logs, flight areas and HMS. Durations are seconds of simulation time.
struct MaintenanceConfig {
    // From the reply of a remote-debug job (cover, putter, charge, aircraft power, format) to its
    // end.
    double job_duration = 4.0;
    // device_reboot holds the dock silent for this time.
    double reboot_duration = 60.0;
    // From ota_create to the new firmware version.
    double ota_duration = 30.0;
    // Each step of a log upload: pull, compress, upload.
    double log_step_duration = 1.0;
    // A real dock asks for its custom flight areas when it goes online.
    bool sync_flight_areas_on_start = true;
    // Between two flight_areas_drone_location events of a flying aircraft.
    double flight_area_report_interval = 5.0;
    // Between two hms events. 0 sends the list once after start.
    double hms_interval = 30.0;
    std::vector<HmsAlarm> hms;

    friend bool operator==(const MaintenanceConfig&, const MaintenanceConfig&) = default;
};

struct Config {
    BrokerConfig broker;
    HttpConfig http;
    DeviceConfig device;
    OrganizationConfig organization;
    TelemetryConfig telemetry;
    MediaConfig media;
    LiveConfig live;
    FlightConfig flight;
    MaintenanceConfig maintenance;
    WeatherConfig weather;
    ApiConfig api;
    LogLevel log_level = LogLevel::info;

    // Throws ConfigError that names every field out of range.
    void validate() const;

    friend bool operator==(const Config&, const Config&) = default;
};

// The configuration cannot be used. The message names every bad field.
class ConfigError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct LoadResult {
    Config config;
    // Dotted names of keys that the schema does not know. They have no effect.
    std::vector<std::string> unknown_keys;
};

using Environment = std::map<std::string, std::string>;

// Read the YAML file (none: defaults), apply DOCKSIM_* overrides, validate.
// Any other key works as DOCKSIM_<SECTION>__<KEY>, for example DOCKSIM_MEDIA__RENDERER.
[[nodiscard]] LoadResult load_config(const std::optional<std::filesystem::path>& path,
                                     const Environment& env);

// The DOCKSIM_* variables of the process environment.
[[nodiscard]] Environment process_environment();

// Bits per second from 6M, 800k or 2000000. Throws ConfigError.
[[nodiscard]] std::int64_t parse_bitrate(std::string_view text);

// Width and height from WIDTHxHEIGHT, for example 1280x720. Both must be positive.
[[nodiscard]] std::optional<std::pair<int, int>> parse_size(std::string_view text);

}  // namespace docksim
