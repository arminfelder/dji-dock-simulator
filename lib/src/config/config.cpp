#include "docksim/config.hpp"

#include "cloud/license.hpp"
#include "config/yaml.hpp"
#include "util/json.hpp"
#include "util/numeric.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <format>
#include <fstream>
#include <functional>
#include <regex>
#include <sstream>
#include <string_view>
#include <tuple>
#include <type_traits>

namespace docksim {

namespace {

constexpr std::string_view env_prefix = "DOCKSIM_";

struct Alias {
    std::string_view env;
    std::string_view section;
    std::string_view key;
};

// Names documented before the rewrite.
constexpr std::array env_aliases{
    Alias{.env = "DOCKSIM_BROKER_HOST", .section = "broker", .key = "host"},
    Alias{.env = "DOCKSIM_BROKER_PORT", .section = "broker", .key = "port"},
    Alias{.env = "DOCKSIM_BROKER_CA", .section = "broker", .key = "ca_file"},
    Alias{.env = "DOCKSIM_MQTT_USERNAME", .section = "broker", .key = "username"},
    Alias{.env = "DOCKSIM_MQTT_PASSWORD", .section = "broker", .key = "password"},
    Alias{.env = "DOCKSIM_BASE_URL", .section = "http", .key = "base_url"},
    Alias{.env = "DOCKSIM_HTTP_USERNAME", .section = "http", .key = "username"},
    Alias{.env = "DOCKSIM_HTTP_PASSWORD", .section = "http", .key = "password"},
    Alias{.env = "DOCKSIM_ORG_ID", .section = "organization", .key = "organization_id"},
    Alias{.env = "DOCKSIM_BINDING_CODE", .section = "organization", .key = "binding_code"},
    Alias{.env = "DOCKSIM_DOCK_SN", .section = "device", .key = "dock_sn"},
    Alias{.env = "DOCKSIM_DRONE_SN", .section = "device", .key = "drone_sn"},
    Alias{.env = "DOCKSIM_API_TOKEN", .section = "api", .key = "token"},
    Alias{.env = "DOCKSIM_LOG_LEVEL", .section = "", .key = "log_level"},
};

// Keys of the Python version that the C++ version does not need. They are
// accepted without a warning, so that old files load unchanged.
constexpr std::array retired_keys{
    std::string_view{"media.rt_build_processes"},
    std::string_view{"media.photo_time"},
    std::string_view{"live.ffmpeg"},
    std::string_view{"live.pattern"},
    std::string_view{"live.font"},
};

template <class E>
struct EnumName {
    std::string_view name;
    E value;
};

template <class E>
struct EnumNames;

template <>
struct EnumNames<BrokerTransport> {
    static constexpr std::array<EnumName<BrokerTransport>, 2> names{
        {{.name = "tcp", .value = BrokerTransport::tcp},
         {.name = "websockets", .value = BrokerTransport::websockets}}};
};
template <>
struct EnumNames<PhotoRenderer> {
    static constexpr std::array<EnumName<PhotoRenderer>, 2> names{
        {{.name = "imagery", .value = PhotoRenderer::imagery},
         {.name = "synthetic", .value = PhotoRenderer::synthetic}}};
};
template <>
struct EnumNames<RenderDevice> {
    // cuda is the name from before the OpenCL renderer.
    static constexpr std::array<EnumName<RenderDevice>, 5> names{
        {{.name = "cpu", .value = RenderDevice::cpu},
         {.name = "gpu", .value = RenderDevice::gpu},
         {.name = "auto", .value = RenderDevice::automatic},
         {.name = "rt", .value = RenderDevice::rt},
         {.name = "cuda", .value = RenderDevice::gpu}}};
};
template <>
struct EnumNames<FacadeStyle> {
    static constexpr std::array<EnumName<FacadeStyle>, 2> names{
        {{.name = "classic", .value = FacadeStyle::classic},
         {.name = "modern", .value = FacadeStyle::modern}}};
};
template <>
struct EnumNames<LiveSource> {
    static constexpr std::array<EnumName<LiveSource>, 2> names{
        {{.name = "render", .value = LiveSource::render},
         {.name = "pattern", .value = LiveSource::pattern}}};
};
template <>
struct EnumNames<VideoEncoder> {
    // libx264 and h264_nvenc are the names from the ffmpeg-based version.
    static constexpr std::array<EnumName<VideoEncoder>, 4> names{
        {{.name = "software", .value = VideoEncoder::software},
         {.name = "nvenc", .value = VideoEncoder::nvenc},
         {.name = "libx264", .value = VideoEncoder::software},
         {.name = "h264_nvenc", .value = VideoEncoder::nvenc}}};
};
template <>
struct EnumNames<LogLevel> {
    static constexpr std::array<EnumName<LogLevel>, 4> names{
        {{.name = "DEBUG", .value = LogLevel::debug},
         {.name = "INFO", .value = LogLevel::info},
         {.name = "WARNING", .value = LogLevel::warning},
         {.name = "ERROR", .value = LogLevel::error}}};
};

template <class E>
concept NamedEnum = requires { EnumNames<E>::names; };

template <class T>
struct is_optional : std::false_type {};
template <class T>
struct is_optional<std::optional<T>> : std::true_type {};

struct BadValue {
    std::string message;
};

std::string upper(std::string s) {
    std::ranges::transform(s, s.begin(),
                           [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}

std::string lower(std::string s) {
    std::ranges::transform(s, s.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::string shown(const Json& v) {
    return v.dump();
}

std::optional<std::int64_t> as_int(const Json& v) {
    if (v.is_number_integer()) return v.get<std::int64_t>();
    if (v.is_number_float()) {
        const double d = v.get<double>();
        if (std::isfinite(d) && std::trunc(d) == d) return static_cast<std::int64_t>(d);
        return std::nullopt;
    }
    if (v.is_string()) {
        std::string s = v.get<std::string>();
        std::erase_if(s, [](unsigned char c) { return std::isspace(c) != 0; });
        std::int64_t out = 0;
        const char* first = s.data() + (!s.empty() && s.front() == '+' ? 1 : 0);
        const auto [ptr, ec] = std::from_chars(first, s.data() + s.size(), out);
        if (ec == std::errc{} && ptr == s.data() + s.size() && !s.empty()) return out;
    }
    return std::nullopt;
}

std::string convert_text(const Json& v) {
    if (v.is_string()) return v.get<std::string>();
    if (v.is_number()) return v.dump();
    throw BadValue{"must be text"};
}

bool convert_flag(const Json& v) {
    if (v.is_boolean()) return v.get<bool>();
    if (v.is_number_integer() && (v == 0 || v == 1)) return v == 1;
    if (v.is_string()) {
        const auto s = lower(v.get<std::string>());
        if (s == "true" || s == "1" || s == "yes" || s == "on" || s == "t" || s == "y") return true;
        if (s == "false" || s == "0" || s == "no" || s == "off" || s == "f" || s == "n")
            return false;
    }
    throw BadValue{"must be true or false"};
}

int convert_whole(const Json& v) {
    const auto i = as_int(v);
    if (!i || *i < std::numeric_limits<int>::min() || *i > std::numeric_limits<int>::max()) {
        throw BadValue{"must be a whole number"};
    }
    return static_cast<int>(*i);
}

double convert_number(const Json& v) {
    if (v.is_number()) return v.get<double>();
    if (v.is_string()) {
        if (auto d = num::parse_double(v.get<std::string>())) return *d;
    }
    throw BadValue{"must be a number"};
}

std::vector<std::string> convert_text_list(const Json& v) {
    if (!v.is_array()) throw BadValue{"must be a list"};
    std::vector<std::string> out;
    for (const auto& item : v) out.push_back(convert_text(item));
    return out;
}

std::pair<int, int> convert_int_pair(const Json& v) {
    if (!v.is_array() || v.size() != 2) throw BadValue{"must be a list of two numbers"};
    return {convert_whole(v[0]), convert_whole(v[1])};
}

std::map<std::int64_t, double> convert_id_metres(const Json& v) {
    if (!v.is_object()) throw BadValue{"must be a mapping of OSM id to metres"};
    std::map<std::int64_t, double> out;
    for (const auto& [key, value] : v.items()) {
        const auto id = as_int(Json(key));
        if (!id) throw BadValue{std::format("key {} must be a whole number", key)};
        out[*id] = convert_number(value);
    }
    return out;
}

std::vector<HmsAlarm> convert_hms(const Json& raw) {
    // From the environment the list comes as JSON text.
    Json v = raw;
    if (v.is_string()) {
        try {
            v = Json::parse(v.get<std::string>());
        } catch (const Json::exception&) {
            throw BadValue{"must be a list of alarms (JSON text in the environment)"};
        }
    }
    if (!v.is_array()) throw BadValue{"must be a list of alarms"};
    std::vector<HmsAlarm> out;
    for (const auto& item : v) {
        if (!item.is_object()) throw BadValue{"each alarm must be a mapping"};
        HmsAlarm alarm;
        for (const auto& [key, value] : item.items()) {
            if (key == "code") {
                alarm.code = convert_text(value);
            } else if (key == "level") {
                alarm.level = convert_whole(value);
            } else if (key == "module") {
                alarm.module = convert_whole(value);
            } else if (key == "device") {
                alarm.device = convert_text(value);
            } else if (key == "in_the_sky") {
                alarm.in_the_sky = convert_flag(value);
            } else if (key == "imminent") {
                alarm.imminent = convert_flag(value);
            } else if (key == "component_index") {
                alarm.component_index = convert_whole(value);
            } else if (key == "sensor_index") {
                alarm.sensor_index = convert_whole(value);
            } else {
                throw BadValue{std::format("alarm key {} is not known", key)};
            }
        }
        out.push_back(std::move(alarm));
    }
    return out;
}

template <NamedEnum E>
E convert_enum(const Json& v) {
    if (v.is_string()) {
        auto s = v.get<std::string>();
        if constexpr (std::is_same_v<E, LogLevel>) s = upper(s);
        for (const auto& [name, value] : EnumNames<E>::names) {
            if (name == s) return value;
        }
    }
    std::string allowed;
    for (const auto& [name, _] : EnumNames<E>::names) {
        allowed += allowed.empty() ? std::string(name) : std::format(", {}", name);
    }
    throw BadValue{std::format("must be one of {}", allowed)};
}

// Values come from YAML, where a bare id such as 100 is a number, and from the
// environment, where every value is text. Both convert to the field type.
template <class T>
T convert(const Json& v) {
    if constexpr (std::is_same_v<T, std::string>) {
        return convert_text(v);
    } else if constexpr (std::is_same_v<T, bool>) {
        return convert_flag(v);
    } else if constexpr (std::is_same_v<T, int>) {
        return convert_whole(v);
    } else if constexpr (std::is_same_v<T, double>) {
        return convert_number(v);
    } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
        return convert_text_list(v);
    } else if constexpr (std::is_same_v<T, std::pair<int, int>>) {
        return convert_int_pair(v);
    } else if constexpr (std::is_same_v<T, std::map<std::int64_t, double>>) {
        return convert_id_metres(v);
    } else if constexpr (std::is_same_v<T, std::vector<HmsAlarm>>) {
        return convert_hms(v);
    } else if constexpr (is_optional<T>::value) {
        if (v.is_null()) return std::nullopt;
        return convert<typename T::value_type>(v);
    } else if constexpr (NamedEnum<T>) {
        return convert_enum<T>(v);
    } else {
        static_assert(false, "no conversion for this field type");
    }
}

template <class T>
using Check = std::function<std::optional<std::string>(const T&)>;

template <class T>
Check<T> range(T lo, T hi) {
    return [=](const T& v) -> std::optional<std::string> {
        if (v < lo || v > hi) return std::format("must be between {} and {}", lo, hi);
        return std::nullopt;
    };
}
template <class T>
Check<T> above(T lo) {
    return [=](const T& v) -> std::optional<std::string> {
        if (!(v > lo)) return std::format("must be greater than {}", lo);
        return std::nullopt;
    };
}
template <class T>
Check<T> at_least(T lo) {
    return [=](const T& v) -> std::optional<std::string> {
        if (v < lo) return std::format("must be at least {}", lo);
        return std::nullopt;
    };
}
template <class T>
Check<std::optional<T>> optional_check(const Check<T>& inner) {
    return [=](const std::optional<T>& v) -> std::optional<std::string> {
        return v ? inner(*v) : std::nullopt;
    };
}
Check<std::string> drone_model() {
    return [](const std::string& v) -> std::optional<std::string> {
        static constexpr std::array models{"m30", "m30t", "m3d", "m3td", "m4d", "m4td"};
        if (std::ranges::find(models, v) == models.end()) {
            return "must be one of m30, m30t, m3d, m3td, m4d, m4td";
        }
        return std::nullopt;
    };
}
Check<std::string> public_key() {
    return [](const std::string& v) -> std::optional<std::string> {
        if (const auto error = license::key_error(v)) {
            return std::format("must be an RSA public key in PEM form ({})", *error);
        }
        return std::nullopt;
    };
}
Check<std::string> dock_model() {
    return [](const std::string& v) -> std::optional<std::string> {
        if (v != "dock1" && v != "dock2" && v != "dock3")
            return "must be one of dock1, dock2, dock3";
        return std::nullopt;
    };
}
Check<std::string> frame_size() {
    return [](const std::string& v) -> std::optional<std::string> {
        if (!parse_size(v)) return "must be WIDTHxHEIGHT, for example 1280x720";
        return std::nullopt;
    };
}
Check<std::string> bitrate() {
    return [](const std::string& v) -> std::optional<std::string> {
        try {
            std::ignore = parse_bitrate(v);
        } catch (const ConfigError& e) {
            return e.what();
        }
        return std::nullopt;
    };
}
Check<std::vector<HmsAlarm>> hms_alarms() {
    return [](const std::vector<HmsAlarm>& list) -> std::optional<std::string> {
        for (std::size_t i = 0; i < list.size(); ++i) {
            const auto& a = list[i];
            if (a.code.empty()) return std::format("alarm {}: code must not be empty", i);
            if (a.level < 0 || a.level > 2)
                return std::format("alarm {}: level must be 0, 1 or 2", i);
            if (a.module < 0 || a.module > 3)
                return std::format("alarm {}: module must be between 0 and 3", i);
            if (a.device != "dock" && a.device != "drone")
                return std::format("alarm {}: device must be dock or drone", i);
            if (a.component_index < 0 || a.sensor_index < 0)
                return std::format("alarm {}: component_index and sensor_index must be 0 or more",
                                   i);
        }
        return std::nullopt;
    };
}
Check<int> qos_level() {
    return [](const int& v) -> std::optional<std::string> {
        if (v < 0 || v > 2) return "must be 0, 1 or 2";
        return std::nullopt;
    };
}

// Walks one section. With a JSON node it reads values into the fields; without
// one it only checks the current values. Both collect every problem.
class Binder {
public:
    Binder(const Json* node, std::string prefix, std::vector<std::string>& problems)
        : node_(node), prefix_(std::move(prefix)), problems_(problems) {}

    template <class T>
    void operator()(std::string_view key, T& field, const Check<T>& check = {}) {
        known_.emplace_back(key);
        const std::string where = prefix_ + std::string(key);
        if (node_ != nullptr) {
            const auto it = node_->find(key);
            if (it != node_->end()) {
                try {
                    field = convert<T>(*it);
                } catch (const BadValue& bad) {
                    problems_.push_back(
                        std::format("  {}: {} (got {})", where, bad.message, shown(*it)));
                    return;
                }
            }
        }
        if (check) {
            if (auto message = check(field)) {
                problems_.push_back(std::format("  {}: {}", where, *message));
            }
        }
    }

    [[nodiscard]] const std::vector<std::string>& known() const { return known_; }

private:
    const Json* node_;
    std::string prefix_;
    std::vector<std::string>& problems_;
    std::vector<std::string> known_;
};

template <class Visitor>
void describe(Visitor& v, BrokerConfig& c) {
    v("host", c.host);
    v("port", c.port, range(1, 65535));
    v("tls", c.tls);
    v("ca_file", c.ca_file);
    v("insecure", c.insecure);
    v("keepalive", c.keepalive, above(0));
    v("transport", c.transport);
    v("ws_path", c.ws_path);
    v("username", c.username);
    v("password", c.password);
    v("qos", c.qos, qos_level());
}

template <class Visitor>
void describe(Visitor& v, HttpConfig& c) {
    v("base_url", c.base_url);
    v("auth_path", c.auth_path);
    v("org_path", c.org_path);
    v("username", c.username);
    v("password", c.password);
    v("verify_tls", c.verify_tls);
    v("ca_file", c.ca_file);
}

template <class Visitor>
void describe(Visitor& v, OrganizationConfig& c) {
    v("bind_on_start", c.bind_on_start);
    v("organization_id", c.organization_id);
    v("binding_code", c.binding_code);
    v("dock_callsign", c.dock_callsign);
    v("drone_callsign", c.drone_callsign);
}

template <class Visitor>
void describe(Visitor& v, DeviceConfig& c) {
    v("dock_sn", c.dock_sn);
    v("drone_sn", c.drone_sn);
    v("dock_model", c.dock_model, dock_model());
    v("drone_model", c.drone_model, drone_model());
    v("thing_version", c.thing_version);
    v("firmware_version", c.firmware_version);
    v("dock_firmware_version", c.dock_firmware_version);
    v("drone_firmware_version", c.drone_firmware_version);
    v("device_secret", c.device_secret);
    v("license_key", c.license_key, optional_check(public_key()));
    v("latitude", c.latitude, range(-90.0, 90.0));
    v("longitude", c.longitude, range(-180.0, 180.0));
    v("altitude", c.altitude);
    v("channel_index", c.channel_index);
}

template <class Visitor>
void describe(Visitor& v, TelemetryConfig& c) {
    v("dock_osd_interval", c.dock_osd_interval, above(0.0));
    v("drone_osd_interval", c.drone_osd_interval, above(0.0));
    v("state_interval", c.state_interval, above(0.0));
    v("topo_interval", c.topo_interval, at_least(0.0));
}

template <class Visitor>
void describe(Visitor& v, MediaConfig& c) {
    v("enabled", c.enabled);
    v("upload_delay", c.upload_delay, at_least(0.0));
    v("image_size", c.image_size,
      Check<std::pair<int, int>>{[](const auto& s) -> std::optional<std::string> {
          if (s.first <= 0 || s.second <= 0) return "both sizes must be positive";
          return std::nullopt;
      }});
    v("endpoint_override", c.endpoint_override);
    v("verify_tls", c.verify_tls);
    v("renderer", c.renderer);
    v("imagery_url", c.imagery_url);
    v("imagery_max_zoom", c.imagery_max_zoom, range(0, 22));
    v("imagery_attribution", c.imagery_attribution);
    v("dem_url", c.dem_url);
    v("dem_zoom", c.dem_zoom, range(0, 15));
    v("dem_near_zoom", c.dem_near_zoom, range(0, 15));
    v("dem_near_radius", c.dem_near_radius, at_least(0.0));
    v("cache_dir", c.cache_dir);
    v("render_device", c.render_device);
    v("rt_allow_cpu", c.rt_allow_cpu);
    v("gpu_device", c.gpu_device);
    v("buildings", c.buildings);
    v("overpass_urls", c.overpass_urls);
    v("building_default_height", c.building_default_height, above(0.0));
    v("building_level_height", c.building_level_height, above(0.0));
    v("building_default_style", c.building_default_style);
    v("building_scan", c.building_scan);
    v("building_scan_radius", c.building_scan_radius, at_least(0.0));
    v("building_radius", c.building_radius, above(0.0));
    v("building_far_radius", c.building_far_radius, at_least(0.0));
    v("building_far_resolution", c.building_far_resolution, above(0.0));
    v("building_heights", c.building_heights);
    v("shadows", c.shadows);
    v("supersample", c.supersample, range(1, 4));
    v("lighting_time", c.lighting_time);
    v("lighting_timezone", c.lighting_timezone);
    v("lighting_frozen", c.lighting_frozen);
}

template <class Visitor>
void describe(Visitor& v, LiveConfig& c) {
    v("enabled", c.enabled);
    v("fps", c.fps, range(1, 60));
    v("size", c.size, frame_size());
    v("bitrate", c.bitrate, bitrate());
    v("snapshot_dir", c.snapshot_dir);
    v("overlay", c.overlay);
    v("candidate_host", c.candidate_host);
    v("candidate_port", c.candidate_port, optional_check(range(1, 65535)));
    v("source", c.source);
    v("render_workers", c.render_workers, above(0));
    v("march_scale", c.march_scale, optional_check(above(0)));
    v("supersample", c.supersample, range(1, 4));
    v("encoder", c.encoder);
}

template <class Visitor>
void describe(Visitor& v, FlightConfig& c) {
    v("speed", c.speed, above(0.0));
    v("climb_rate", c.climb_rate, above(0.0));
    v("takeoff_altitude", c.takeoff_altitude, above(0.0));
    v("tick", c.tick, above(0.0));
    v("prepare_seconds", c.prepare_seconds, at_least(0.0));
    v("land_seconds", c.land_seconds, at_least(0.0));
}

template <class Visitor>
void describe(Visitor& v, MaintenanceConfig& c) {
    v("job_duration", c.job_duration, at_least(0.0));
    v("reboot_duration", c.reboot_duration, at_least(0.0));
    v("ota_duration", c.ota_duration, at_least(0.0));
    v("log_step_duration", c.log_step_duration, at_least(0.0));
    v("sync_flight_areas_on_start", c.sync_flight_areas_on_start);
    v("flight_area_report_interval", c.flight_area_report_interval, above(0.0));
    v("hms_interval", c.hms_interval, at_least(0.0));
    v("hms", c.hms, hms_alarms());
}

template <class Visitor>
void describe(Visitor& v, WeatherConfig& c) {
    v("max_wind_takeoff", c.max_wind_takeoff, above(0.0));
    v("max_rainfall_takeoff", c.max_rainfall_takeoff, range(1, 3));
}

template <class Visitor>
void describe(Visitor& v, ApiConfig& c) {
    v("enabled", c.enabled);
    v("bind", c.bind);
    v("port", c.port, range(0, 65535));
    v("token", c.token);
    v("web_root", c.web_root);
}

// Calls fn(name, section) for each section in file order.
template <class Fn>
void for_each_section(Config& c, Fn&& fn) {
    fn("broker", c.broker);
    fn("http", c.http);
    fn("device", c.device);
    fn("organization", c.organization);
    fn("telemetry", c.telemetry);
    fn("media", c.media);
    fn("live", c.live);
    fn("flight", c.flight);
    fn("maintenance", c.maintenance);
    fn("weather", c.weather);
    fn("api", c.api);
}

bool is_retired(std::string_view dotted) {
    return std::ranges::find(retired_keys, dotted) != retired_keys.end();
}

void bind_config(Config& cfg, const Json* root, std::vector<std::string>& problems,
                 std::vector<std::string>* unknown) {
    std::vector<std::string> known_sections;
    for_each_section(cfg, [&](std::string_view name, auto& section) {
        known_sections.emplace_back(name);
        const Json* node = nullptr;
        if (root != nullptr) {
            const auto it = root->find(name);
            if (it != root->end()) {
                if (!it->is_object()) {
                    problems.push_back(
                        std::format("  {}: must be a mapping (got {})", name, shown(*it)));
                    return;
                }
                node = &*it;
            }
        }
        Binder binder(node, std::format("{}.", name), problems);
        describe(binder, section);
        if (node != nullptr && unknown != nullptr) {
            for (const auto& [key, _] : node->items()) {
                const auto dotted = std::format("{}.{}", name, key);
                if (std::ranges::find(binder.known(), key) == binder.known().end() &&
                    !is_retired(dotted)) {
                    unknown->push_back(dotted);
                }
            }
        }
    });

    Binder top(root, "", problems);
    top("log_level", cfg.log_level);
    if (root != nullptr && unknown != nullptr) {
        for (const auto& [key, _] : root->items()) {
            if (key != "log_level" &&
                std::ranges::find(known_sections, key) == known_sections.end()) {
                unknown->push_back(key);
            }
        }
    }
}

[[noreturn]] void fail(std::string_view source, const std::vector<std::string>& problems) {
    std::string message = std::format("{} has {} problem(s):", source, problems.size());
    for (const auto& p : problems) message += "\n" + p;
    throw ConfigError(message);
}

Json read_file(const std::filesystem::path& path) {
    const std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw ConfigError(
            std::format("cannot read {}: {}", path.string(),
                        std::make_error_code(std::errc::no_such_file_or_directory).message()));
    }
    std::ostringstream text;
    text << in.rdbuf();
    Json data;
    try {
        data = yaml::parse(text.str());
    } catch (const std::exception& e) {
        throw ConfigError(std::format("{} is not valid YAML: {}", path.string(), e.what()));
    }
    if (data.is_null()) return Json::object();
    if (!data.is_object()) {
        throw ConfigError(std::format("{} must hold a mapping at the top level", path.string()));
    }
    return data;
}

void apply_env(Json& raw, const Environment& env) {
    for (const auto& [name, value] : env) {
        if (!name.starts_with(env_prefix) || value.empty()) continue;
        std::vector<std::string> keys;
        if (const auto* alias = std::ranges::find(env_aliases, name, &Alias::env);
            alias != env_aliases.end()) {
            if (!alias->section.empty()) keys.emplace_back(alias->section);
            keys.emplace_back(alias->key);
        } else if (name.find("__") != std::string::npos) {
            const std::string_view rest = std::string_view(name).substr(env_prefix.size());
            for (std::size_t pos = 0;;) {
                const auto next = rest.find("__", pos);
                keys.push_back(lower(std::string(rest.substr(pos, next - pos))));
                if (next == std::string_view::npos) break;
                pos = next + 2;
            }
        } else {
            continue;
        }
        Json* target = &raw;
        for (std::size_t i = 0; i + 1 < keys.size(); ++i) {
            Json& next = (*target)[keys[i]];
            if (!next.is_object()) next = Json::object();
            target = &next;
        }
        (*target)[keys.back()] = value;
    }
}

}  // namespace

std::int64_t parse_bitrate(std::string_view text) {
    static const std::regex re{R"(\s*([0-9]+(\.[0-9]+)?)\s*([kKmM]?)\s*)"};
    std::match_results<std::string_view::const_iterator> m;
    if (!std::regex_match(text.begin(), text.end(), m, re)) {
        throw ConfigError("must be a bitrate such as 6M, 800k or 2000000");
    }
    const double value = num::parse_double(m[1].str()).value();
    const char unit = m[3].length() > 0 ? static_cast<char>(std::tolower(*m[3].first)) : '\0';
    const double scale = unit == 'k' ? 1e3 : unit == 'm' ? 1e6 : 1.0;
    return static_cast<std::int64_t>(value * scale);
}

std::int64_t LiveConfig::bitrate_bps() const {
    return parse_bitrate(bitrate);
}

std::pair<int, int> LiveConfig::frame() const {
    const auto parsed = parse_size(size);
    if (!parsed) throw ConfigError(std::format("live.size {:?} must be WIDTHxHEIGHT", size));
    return *parsed;
}

std::optional<std::pair<int, int>> parse_size(std::string_view text) {
    const auto whole = [](std::string_view part) -> std::optional<int> {
        int v = 0;
        const auto* end = part.data() + part.size();
        const auto [ptr, ec] = std::from_chars(part.data(), end, v);
        if (ec != std::errc{} || ptr != end || v <= 0) return std::nullopt;
        return v;
    };
    const auto x = text.find('x');
    if (x == std::string_view::npos) return std::nullopt;
    const auto w = whole(text.substr(0, x));
    const auto h = whole(text.substr(x + 1));
    if (!w || !h) return std::nullopt;
    return std::pair{*w, *h};
}

void Config::validate() const {
    Config copy = *this;
    std::vector<std::string> problems;
    bind_config(copy, nullptr, problems, nullptr);
    if (!problems.empty()) fail("configuration", problems);
}

LoadResult load_config(const std::optional<std::filesystem::path>& path, const Environment& env) {
    Json raw = path ? read_file(*path) : Json::object();
    apply_env(raw, env);
    LoadResult result;
    std::vector<std::string> problems;
    bind_config(result.config, &raw, problems, &result.unknown_keys);
    if (!problems.empty()) fail(path ? path->string() : "configuration", problems);
    return result;
}

Environment process_environment() {
    Environment env;
    for (char* const* e = environ; e != nullptr && *e != nullptr; ++e) {
        const std::string_view entry(*e);
        if (!entry.starts_with(env_prefix)) continue;
        const auto eq = entry.find('=');
        if (eq == std::string_view::npos) continue;
        env.emplace(entry.substr(0, eq), entry.substr(eq + 1));
    }
    return env;
}

}  // namespace docksim
