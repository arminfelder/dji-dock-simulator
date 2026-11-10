#include "protocol/payloads.hpp"

#include "device/catalog.hpp"
#include "device/geo.hpp"
#include "util/numeric.hpp"

#include <cmath>
#include <format>

namespace docksim::payloads {

namespace {

using num::round_to;

// One live-capacity entry. video_index is the lens name and the gimbal index,
// which is what a video_id repeats after the payload index.
Json capacity_device(const std::string& sn, std::string_view camera_index,
                     std::span<const std::string_view> video_types) {
    Json types = Json::array();
    for (const auto t : video_types) types.push_back(t);
    if (types.empty()) types.push_back("normal");
    return {
        {"sn", sn},
        {"available_video_number", 1},
        {"coexist_video_number_max", 1},
        {"camera_list", Json::array({{
                            {"available_video_number", 1},
                            {"coexist_video_number_max", 1},
                            {"camera_index", camera_index},
                            {"video_list", Json::array({{{"video_index", "normal-0"},
                                                         {"video_type", "normal"},
                                                         {"switchable_video_types", types}}})},
                        }})},
    };
}

Json lens_exposure(std::string_view lens, const device::LensSettings& s) {
    const auto key = [lens](std::string_view name) { return std::format("{}_{}", lens, name); };
    return {
        {key("exposure_mode"), s.exposure_mode},
        {key("iso"), s.iso},
        {key("shutter_speed"), s.shutter_speed},
        {key("exposure_value"), s.exposure_value},
        {key("aperture_value"), s.aperture_value},
        {key("focus_mode"), s.focus_mode},
        {key("focus_state"), s.focus_state},
        {key("focus_value"), s.focus_value},
        {key("max_focus_value"), device::focus_value_max},
        {key("min_focus_value"), device::focus_value_min},
        {key("calibrate_farthest_focus_value"), device::focus_value_min + 1},
        {key("calibrate_nearest_focus_value"), device::focus_value_max},
    };
}

Json ir_metering(const device::IrMetering& ir) {
    // A flat scene at 18 degrees Celsius, with a warm spot in the metering area.
    return {
        {"ir_metering_mode", ir.mode},
        {"ir_metering_point",
         {{"x", round_to(ir.point_x, 3)}, {"y", round_to(ir.point_y, 3)}, {"temperature", 18.0}}},
        {"ir_metering_area",
         {
             {"x", round_to(ir.area_x, 3)},
             {"y", round_to(ir.area_y, 3)},
             {"width", round_to(ir.area_width, 3)},
             {"height", round_to(ir.area_height, 3)},
             {"aver_temperature", 18.5},
             {"max_temperature_point",
              {{"x", round_to(ir.area_x + ir.area_width / 2, 3)},
               {"y", round_to(ir.area_y + ir.area_height / 2, 3)},
               {"temperature", 24.0}}},
             {"min_temperature_point",
              {{"x", round_to(ir.area_x, 3)},
               {"y", round_to(ir.area_y, 3)},
               {"temperature", 16.0}}},
         }},
    };
}

Json storage_list(const std::vector<std::string>& lenses) {
    Json out = Json::array();
    for (const auto& lens : lenses) out.push_back(lens);
    return out;
}

// One entry of the aircraft osd cameras. The cloud gates payload commands on
// photo_state, recording_state, camera_mode and the remaining capacity.
Json osd_camera(const device::DroneState& drone) {
    const auto& camera = drone.camera;
    Json out = {
        {"payload_index", drone.payload_index},
        {"camera_mode", camera.camera_mode},
        {"photo_state", camera.photo_state},
        {"recording_state", camera.recording_state},
        {"record_time", camera.record_time},
        {"remain_photo_num", drone.remain_photo_num()},
        {"remain_record_duration", drone.remain_record_duration()},
        {"remain_upload", 0},
        {"zoom_factor", round_to(drone.zoom_factor, 1)},
        {"ir_zoom_factor", round_to(drone.ir_zoom_factor, 1)},
        {"screen_split_enable", camera.screen_split_enable},
        {"photo_storage_settings", storage_list(camera.photo_storage_settings)},
        {"video_storage_settings", storage_list(camera.video_storage_settings)},
        {"thermal_supported_palette_styles", Json::array({0, 1, 2, 3, 5, 6, 8, 11, 12, 13})},
    };
    out.update(lens_exposure("wide", camera.wide));
    out.update(lens_exposure("zoom", camera.zoom));
    out.update(ir_metering(camera.ir));
    return out;
}

}  // namespace

std::vector<Json> dock_osd(device::DockState& dock) {
    const auto& cfg = *dock.cfg;
    const auto& weather = dock.weather;
    Json half = {
        {"mode_code", dock.mode_code},
        {"cover_state", dock.cover_state},
        {"putter_state", dock.putter_state},
        {"supplement_light_state", dock.supplement_light_state},
        {"drone_in_dock", dock.drone_in_dock},
        {"drone_charge_state",
         {{"state", dock.drone_charge_state}, {"capacity_percent", dock.drone_charge_percent}}},
        {"emergency_stop_state", dock.emergency_stop_state},
        {"alarm_state", dock.alarm_state},
        {"battery_store_mode", dock.battery_store_mode},
        {"air_conditioner",
         {{"air_conditioner_state", dock.air_conditioner_state}, {"switch_time", 0}}},
        {"rainfall", weather.noise ? 0 : weather.rainfall},
        {"wind_speed", weather.noise ? round_to(dock.uniform(0.4, 3.2), 1)
                                     : round_to(weather.wind_speed, 1)},
        {"environment_temperature", weather.noise ? round_to(dock.uniform(8.0, 21.0), 1)
                                                  : round_to(weather.temperature, 1)},
        {"temperature", round_to(dock.uniform(18.0, 26.0), 1)},
        {"humidity", weather.noise ? dock.randint(35, 60) : weather.humidity},
        {"latitude", dock.site.latitude},
        {"longitude", dock.site.longitude},
        {"height", dock.site.altitude},
        {"storage", {{"total", 64'000'000}, {"used", 1'200'000}}},
        {"network_state", {{"type", 2}, {"quality", 4}, {"rate", dock.network_rate}}},
        {"position_state",
         {{"is_calibration", dock.gnss.fix},
          {"quality", dock.gnss.quality},
          {"gps_number", dock.gnss.gps_number},
          {"rtk_number", dock.gnss.rtk_number}}},
        {"sub_device",
         {
             {"device_sn", cfg.drone_sn},
             {"device_model_key", catalog::model_key(cfg.drone_model)},
             {"device_online_status", dock.drone_powered ? 1 : 0},
             {"device_paired", 1},
         }},
    };
    // The real dock sends these in a message of its own. The backend needs that:
    // it keeps the stored drc_state for each message with mode_code. Thus
    // drc_state next to mode_code never gets to the DRC check of the backend.
    Json mission = {
        {"flighttask_step_code", dock.flighttask_step_code},
        {"flighttask_prepare_capacity", dock.flighttask_prepare_capacity},
        {"media_file_detail", {{"remain_upload", dock.media_remain_upload}}},
        {"wireless_link",
         {
             {"dongle_number", 1},
             {"link_workmode", dock.link_workmode},
             {"sdr_link_state", 1},
             {"sdr_quality", 5},
             {"sdr_freq_band", 2.4999},
             {"4g_link_state", 1},
             {"4g_quality", 5},
             {"4g_freq_band", 2.4999},
             {"4g_gnd_quality", 5},
             {"4g_uav_quality", 5},
         }},
        {"drc_state", dock.drc_state},
    };
    Json following = {
        {"job_number", dock.job_number},
        {"acc_time", dock.acc_time},
        {"activation_time", 1700000000},
        {"electric_supply_voltage", 230},
        {"working_voltage", 25440},
        {"working_current", 1120},
        {"backup_battery", {{"voltage", 26631}, {"temperature", 27.9}, {"switch", 1}}},
        {"drone_battery_maintenance_info",
         {{"maintenance_state", dock.battery_maintenance_state}, {"maintenance_time_left", 0}}},
    };
    return {std::move(half), std::move(mission), std::move(following)};
}

Json firmware_group(const std::string& version, bool upgrading) {
    return {{"firmware_version", version},
            {"compatible_status", true},
            {"firmware_upgrade_status", upgrading}};
}

std::vector<Json> dock_state(const device::DockState& dock) {
    const auto& cfg = *dock.cfg;
    static constexpr std::array<std::string_view, 1> dock_lenses{"normal"};
    return {
        firmware_group(dock.firmware_version, dock.firmware_upgrading),
        {{"live_capacity",
          {
              {"available_video_number", 2},
              {"coexist_video_number_max", 2},
              {"device_list",
               Json::array({
                   capacity_device(cfg.dock_sn, catalog::dock_camera_index, dock_lenses),
                   capacity_device(cfg.drone_sn, catalog::camera_index(cfg.drone_model),
                                   catalog::switchable_video_types(cfg.drone_model)),
               })},
          }}},
    };
}

Json drone_osd(const device::DroneState& drone) {
    Json osd = Json::object();
    // Gimbal attitude per payload. gimbal_yaw is the geographic heading.
    osd[drone.payload_index] = {
        {"payload_index", drone.payload_index},
        {"gimbal_pitch", round_to(drone.gimbal_pitch, 1)},
        {"gimbal_roll", 0.0},
        {"gimbal_yaw", round_to(geo::signed_angle(drone.camera_heading()), 1)},
        {"zoom_factor", round_to(drone.zoom_factor, 1)},
    };
    osd.update(Json{
        {"mode_code", drone.mode_code},
        {"gear", drone.gear},
        {"latitude", round_to(drone.latitude, 8)},
        {"longitude", round_to(drone.longitude, 8)},
        {"height", round_to(drone.home.altitude + drone.height, 3)},
        {"elevation", round_to(drone.height, 3)},
        {"attitude_head", round_to(drone.attitude_head, 1)},
        {"attitude_pitch", 0.0},
        {"attitude_roll", 0.0},
        {"horizontal_speed", round_to(drone.horizontal_speed, 2)},
        {"vertical_speed", round_to(drone.vertical_speed, 2)},
        {"home_distance", round_to(drone.home_distance(), 2)},
        {"home_latitude", drone.home.latitude},
        {"home_longitude", drone.home.longitude},
        {"total_flight_distance", round_to(drone.total_flight_distance, 2)},
        {"total_flight_time", static_cast<std::int64_t>(std::trunc(drone.total_flight_time))},
        {"battery", drone_battery(drone)},
        {"position_state",
         {{"is_fixed", drone.gnss.fix},
          {"gps_number", drone.gnss.gps_number},
          {"rtk_number", drone.gnss.rtk_number},
          {"quality", drone.gnss.quality}}},
        {"storage", {{"total", 128'000'000}, {"used", 4'000'000}}},
        {"firmware_version", drone.firmware_version},
        {"cameras", Json::array({osd_camera(drone)})},
    });
    return osd;
}

Json drone_battery(const device::DroneState& drone) {
    const auto& cfg = *drone.cfg;
    const int battery = drone.battery_percent();
    return {
        {"capacity_percent", battery},
        {"remain_flight_time", std::max(0, battery * 15)},
        {"return_home_power", device::return_home_power_percent},
        {"landing_power", device::landing_power_percent},
        {"batteries", Json::array({{
                          {"capacity_percent", battery},
                          {"index", 0},
                          {"sn", std::format("{}-BAT", cfg.drone_sn)},
                          {"temperature", 26.0},
                          {"voltage", 24800},
                          {"high_voltage_storage_days", 4},
                          {"loop_times", 42},
                          {"firmware_version", "01.00.00.00"},
                      }})},
    };
}

Json drc_batteries_info(const device::DroneState& drone) {
    Json out = drone_battery(drone);
    out["bat_remaining_capacity"] = out["capacity_percent"];
    out["remain_job_time"] = out["remain_flight_time"];
    out["need_gohome_time"] = 0;
    out["low_battery_warning_threshold"] = 25;
    out["serious_low_battery_warning_threshold"] = 15;
    return out;
}

Json control_source_group(const device::DroneState& drone) {
    const auto& cfg = *drone.cfg;
    return {
        {"control_source", "A"},
        {"home_latitude", drone.home.latitude},
        {"home_longitude", drone.home.longitude},
        {"low_battery_warning_threshold", 25},
        {"serious_low_battery_warning_threshold", 15},
        {"payloads", Json::array({{
                         {"sn", std::format("{}-CAM", cfg.drone_sn)},
                         {"payload_index", drone.payload_index},
                         {"control_source", "A"},
                     }})},
    };
}

std::vector<Json> drone_state(const device::DroneState& drone) {
    return {firmware_group(drone.firmware_version, drone.firmware_upgrading),
            control_source_group(drone)};
}

Json drc_osd_info(const device::DroneState& drone) {
    const double course = geo::radians(drone.course);
    return {
        {"latitude", round_to(drone.latitude, 8)},
        {"longitude", round_to(drone.longitude, 8)},
        {"height", round_to(drone.home.altitude + drone.height, 3)},
        {"elevation", round_to(drone.height, 3)},
        {"attitude_head", round_to(geo::signed_angle(drone.attitude_head), 1)},
        {"horizontal_speed", round_to(drone.horizontal_speed, 2)},
        {"vertical_speed", round_to(drone.vertical_speed, 2)},
        {"speed_x", round_to(drone.horizontal_speed * std::cos(course), 2)},
        {"speed_y", round_to(drone.horizontal_speed * std::sin(course), 2)},
        {"speed_z", round_to(drone.vertical_speed, 2)},
        {"home_distance", round_to(drone.home_distance(), 2)},
        {"gimbal_pitch", round_to(drone.gimbal_pitch, 1)},
        {"gimbal_roll", 0.0},
        {"gimbal_yaw", round_to(geo::signed_angle(drone.camera_heading()), 1)},
        {"wind_speed", drone.wind_speed},
        {"wind_direction", drone.wind_direction},
        {"ultrasonic_height", -1},
    };
}

Json drc_hsi_info(const device::DroneState& drone) {
    const int down_mm = static_cast<int>(std::lround(std::max(0.0, drone.height) * 1000.0));
    Json hsi = {
        {"up_distance", hsi_clear_mm},
        {"down_distance", std::min(down_mm, hsi_clear_mm)},
        {"around_distances", Json(std::vector<int>(360, hsi_clear_mm))},
    };
    for (const auto* side :
         {"up", "down", "left", "right", "front", "back", "vertical", "horizontal"}) {
        hsi[std::format("{}_enable", side)] = true;
        hsi[std::format("{}_work", side)] = true;
    }
    return hsi;
}

Json drc_delay_info(const device::DroneState& drone) {
    return {
        {"sdr_cmd_delay", 20},
        {"liveview_delay_list",
         Json::array({{{"video_id",
                        std::format("{}/{}/normal-0", drone.cfg->drone_sn, drone.payload_index)},
                       {"liveview_delay_time", 120}}})},
    };
}

Json drc_drone_state(const device::DroneState& drone) {
    return {
        {"mode_code", drone.mode_code},
        {"rth_altitude", static_cast<int>(std::lround(drone.rth_altitude))},
        {"night_lights_state", drone.night_lights_state},
        {"stealth_state", drone.stealth_state},
        {"current_commander_flight_mode", 0},
        {"is_near_height_limit", 0},
        {"is_near_distance_limit", 0},
        {"assistant_video", {{"switch", 0}, {"mode", 0}}},
        {"landing_type", 0},
        {"landing_protection_type", 0},
    };
}

Json drc_camera_state(const device::DroneState& drone) {
    const auto& camera = drone.camera;
    return {
        {"payload_index", drone.payload_index},
        {"camera_state",
         {
             {"camera_mode", camera.camera_mode},
             {"photo_state", camera.photo_state},
             {"recording_state", camera.recording_state},
             {"record_time", camera.record_time},
             {"remain_photo_num", drone.remain_photo_num()},
             {"remain_record_duration", drone.remain_record_duration()},
             {"interval_photo_interval", camera.interval_photo_interval},
             {"linkage_zoom_state", camera.linkage_zoom_state ? 1 : 0},
             {"photo_size", camera.photo_size},
             {"video_resolution", camera.video_resolution},
         }},
        {"media_storage",
         {
             {"photo_storage_settings", storage_list(camera.photo_storage_settings)},
             {"video_storage_settings", storage_list(camera.video_storage_settings)},
         }},
    };
}

MeasureTarget measure_target(const device::DroneState& drone) {
    const double depression = -drone.gimbal_pitch;
    if (depression < 1.0 || drone.height <= 0.0) return {};
    const double tilt = geo::radians(depression);
    const double ground = drone.height / std::tan(tilt);
    if (ground > laser_range_max_m) return {};
    const double heading = geo::radians(drone.camera_heading());
    const double lat = geo::radians(drone.latitude);
    return {
        .latitude = drone.latitude + geo::degrees(ground * std::cos(heading) / geo::earth_radius_m),
        .longitude = drone.longitude + geo::degrees(ground * std::sin(heading) /
                                                    (geo::earth_radius_m * std::cos(lat))),
        .altitude = drone.home.altitude,
        .distance = drone.height / std::sin(tilt),
        .error_state = 0,
    };
}

Json drc_camera_osd_info(const device::DroneState& drone) {
    const auto& camera = drone.camera;
    const auto target = measure_target(drone);
    Json wide = lens_exposure("wide", camera.wide);
    wide["wide_dewarping_state"] = camera.wide.dewarping_state;
    wide["wide_mechanical_shutter_state"] = camera.wide.mechanical_shutter_state;
    Json zoom = lens_exposure("zoom", camera.zoom);
    zoom["zoom_factor"] = round_to(drone.zoom_factor, 1);
    // The zoom view inside the wide view. The zoom factor counts against the wide lens.
    const double half = 0.5 / std::max(drone.zoom_factor, 1.0);
    return {
        {"payload_index", drone.payload_index},
        {"gimbal_pitch", round_to(drone.gimbal_pitch, 1)},
        {"gimbal_roll", 0.0},
        {"gimbal_yaw", round_to(geo::signed_angle(drone.camera_heading()), 1)},
        {"wide_lense", std::move(wide)},
        {"zoom_lense", std::move(zoom)},
        {"ir_lense",
         {
             {"ir_zoom_factor", round_to(drone.ir_zoom_factor, 1)},
             {"screen_split_enable", camera.screen_split_enable},
             {"thermal_current_palette_style", 0},
             {"thermal_gain_mode", 2},
             {"thermal_isotherm_state", 0},
             {"thermal_isotherm_upper_limit", 150},
             {"thermal_isotherm_lower_limit", -20},
             {"thermal_global_temperature_min", 16.0},
             {"thermal_global_temperature_max", 24.0},
         }},
        {"ir_thermometry", ir_metering(camera.ir)},
        {"liveview",
         {{"liveview_world_region",
           {{"left", round_to(0.5 - half, 4)},
            {"top", round_to(0.5 - half, 4)},
            {"right", round_to(0.5 + half, 4)},
            {"bottom", round_to(0.5 + half, 4)}}}}},
        {"measure_target",
         {
             {"measure_target_latitude", round_to(target.latitude, 8)},
             {"measure_target_longitude", round_to(target.longitude, 8)},
             {"measure_target_altitude", round_to(target.altitude, 2)},
             {"measure_target_distance", round_to(target.distance, 2)},
             {"measure_target_error_state", target.error_state},
         }},
    };
}

Json file_upload_callback(const FileUpload& f) {
    return {
        {"result", 0},
        {"progress", 100},
        {"file",
         {
             {"cloud_to_cloud_id", "DEFAULT"},
             {"ext",
              {
                  {"drone_model_key", f.drone_model_key},
                  {"flight_id", f.flight_id},
                  {"is_original", true},
                  {"payload_model_key", f.payload_model_key},
              }},
             {"metadata",
              {
                  {"absolute_altitude", round_to(f.absolute_altitude, 3)},
                  {"created_time", num::iso8601_utc_seconds(f.created_time)},
                  {"gimbal_yaw_degree", round_to(f.gimbal_yaw, 1)},
                  {"relative_altitude", round_to(f.relative_altitude, 3)},
                  {"shoot_position",
                   {{"lat", round_to(f.latitude, 8)}, {"lng", round_to(f.longitude, 8)}}},
              }},
             {"name", f.file_name},
             {"object_key", f.object_key},
             {"path", f.path},
         }},
        {"flight_task",
         {
             {"expected_file_count", f.expected},
             {"flight_type", 0},
             {"uploaded_file_count", f.uploaded},
         }},
    };
}

}  // namespace docksim::payloads
