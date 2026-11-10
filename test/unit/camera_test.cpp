// Camera services and the DRC camera pushes, driven on the virtual clock of
// the golden harness.

#include "services/camera.hpp"

#include "flight_support.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

using namespace docksim;
using namespace docksim::golden;
using namespace docksim::flight_test;
using Catch::Approx;

namespace {

const std::string camera = "80-0-0";

Json payload(Json data = Json::object()) {
    data["payload_index"] = camera;
    return data;
}

Json osd_camera(Driver& d) {
    return drone_osd(d)["cameras"][0];
}

}  // namespace

TEST_CASE("camera: camera_photo_take reports progress and queues the photo", "[camera]") {
    Driver d(manual_config());
    d.connect();
    auto& drone = d.sim().ctx().drone;
    d.clear();
    d.service("camera_photo_take", payload());
    CHECK(reply_result(d, "camera_photo_take") == 0);
    CHECK(drone.camera.photo_state == 1);
    CHECK(osd_camera(d)["photo_state"] == 1);
    // The cloud and the dock refuse a photo while one is in progress.
    d.service("camera_photo_take", payload());
    CHECK(reply_result(d, "camera_photo_take") == 327006);
    CHECK(events(d, "camera_photo_take_progress").empty());

    d.advance(services::photo_seconds + 0.1);
    const auto progress = events(d, "camera_photo_take_progress");
    REQUIRE(progress.size() == 1);
    CHECK(progress[0]["result"] == 0);
    CHECK(progress[0]["output"]["status"] == "ok");
    CHECK(progress[0]["output"]["ext"]["camera_mode"] == 0);
    CHECK(progress[0]["output"]["progress"]["current_step"] == 0);
    CHECK(progress[0]["output"]["progress"]["percent"] == 100);
    CHECK(drone.camera.photo_state == 0);
    CHECK(drone.camera_photos == 1);
    const auto osd = osd_camera(d);
    CHECK(osd["photo_state"] == 0);
    CHECK(osd["remain_photo_num"] == 4095);

    // On the ground the photo uploads at once, under an id of its own.
    const auto& pending = d.sim().uploads().pending();
    REQUIRE(pending.size() == 1);
    const auto& task = pending.begin()->second;
    CHECK(task.media_count == 1);
    REQUIRE(task.shots.size() == 1);
    CHECK(task.shots[0].height == 0.0);

    d.service("camera_photo_take", payload());
    d.advance(services::photo_seconds + 0.1);
    REQUIRE(pending.size() == 2);
    CHECK(pending.begin()->first.substr(0, 8) != std::next(pending.begin())->first.substr(0, 8));
}

TEST_CASE("camera: a photo in flight uploads with the media of the flight", "[camera]") {
    Driver d(manual_config());
    hovering(d);
    auto& drone = d.sim().ctx().drone;
    d.service("camera_look_at", payload({{"latitude", home(d).latitude},
                                         {"longitude", home(d).longitude},
                                         {"height", home(d).altitude},
                                         {"locked", false}}));
    d.service("camera_photo_take", payload());
    d.advance(services::photo_seconds + 0.1);
    const auto& uploads = d.sim().uploads();
    CHECK(uploads.pending().empty());
    REQUIRE(uploads.held().contains("takeoff-1"));
    const auto& shot = uploads.held().at("takeoff-1").shots.at(0);
    CHECK(shot.height == Approx(40.0).margin(0.3));
    CHECK(shot.gimbal_pitch < -30.0);
    CHECK(shot.heading == Approx(drone.camera_heading()).margin(0.5));

    d.service("return_home", Json::object());
    d.wait_manual();
    CHECK(uploads.held().empty());
    REQUIRE(uploads.pending().contains("takeoff-1"));
    CHECK(uploads.pending().at("takeoff-1").media_count == 1);
    CHECK(d.sim().ctx().dock.media_remain_upload == 1);
}

TEST_CASE("camera: recording keeps record_time and the remaining duration", "[camera]") {
    Driver d(manual_config());
    d.connect();
    d.clear();
    d.service("camera_recording_start", payload());
    CHECK(reply_result(d, "camera_recording_start") == 0);
    d.advance(5.5);
    auto osd = osd_camera(d);
    CHECK(osd["camera_mode"] == 1);
    CHECK(osd["recording_state"] == 1);
    CHECK(osd["record_time"] == 5);
    CHECK(osd["remain_record_duration"] == 3595);

    d.service("camera_recording_start", payload());
    CHECK(reply_result(d, "camera_recording_start") == 327007);
    d.service("camera_mode_switch", payload({{"camera_mode", 0}}));
    CHECK(reply_result(d, "camera_mode_switch") == 327009);
    d.service("camera_photo_take", payload());
    CHECK(reply_result(d, "camera_photo_take") == 0);
    d.advance(2.0);

    d.service("camera_recording_stop", payload());
    CHECK(reply_result(d, "camera_recording_stop") == 0);
    osd = osd_camera(d);
    CHECK(osd["recording_state"] == 0);
    CHECK(osd["record_time"] == 0);
    CHECK(osd["remain_record_duration"] == 3593);
    d.service("camera_recording_stop", payload());
    CHECK(reply_result(d, "camera_recording_stop") == 327008);
}

TEST_CASE("camera: camera_mode_switch checks the mode", "[camera]") {
    Driver d(manual_config());
    d.connect();
    d.clear();
    d.service("camera_mode_switch", payload({{"camera_mode", 3}}));
    CHECK(reply_result(d, "camera_mode_switch") == 0);
    CHECK(osd_camera(d)["camera_mode"] == 3);
    d.service("camera_mode_switch", payload({{"camera_mode", 9}}));
    CHECK(reply_result(d, "camera_mode_switch") == 514171);
    d.service("camera_mode_switch", payload());
    CHECK(reply_result(d, "camera_mode_switch") == 514171);
    CHECK(osd_camera(d)["camera_mode"] == 3);
}

TEST_CASE("camera: storage, split screen, exposure, focus and IR settings show in the osd",
          "[camera]") {
    Driver d(manual_config());
    d.connect();
    d.clear();
    d.service("photo_storage_set",
              payload({{"photo_storage_settings", Json::array({"current", "ir"})}}));
    d.service("video_storage_set",
              payload({{"video_storage_settings", Json::array({"wide", "zoom", "vision"})}}));
    d.service("camera_screen_split", payload({{"enable", true}}));
    d.service("camera_exposure_mode_set", payload({{"camera_type", "zoom"}, {"exposure_mode", 4}}));
    d.service("camera_exposure_set", payload({{"camera_type", "wide"}, {"exposure_value", 19}}));
    d.service("camera_focus_mode_set", payload({{"camera_type", "zoom"}, {"focus_mode", 0}}));
    d.service("camera_focus_value_set", payload({{"camera_type", "zoom"}, {"focus_value", 99}}));
    d.service("camera_point_focus_action",
              payload({{"camera_type", "wide"}, {"x", 0.2}, {"y", 0.8}}));
    d.service("ir_metering_mode_set", payload({{"mode", 2}}));
    d.service("ir_metering_point_set", payload({{"x", 0.3}, {"y", 0.4}}));
    d.service("ir_metering_area_set",
              payload({{"x", 0.1}, {"y", 0.2}, {"width", 0.5}, {"height", 0.6}}));
    for (const auto* method :
         {"photo_storage_set", "video_storage_set", "camera_screen_split",
          "camera_exposure_mode_set", "camera_exposure_set", "camera_focus_mode_set",
          "camera_focus_value_set", "camera_point_focus_action", "ir_metering_mode_set",
          "ir_metering_point_set", "ir_metering_area_set"}) {
        INFO(method);
        CHECK(reply_result(d, method) == 0);
    }
    const auto osd = osd_camera(d);
    CHECK(osd["photo_storage_settings"] == Json::array({"current", "ir"}));
    CHECK(osd["video_storage_settings"] == Json::array({"wide", "zoom", "vision"}));
    CHECK(osd["screen_split_enable"] == true);
    CHECK(osd["zoom_exposure_mode"] == 4);
    CHECK(osd["wide_exposure_mode"] == 1);
    CHECK(osd["wide_exposure_value"] == 19);
    CHECK(osd["zoom_focus_mode"] == 0);
    CHECK(osd["zoom_focus_value"] == 64);
    CHECK(osd["wide_focus_state"] == 2);
    CHECK(osd["ir_metering_mode"] == 2);
    CHECK(osd["ir_metering_point"]["x"] == Approx(0.3));
    CHECK(osd["ir_metering_area"]["height"] == Approx(0.6));

    d.service("photo_storage_set", payload({{"photo_storage_settings", Json::array({"tele"})}}));
    CHECK(reply_result(d, "photo_storage_set") == 514171);
    d.service("camera_exposure_mode_set", payload({{"camera_type", "ir"}, {"exposure_mode", 1}}));
    CHECK(reply_result(d, "camera_exposure_mode_set") == 514171);
    d.service("ir_metering_point_set", payload({{"x", 1.5}, {"y", 0.4}}));
    CHECK(reply_result(d, "ir_metering_point_set") == 514171);
    CHECK(osd_camera(d)["photo_storage_settings"] == Json::array({"current", "ir"}));
}

TEST_CASE("camera: DRC pushes the camera state and the camera osd", "[camera]") {
    Driver d(manual_config());
    hovering(d);
    d.service("drc_mode_enter", drc_enter());
    d.drc_session().published.clear();
    d.advance(2.05);
    const auto state = drc_data(d, "drc_camera_state_push");
    const auto osd = drc_data(d, "drc_camera_osd_info_push");
    CHECK(state.size() == 2);
    CHECK(osd.size() == state.size());
    REQUIRE_FALSE(osd.empty());
    CHECK(state[0]["payload_index"] == camera);
    CHECK(state[0]["camera_state"]["remain_photo_num"] == 4096);
    CHECK(state[0]["camera_state"]["video_resolution"] == 0);
    CHECK(state[0]["camera_state"]["linkage_zoom_state"] == 0);
    CHECK(state[0]["media_storage"]["photo_storage_settings"] == Json::array({"current"}));
    CHECK(osd[0]["payload_index"] == camera);
    CHECK(osd[0]["gimbal_pitch"] == Approx(-90.0));
    CHECK(osd[0]["zoom_lense"]["zoom_factor"] == Approx(7.0));
    CHECK(osd[0]["wide_lense"]["wide_dewarping_state"] == 0);
    CHECK(osd[0]["ir_lense"]["screen_split_enable"] == false);
    // Straight down from 40 m: the target is under the aircraft.
    const auto& target = osd[0]["measure_target"];
    const auto& drone = d.sim().ctx().drone;
    CHECK(target["measure_target_error_state"] == 0);
    CHECK(target["measure_target_distance"].get<double>() == Approx(drone.height).margin(0.05));
    CHECK(target["measure_target_latitude"].get<double>() == Approx(drone.latitude).margin(1e-6));
    CHECK(target["measure_target_altitude"].get<double>() == Approx(home(d).altitude));

    SECTION("a level camera has no target") {
        d.service("gimbal_reset", payload({{"reset_mode", 0}}));
        d.drc_session().published.clear();
        d.advance(1.05);
        const auto level = drc_data(d, "drc_camera_osd_info_push");
        REQUIRE_FALSE(level.empty());
        CHECK(level.back()["measure_target"]["measure_target_error_state"] == 1);
    }

    SECTION("drc/down camera setters store their value and push at once") {
        d.drc_session().published.clear();
        d.drc_inject(drc_down, {{"method", "drc_camera_iso_set"},
                                {"data", payload({{"camera_type", "zoom"}, {"iso_value", 5}})},
                                {"seq", "iso"}});
        const auto replies = drc_up(d, "drc_camera_iso_set");
        REQUIRE(replies.size() == 1);
        CHECK(replies[0]["seq"] == "iso");
        CHECK(replies[0]["data"]["result"] == 0);
        const auto pushed = drc_data(d, "drc_camera_osd_info_push");
        REQUIRE(pushed.size() == 1);
        CHECK(pushed[0]["zoom_lense"]["zoom_iso"] == 5);
        CHECK(pushed[0]["wide_lense"]["wide_iso"] == 0);

        const std::vector<std::pair<std::string, Json>> setters{
            {"drc_camera_shutter_set", {{"camera_type", "wide"}, {"shutter_value", 27}}},
            {"drc_camera_aperture_value_set", {{"camera_type", "zoom"}, {"aperture_value", 440}}},
            {"drc_camera_dewarping_set", {{"camera_type", "wide"}, {"dewarping_state", 1}}},
            {"drc_camera_mechanical_shutter_set",
             {{"camera_type", "wide"}, {"mechanical_shutter_state", 1}}},
            {"drc_video_resolution_set", {{"video_resolution", 1}}},
            {"drc_linkage_zoom_set", {{"state", 1}}},
            {"drc_interval_photo_set", {{"interval", 3}}},
            {"drc_photo_storage_set", {{"photo_storage_settings", Json::array({"zoom"})}}},
            {"drc_video_storage_set", {{"video_storage_settings", Json::array({"ir"})}}},
            {"drc_camera_mode_switch", {{"camera_mode", 1}}},
        };
        for (const auto& [method, data] : setters) {
            d.drc_inject(drc_down, {{"method", method}, {"data", payload(data)}, {"seq", method}});
            const auto reply = drc_up(d, method);
            INFO(method);
            REQUIRE(reply.size() == 1);
            CHECK(reply[0]["data"]["result"] == 0);
        }
        const auto last_osd = drc_data(d, "drc_camera_osd_info_push").back();
        CHECK(last_osd["wide_lense"]["wide_shutter_speed"] == 27);
        CHECK(last_osd["zoom_lense"]["zoom_aperture_value"] == 440);
        CHECK(last_osd["wide_lense"]["wide_dewarping_state"] == 1);
        CHECK(last_osd["wide_lense"]["wide_mechanical_shutter_state"] == 1);
        const auto last_state = drc_data(d, "drc_camera_state_push").back();
        CHECK(last_state["camera_state"]["video_resolution"] == 1);
        CHECK(last_state["camera_state"]["linkage_zoom_state"] == 1);
        CHECK(last_state["camera_state"]["interval_photo_interval"] == Approx(3.0));
        CHECK(last_state["camera_state"]["camera_mode"] == 1);
        CHECK(last_state["media_storage"]["photo_storage_settings"] == Json::array({"zoom"}));
        CHECK(last_state["media_storage"]["video_storage_settings"] == Json::array({"ir"}));

        d.drc_inject(drc_down, {{"method", "drc_photo_storage_set"},
                                {"data", payload({{"photo_storage_settings", "zoom"}})},
                                {"seq", "bad"}});
        CHECK(drc_up(d, "drc_photo_storage_set").back()["data"]["result"] == 514171);
    }
}
