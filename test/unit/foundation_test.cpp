#include "cloud/license.hpp"
#include "cloud/redact.hpp"
#include "device/catalog.hpp"
#include "device/geo.hpp"
#include "device/gimbal.hpp"
#include "protocol/codes.hpp"
#include "protocol/envelope.hpp"
#include "render/sun.hpp"
#include "util/base64.hpp"
#include "util/numeric.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/rsa.h>

#include <cmath>
#include <memory>
#include <regex>

using namespace docksim;
using Catch::Approx;

TEST_CASE("round_to rounds on the decimal value, ties to even") {
    CHECK(num::round_to(2.675, 2) == 2.67);
    CHECK(num::round_to(0.125, 2) == 0.12);
    CHECK(num::round_to(0.375, 2) == 0.38);
    CHECK(num::round_to(13.65, 1) == 13.7);
    CHECK(num::round_to(2.49985, 4) == 2.4998);
    CHECK(num::round_to(-1.25, 1) == -1.2);
}

TEST_CASE("floor_mod takes the sign of the divisor") {
    CHECK(num::floor_mod(-10.0, 360.0) == 350.0);
    CHECK(num::floor_mod(370.0, 360.0) == 10.0);
}

TEST_CASE("parse_double accepts whitespace, inf and nan") {
    CHECK(num::parse_double(" 12.5 ") == 12.5);
    CHECK(std::isinf(*num::parse_double("-inf")));
    CHECK(std::isnan(*num::parse_double("nan")));
    CHECK_FALSE(num::parse_double("12a"));
    CHECK_FALSE(num::parse_double("--5"));
    CHECK_FALSE(num::parse_double("+-5"));
    CHECK_FALSE(num::parse_double("-"));
    CHECK(num::parse_double("-5") == -5.0);
    CHECK(num::parse_double("+5") == 5.0);
    CHECK_FALSE(num::parse_double(""));
}

TEST_CASE("iso8601 uses +00:00") {
    const std::chrono::sys_seconds t{std::chrono::seconds{1'760'000'000}};
    CHECK(num::iso8601_utc_seconds(t) == "2025-10-09T08:53:20+00:00");
}

TEST_CASE("haversine of one degree of latitude") {
    CHECK(units::in_metres(geo::haversine(geo::point(47.0, 11.0), geo::point(48.0, 11.0))) ==
          Approx(111'195).epsilon(1e-4));
}

TEST_CASE("bearing to the cardinal points") {
    const auto [lat2, lon2, expected] = GENERATE(table<double, double, double>(
        {{48.0, 11.0, 0.0}, {47.0, 12.0, 90.0}, {46.0, 11.0, 180.0}, {47.0, 10.0, 270.0}}));
    CHECK(units::in_degrees(geo::bearing(geo::point(47.0, 11.0), geo::point(lat2, lon2))) ==
          Approx(expected).margin(0.4));
}

TEST_CASE("move_towards stops at the target and moves part of the way") {
    const auto start = geo::point(47.0, 11.0);
    const auto target = geo::point(47.001, 11.0);
    CHECK(geo::move_towards(start, target, 1e6 * units::metre) == target);
    const auto part = geo::move_towards(start, target, 50.0 * units::metre);
    CHECK(units::in_metres(geo::haversine(start, part)) == Approx(50.0).epsilon(1e-3));
}

TEST_CASE("signed_angle wraps to -180..180") {
    const auto [raw, wrapped] =
        GENERATE(table<double, double>({{0, 0}, {190, -170}, {-190, 170}, {540, -180}}));
    CHECK(geo::signed_angle(raw) == Approx(wrapped));
}

namespace {

struct TestAircraft {
    double latitude = 47.0;
    double longitude = 11.0;
    double height = 50.0;
    double attitude_head = 0.0;
    double gimbal_pitch = -90.0;
    double gimbal_yaw = 0.0;
    double zoom_factor = 7.0;
    double ir_zoom_factor = 1.0;

    [[nodiscard]] double camera_heading() const {
        return num::floor_mod(attitude_head + gimbal_yaw, 360.0);
    }
};
static_assert(gimbal::Aircraft<TestAircraft>);

}  // namespace

TEST_CASE("tilt is clamped") {
    TestAircraft a;
    gimbal::tilt(a, 80.0);
    CHECK(a.gimbal_pitch == gimbal::pitch_max);
    gimbal::tilt(a, -120.0);
    CHECK(a.gimbal_pitch == gimbal::pitch_min);
}

TEST_CASE("turn locked moves the body, unlocked moves the gimbal") {
    TestAircraft a;
    gimbal::turn(a, 200.0, true);
    CHECK(a.attitude_head == 200.0);
    CHECK(a.gimbal_yaw == 0.0);
    gimbal::turn(a, 200.0, false);
    CHECK(a.gimbal_yaw == Approx(-160.0));
}

TEST_CASE("drag integrates the speed") {
    TestAircraft a{.gimbal_pitch = 0.0};
    gimbal::drag(a, 0.1, 0.0, false, 0.5);
    CHECK(a.gimbal_pitch == Approx(2.8648).margin(1e-3));
}

TEST_CASE("aim at the centre changes nothing") {
    TestAircraft a{.gimbal_pitch = -30.0};
    gimbal::aim(a, 0.5, 0.5, false, 84.0, {1280, 720});
    CHECK(a.gimbal_pitch == -30.0);
    CHECK(a.gimbal_yaw == 0.0);
}

TEST_CASE("look_at points east and down") {
    TestAircraft a;
    gimbal::look_at(a, 500.0, 47.0, 11.001, 500.0, false);
    CHECK(a.camera_heading() == Approx(90.0).margin(0.1));
    CHECK(a.gimbal_pitch < 0);
}

TEST_CASE("gimbal reset modes") {
    const auto [mode, pitch, yaw] = GENERATE(table<int, double, double>(
        {{0, 0.0, 0.0}, {1, -90.0, 0.0}, {2, -45.0, 0.0}, {3, -90.0, 30.0}}));
    TestAircraft a{.gimbal_pitch = -45.0, .gimbal_yaw = 30.0};
    gimbal::reset(a, mode);
    CHECK(a.gimbal_pitch == pitch);
    CHECK(a.gimbal_yaw == yaw);
}

TEST_CASE("zoom narrows the field of view") {
    TestAircraft a{.zoom_factor = 2.0};
    CHECK(gimbal::lens_dfov(a, gimbal::Lens::wide, 84.0) == 84.0);
    CHECK(gimbal::lens_dfov(a, gimbal::Lens::zoom, 84.0) < 84.0 / 1.5);
    gimbal::zoom(a, 500.0, gimbal::Lens::zoom);
    CHECK(a.zoom_factor == gimbal::zoom_max);
}

TEST_CASE("catalog keys") {
    CHECK(catalog::model_key("dock2") == "3-2-0");
    CHECK(catalog::model_key("m3d") == "0-91-0");
    CHECK(catalog::camera_index("unknown") == "80-0-0");
    CHECK(catalog::payload_model_key("165-0-7") == "1-165-0");
    CHECK(catalog::switchable_video_types("m3td").size() == 4);
}

TEST_CASE("codes serialize as plain values") {
    const Json payload = {{"result", code::Result::param_error},
                          {"status", code::FlighttaskStatus::canceled}};
    CHECK(payload.dump() == R"({"result":514171,"status":"canceled"})");
}

TEST_CASE("envelope ids and reply") {
    static const std::regex uuid4{
        "[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}"};
    CHECK(std::regex_match(envelope::new_id(), uuid4));

    const Json msg = envelope::make({{"a", 1}}, "update_topo");
    CHECK(msg.contains("tid"));
    CHECK(msg["method"] == "update_topo");

    const Json request = {{"tid", "t"}, {"bid", "b"}, {"method", "cover_open"}};
    const Json answer = envelope::reply(request, envelope::ok_output(), "SN1");
    CHECK(answer["tid"] == "t");
    CHECK(answer["bid"] == "b");
    CHECK(answer["gateway"] == "SN1");
    CHECK(answer["data"] == Json{{"result", 0}, {"output", Json::object()}});
    CHECK(answer.begin().key() == "tid");
}

TEST_CASE("base64 round trip and lenient decode") {
    const std::vector<std::uint8_t> data{'h', 'e', 'l', 'l', 'o'};
    CHECK(base64::encode(data) == "aGVsbG8=");
    CHECK(base64::decode_lenient("aGV s\nbG8=") == data);
    CHECK_FALSE(base64::decode_lenient("a"));
}

namespace {

// A throwaway RSA key pair. The license signer is the private half.
struct TestKey {
    std::unique_ptr<EVP_PKEY, decltype(&EVP_PKEY_free)> key{EVP_RSA_gen(1024), &EVP_PKEY_free};

    [[nodiscard]] std::string public_pem() const {
        const std::unique_ptr<BIO, decltype(&BIO_free)> bio(BIO_new(BIO_s_mem()), &BIO_free);
        REQUIRE(PEM_write_bio_PUBKEY(bio.get(), key.get()) == 1);
        char* data = nullptr;
        const auto size = BIO_get_mem_data(bio.get(), &data);
        return {data, static_cast<std::size_t>(size)};
    }

    [[nodiscard]] std::string sign(std::string_view message) const {
        const std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(),
                                                                          &EVP_MD_CTX_free);
        REQUIRE(EVP_DigestSignInit(ctx.get(), nullptr, EVP_sha256(), nullptr, key.get()) == 1);
        std::size_t size = 0;
        const auto* bytes = reinterpret_cast<const unsigned char*>(message.data());
        REQUIRE(EVP_DigestSign(ctx.get(), nullptr, &size, bytes, message.size()) == 1);
        std::vector<std::uint8_t> signature(size);
        REQUIRE(EVP_DigestSign(ctx.get(), signature.data(), &size, bytes, message.size()) == 1);
        signature.resize(size);
        return base64::encode(signature);
    }
};

}  // namespace

TEST_CASE("license accepts a signature of app_id + app_key with the configured key") {
    const TestKey signer;
    const auto pem = signer.public_pem();
    const auto lic = signer.sign("126988key");
    CHECK_FALSE(license::key_error(pem));
    CHECK(license::verify(pem, "126988", "key", lic));
    CHECK_FALSE(license::verify(pem, "126988", "other", lic));
    CHECK_FALSE(license::verify(TestKey{}.public_pem(), "126988", "key", lic));
}

TEST_CASE("license rejects a wrong signature, non-base64 text and a bad key") {
    const auto pem = TestKey{}.public_pem();
    CHECK_FALSE(license::verify(pem, "126988", "key", "AAAA"));
    CHECK_FALSE(license::verify(pem, "126988", "key", "not base64 !!"));
    CHECK_FALSE(license::verify("not a key", "126988", "key", "AAAA"));
    CHECK(license::key_error("not a key") == "not a PEM public key");
}

TEST_CASE("redact hides stream tokens") {
    CHECK(redact::safe_url("https://host/mtx/100-SIM-165-0-7-normal/whip?streamToken=d9b0a0a4") ==
          "https://host/mtx/100-SIM-165-0-7-normal/whip?streamToken=***");
}

TEST_CASE("redact keeps harmless S3 keys") {
    const auto out = redact::safe_url(
        "https://storage/user100/w.kmz?X-Amz-Algorithm=AWS4-HMAC-SHA256"
        "&X-Amz-Credential=minioadmin%2F20261016&X-Amz-Expires=86400&X-Amz-Signature=7734ea");
    CHECK(out.find("minioadmin") == std::string::npos);
    CHECK(out.find("7734ea") == std::string::npos);
    CHECK(out.find("X-Amz-Expires=86400") != std::string::npos);
    CHECK(out.find("X-Amz-Algorithm=AWS4-HMAC-SHA256") != std::string::npos);
}

TEST_CASE("redact leaves a URL without query unchanged") {
    CHECK(redact::safe_url("rtmp://host/live/stream") == "rtmp://host/live/stream");
}

TEST_CASE("redact hides the password of a URL") {
    CHECK(redact::safe_url("rtmp://user:secret@host/live/stream?streamToken=t") ==
          "rtmp://user:***@host/live/stream?streamToken=***");
    CHECK(redact::safe_url("rtsp://host:8554/live") == "rtsp://host:8554/live");
    CHECK(redact::safe_url("https://h/x?a=1#streamToken=t").find("streamToken=t") ==
          std::string::npos);
}

TEST_CASE("parse_time reads ISO 8601 with T or space, seconds, fractions and offsets") {
    using namespace std::chrono;
    const auto at = [](int h, int m, int s) {
        return sim::WallTime{sys_days{year{2026} / 6 / 21} + hours{h} + minutes{m} + seconds{s}};
    };
    CHECK(render::parse_time("2026-06-21T10:00") == at(10, 0, 0));
    CHECK(render::parse_time("2026-06-21 10:00:30") == at(10, 0, 30));
    CHECK(render::parse_time("2026-06-21T10:00:30.987654Z") == at(10, 0, 30));
    CHECK(render::parse_time("2026-06-21T10:00:00+02:00") == at(8, 0, 0));
    CHECK(render::parse_time("2026-06-21T10:00:00-0530") == at(15, 30, 0));
    CHECK(render::parse_time("2026-06-21T10:00Z") == at(10, 0, 0));
    CHECK_FALSE(render::parse_time("2026-02-30T10:00"));
    CHECK_FALSE(render::parse_time("2026-06-21T25:00"));
    CHECK_FALSE(render::parse_time("2026-06-21"));
    CHECK_FALSE(render::parse_time("yesterday"));
}
