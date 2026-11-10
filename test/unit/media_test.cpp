#include "image/image.hpp"
#include "media/photo.hpp"
#include "media/s3.hpp"
#include "support/http_server.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <exiv2/exiv2.hpp>

#include <algorithm>
#include <array>
#include <format>
#include <span>
#include <string>
#include <string_view>
#include <utility>

using namespace docksim;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace {

media::PhotoMeta meta() {
    return {
        .latitude = 47.2635,
        .longitude = 11.401,
        .absolute_altitude = 620.0,
        .relative_altitude = 40.0,
        .heading = 270.0,
        .gimbal_pitch = -90.0,
        .time = std::chrono::sys_seconds{std::chrono::seconds{1'760'000'000}},
        .model = "Mavic 3E Camera",
    };
}

std::string as_text(std::span<const std::byte> bytes) {
    return {reinterpret_cast<const char*>(bytes.data()), bytes.size()};
}

}  // namespace

namespace {

Json storage_output(const std::string& endpoint) {
    return {
        {"bucket", "media"},
        {"object_key_prefix", "/org-1/"},
        {"region", "eu-central-1"},
        {"endpoint", endpoint},
        {"credentials",
         {{"access_key_id", "AKIDEXAMPLE"},
          {"access_key_secret", "secret"},
          {"security_token", "tok"}}},
    };
}

}  // namespace

TEST_CASE("S3 put sends a signed path-style PutObject to the endpoint", "[s3]") {
    const test::Server server([](const test::Seen&, const std::stop_token&) {
        return test::Reply{.status = 200, .headers = {{"ETag", "\"abc\""}}, .body = ""};
    });
    const MediaConfig cfg;
    s3::S3Storage storage(storage_output(server.url("")), cfg);
    const auto key = storage.object_key("DJI_f-1_0001.jpeg");
    CHECK(key == "org-1/DJI_f-1_0001.jpeg");
    const std::string body = "jpeg bytes";
    storage.put(key, std::as_bytes(std::span(body)), "image/jpeg");

    const auto seen = server.seen();
    REQUIRE(seen.size() == 1);
    CHECK(seen[0].method == "PUT");
    CHECK(seen[0].path == "/media/org-1/DJI_f-1_0001.jpeg");
    CHECK(seen[0].body == body);
    CHECK_THAT(seen[0].head, ContainsSubstring("AWS4-HMAC-SHA256 Credential=AKIDEXAMPLE/"));
    CHECK_THAT(seen[0].head, ContainsSubstring("/eu-central-1/s3/aws4_request"));
    CHECK_THAT(seen[0].head, ContainsSubstring("tok"));
    CHECK_THAT(seen[0].head, ContainsSubstring("image/jpeg"));
    CHECK_THAT(seen[0].head, !ContainsSubstring("aws-chunked"));
}

TEST_CASE("S3 put over https refuses an unknown certificate unless verification is off", "[s3]") {
    const test::TestCertificate cert;
    const test::Server server(
        [](const test::Seen&, const std::stop_token&) {
            return test::Reply{.status = 200, .headers = {}, .body = ""};
        },
        &cert);
    const std::string body = "x";
    const MediaConfig strict;
    s3::S3Storage checked(storage_output(server.url("")), strict);
    CHECK_THROWS(checked.put("k", std::as_bytes(std::span(body)), "image/jpeg"));

    MediaConfig relaxed;
    relaxed.verify_tls = false;
    s3::S3Storage unchecked(storage_output(server.url("")), relaxed);
    CHECK_NOTHROW(unchecked.put("k", std::as_bytes(std::span(body)), "image/jpeg"));
    CHECK(server.seen().size() == 1);
}

TEST_CASE("S3 put throws on an error status", "[s3]") {
    const test::Server server([](const test::Seen&, const std::stop_token&) {
        return test::Reply{.status = 403,
                           .headers = {{"Content-Type", "application/xml"}},
                           .body = "<Error><Code>AccessDenied</Code><Message>no</Message></Error>"};
    });
    const MediaConfig cfg;
    s3::S3Storage storage(storage_output(server.url("")), cfg);
    const std::string body = "x";
    CHECK_THROWS_WITH(storage.put("k", std::as_bytes(std::span(body)), "image/jpeg"),
                      ContainsSubstring("403"));
}

TEST_CASE("photo JPEG decodes and carries EXIF and DJI XMP") {
    const auto img = media::synthetic_image({64, 48}, "DJI_f-1_0001.jpeg\nf-1", 7);
    const auto jpeg = media::encode_photo(img, meta());
    const auto decoded = image::decode(jpeg);
    CHECK(decoded.width() == 64);
    CHECK(decoded.height() == 48);

    const auto text = as_text(jpeg);
    CHECK(text.starts_with("\xFF\xD8"));
    CHECK_THAT(text, ContainsSubstring(std::string("Exif\0\0II", 8)));
    CHECK_THAT(text, ContainsSubstring("2025:10:09 08:53:20"));
    CHECK_THAT(text, ContainsSubstring("Mavic 3E Camera"));
    CHECK_THAT(text, ContainsSubstring(R"(drone-dji:GpsLatitude="47.26350000")"));
    CHECK_THAT(text, ContainsSubstring(R"(drone-dji:GimbalYawDegree="-90.00")"));
    CHECK_THAT(text, ContainsSubstring(R"(drone-dji:AbsoluteAltitude="+620.000")"));
}

namespace {

struct PhotoMetadata {
    Exiv2::ExifData exif;
    Exiv2::XmpData xmp;
};

PhotoMetadata read_metadata(std::span<const std::byte> jpeg) {
    Exiv2::XmpProperties::registerNs("http://www.dji.com/drone-dji/1.0/", "drone-dji");
    auto image =
        Exiv2::ImageFactory::open(reinterpret_cast<const Exiv2::byte*>(jpeg.data()), jpeg.size());
    image->readMetadata();
    return {.exif = image->exifData(), .xmp = image->xmpData()};
}

double degrees_of(const Exiv2::Exifdatum& d) {
    REQUIRE(d.count() == 3);
    const double minutes = d.toFloat(1);
    const double seconds = d.toFloat(2);
    CHECK(minutes < 60.0);
    CHECK(seconds < 60.0);
    return d.toFloat(0) + minutes / 60.0 + seconds / 3600.0;
}

}  // namespace

TEST_CASE("photo EXIF and XMP read back with every DJI field") {
    const auto jpeg = media::encode_photo(media::synthetic_image({64, 48}, "x", 7), meta());
    auto [exif, xmp] = read_metadata(jpeg);
    CHECK(exif["Exif.Image.Make"].toString() == "DJI");
    CHECK(exif["Exif.Image.Model"].toString() == "Mavic 3E Camera");
    CHECK(exif["Exif.Image.DateTime"].toString() == "2025:10:09 08:53:20");
    CHECK(exif["Exif.Photo.DateTimeOriginal"].toString() == "2025:10:09 08:53:20");
    CHECK(exif["Exif.Photo.DateTimeDigitized"].toString() == "2025:10:09 08:53:20");
    CHECK(exif["Exif.GPSInfo.GPSVersionID"].toString() == "2 3 0 0");
    CHECK(exif["Exif.GPSInfo.GPSLatitudeRef"].toString() == "N");
    CHECK(degrees_of(exif["Exif.GPSInfo.GPSLatitude"]) == Approx(47.2635).margin(1e-8));
    CHECK(exif["Exif.GPSInfo.GPSLongitudeRef"].toString() == "E");
    CHECK(degrees_of(exif["Exif.GPSInfo.GPSLongitude"]) == Approx(11.401).margin(1e-8));
    CHECK(exif["Exif.GPSInfo.GPSAltitudeRef"].toInt64() == 0);
    CHECK(exif["Exif.GPSInfo.GPSAltitude"].toFloat() == Approx(620.0));

    const std::array<std::pair<std::string_view, std::string_view>, 10> expected{{
        {"GpsLatitude", "47.26350000"},
        {"GpsLongitude", "11.40100000"},
        {"AbsoluteAltitude", "+620.000"},
        {"RelativeAltitude", "+40.000"},
        {"GimbalRollDegree", "+0.00"},
        {"GimbalYawDegree", "-90.00"},
        {"GimbalPitchDegree", "-90.00"},
        {"FlightRollDegree", "+0.00"},
        {"FlightYawDegree", "-90.00"},
        {"FlightPitchDegree", "+0.00"},
    }};
    for (const auto& [key, value] : expected) {
        INFO(key);
        const auto it = xmp.findKey(Exiv2::XmpKey(std::format("Xmp.drone-dji.{}", key)));
        REQUIRE(it != xmp.end());
        CHECK(it->toString() == value);
    }
}

TEST_CASE("photo GPS seconds that round to 60 carry into the minutes") {
    auto m = meta();
    m.latitude = -47.99999999;
    m.longitude = 11.99999999;
    const auto jpeg = media::encode_photo(media::synthetic_image({64, 48}, "x", 7), m);
    auto [exif, xmp] = read_metadata(jpeg);
    CHECK(exif["Exif.GPSInfo.GPSLatitudeRef"].toString() == "S");
    CHECK(degrees_of(exif["Exif.GPSInfo.GPSLatitude"]) == Approx(47.99999999).margin(1e-6));
    CHECK(degrees_of(exif["Exif.GPSInfo.GPSLongitude"]) == Approx(11.99999999).margin(1e-6));
}

TEST_CASE("text drawing changes pixels") {
    image::RgbImage img(80, 30, {0, 0, 0});
    image::draw_text(img, 2, 2, "Hi", 16.0F, {255, 255, 255});
    CHECK(std::ranges::any_of(img.bytes(), [](auto b) { return b > 128; }));
    const auto box = image::measure_text("ab\ncd", 16.0F);
    CHECK(box.width > 0);
    CHECK(box.height > 16);
}
