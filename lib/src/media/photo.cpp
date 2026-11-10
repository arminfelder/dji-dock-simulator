#include "media/photo.hpp"

#include "device/geo.hpp"

#include <exiv2/exiv2.hpp>

#include <cmath>
#include <cstdint>
#include <format>
#include <mutex>
#include <string_view>

namespace docksim::media {

namespace {

constexpr std::string_view dji_namespace = "http://www.dji.com/drone-dji/1.0/";

// Exiv2 XMP uses process-wide state, so photos get their metadata one at a time.
std::mutex& exiv2_mutex() {
    static std::mutex mutex;
    return mutex;
}

void register_dji_namespace() {
    static std::once_flag once;
    std::call_once(once, [] {
        Exiv2::XmpParser::initialize();
        Exiv2::XmpProperties::registerNs(std::string(dji_namespace), "drone-dji");
    });
}

// Degrees, minutes and seconds to 1/10000 s. Counting in integers carries 60 s into the minutes.
Exiv2::URationalValue dms(double value) {
    constexpr std::int64_t per_second = 10'000;
    const auto total = std::llround(std::abs(value) * 3600.0 * per_second);
    const auto degrees = total / (3600 * per_second);
    const auto minutes = total / (60 * per_second) % 60;
    const auto seconds = total % (60 * per_second);
    Exiv2::URationalValue out;
    out.value_ = {{static_cast<std::uint32_t>(degrees), 1},
                  {static_cast<std::uint32_t>(minutes), 1},
                  {static_cast<std::uint32_t>(seconds), static_cast<std::uint32_t>(per_second)}};
    return out;
}

Exiv2::ExifData exif_of(const PhotoMeta& meta) {
    const auto stamp =
        std::format("{:%Y:%m:%d %H:%M:%S}", std::chrono::floor<std::chrono::seconds>(meta.time));
    Exiv2::ExifData exif;
    exif["Exif.Image.Make"] = "DJI";
    exif["Exif.Image.Model"] = meta.model;
    exif["Exif.Image.DateTime"] = stamp;
    exif["Exif.Photo.DateTimeOriginal"] = stamp;
    exif["Exif.Photo.DateTimeDigitized"] = stamp;

    Exiv2::DataValue version(Exiv2::unsignedByte);
    version.read("2 3 0 0");
    exif["Exif.GPSInfo.GPSVersionID"] = version;
    exif["Exif.GPSInfo.GPSLatitudeRef"] = meta.latitude >= 0 ? "N" : "S";
    exif["Exif.GPSInfo.GPSLatitude"] = dms(meta.latitude);
    exif["Exif.GPSInfo.GPSLongitudeRef"] = meta.longitude >= 0 ? "E" : "W";
    exif["Exif.GPSInfo.GPSLongitude"] = dms(meta.longitude);
    Exiv2::DataValue below_sea(Exiv2::unsignedByte);
    below_sea.read(meta.absolute_altitude < 0 ? "1" : "0");
    exif["Exif.GPSInfo.GPSAltitudeRef"] = below_sea;
    Exiv2::URationalValue altitude;
    altitude.value_ = {
        {static_cast<std::uint32_t>(std::llround(std::abs(meta.absolute_altitude) * 1000.0)),
         1000}};
    exif["Exif.GPSInfo.GPSAltitude"] = altitude;
    return exif;
}

Exiv2::XmpData xmp_of(const PhotoMeta& meta) {
    const double yaw = geo::signed_angle(meta.heading);
    const std::array<std::pair<const char*, std::string>, 10> tags{{
        {"GpsLatitude", std::format("{:.8f}", meta.latitude)},
        {"GpsLongitude", std::format("{:.8f}", meta.longitude)},
        {"AbsoluteAltitude", std::format("{:+.3f}", meta.absolute_altitude)},
        {"RelativeAltitude", std::format("{:+.3f}", meta.relative_altitude)},
        {"GimbalRollDegree", "+0.00"},
        {"GimbalYawDegree", std::format("{:+.2f}", yaw)},
        {"GimbalPitchDegree", std::format("{:+.2f}", meta.gimbal_pitch)},
        {"FlightRollDegree", "+0.00"},
        {"FlightYawDegree", std::format("{:+.2f}", yaw)},
        {"FlightPitchDegree", "+0.00"},
    }};
    Exiv2::XmpData xmp;
    for (const auto& [key, value] : tags) xmp[std::format("Xmp.drone-dji.{}", key)] = value;
    return xmp;
}

}  // namespace

image::RgbImage synthetic_image(std::pair<int, int> size, const std::string& caption,
                                std::uint64_t seed) {
    const auto [w, h] = size;
    image::RgbImage img(w, h, {18, 34, 52});
    std::mt19937_64 rng(seed);
    const auto pick = [&](int lo, int hi) {
        return std::uniform_int_distribution<int>(lo, hi)(rng);
    };
    for (int i = 0; i < 60; ++i) {
        const int x1 = pick(0, w);
        const int y1 = pick(0, h);
        image::rectangle_outline(img, x1, y1, x1 + pick(10, 120), y1 + pick(10, 120),
                                 {90, 140, 190});
    }
    image::draw_text(img, 24, 24, caption, 14.0F, {240, 240, 240});
    return img;
}

std::vector<std::byte> encode_photo(const image::RgbImage& img, const PhotoMeta& meta) {
    const auto jpeg = image::encode_jpeg(img, jpeg_quality);
    const std::scoped_lock lock(exiv2_mutex());
    register_dji_namespace();
    auto photo =
        Exiv2::ImageFactory::open(reinterpret_cast<const Exiv2::byte*>(jpeg.data()), jpeg.size());
    photo->setByteOrder(Exiv2::littleEndian);
    photo->setExifData(exif_of(meta));
    photo->setXmpData(xmp_of(meta));
    photo->writeMetadata();
    auto& io = photo->io();
    io.seek(0, Exiv2::BasicIo::beg);
    const auto data = io.read(io.size());
    const auto* first = reinterpret_cast<const std::byte*>(data.c_data());
    return {first, first + data.size()};
}

}  // namespace docksim::media
