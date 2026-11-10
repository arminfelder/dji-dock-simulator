#include "live/av.hpp"
#include "live/feed.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <random>

using namespace docksim;

namespace {

std::filesystem::path temp_file(const char* suffix) {
    std::random_device rd;
    return std::filesystem::temp_directory_path() /
           std::format("docksim-live-{:08x}{}", rd(), suffix);
}

struct Probe {
    int streams = 0;
    std::string video_codec;
    int width = 0;
    int height = 0;
    bool audio = false;
    int video_packets = 0;
    int keyframes = 0;
};

Probe probe(const std::filesystem::path& path) {
    AVFormatContext* fmt = nullptr;
    REQUIRE(avformat_open_input(&fmt, path.c_str(), nullptr, nullptr) == 0);
    REQUIRE(avformat_find_stream_info(fmt, nullptr) >= 0);
    Probe p;
    p.streams = static_cast<int>(fmt->nb_streams);
    int video = -1;
    for (unsigned i = 0; i < fmt->nb_streams; ++i) {
        const auto* par = fmt->streams[i]->codecpar;
        if (par->codec_type == AVMEDIA_TYPE_VIDEO) {
            video = static_cast<int>(i);
            p.video_codec = avcodec_get_name(par->codec_id);
            p.width = par->width;
            p.height = par->height;
        }
        if (par->codec_type == AVMEDIA_TYPE_AUDIO && par->codec_id == AV_CODEC_ID_AAC)
            p.audio = true;
    }
    AVPacket* packet = av_packet_alloc();
    while (av_read_frame(fmt, packet) >= 0) {
        if (packet->stream_index == video) {
            ++p.video_packets;
            if ((packet->flags & AV_PKT_FLAG_KEY) != 0) ++p.keyframes;
        }
        av_packet_unref(packet);
    }
    av_packet_free(&packet);
    avformat_close_input(&fmt);
    return p;
}

}  // namespace

TEST_CASE("rendered frames encode to H.264 and AAC in FLV", "[live]") {
    const auto path = temp_file(".flv");
    {
        live::MuxPush push({.url = path.string(),
                            .muxer = "flv",
                            .video = {.width = 320, .height = 240, .fps = 25, .bitrate = 800'000},
                            .audio = true});
        CHECK(push.codec_name() == "libopenh264");
        for (int i = 0; i < 60; ++i) push.push(live::pattern(320, 240, i, "test"));
    }
    const auto p = probe(path);
    std::filesystem::remove(path);
    CHECK(p.streams == 2);
    CHECK(p.video_codec == "h264");
    CHECK(p.width == 320);
    CHECK(p.height == 240);
    CHECK(p.audio);
    CHECK(p.video_packets == 60);
    // A keyframe every 2 * fps frames: 0 and 50.
    CHECK(p.keyframes >= 2);
}

TEST_CASE("NVENC falls back to openh264 without an NVIDIA driver", "[live]") {
    live::H264Encoder encoder(
        {.width = 64, .height = 48, .fps = 25, .bitrate = 200'000, .wanted = VideoEncoder::nvenc});
    // In the sandbox there is no NVIDIA driver; on a GPU machine NVENC is fine too.
    CHECK((encoder.codec_name() == "libopenh264" || encoder.codec_name() == "h264_nvenc"));
    int packets = 0;
    for (int i = 0; i < 5; ++i)
        encoder.encode(live::pattern(64, 48, i, ""), i, [&](AVPacket*) { ++packets; });
    encoder.flush([&](AVPacket*) { ++packets; });
    CHECK(packets == 5);
}

TEST_CASE("a requested keyframe is an IDR frame mid-GOP", "[live]") {
    live::H264Encoder encoder({.width = 64, .height = 48, .fps = 25, .bitrate = 200'000});
    std::vector<int> keyframes;
    for (int i = 0; i < 10; ++i) {
        encoder.encode(
            live::pattern(64, 48, i, ""), i,
            [&](AVPacket* p) {
                if ((p->flags & AV_PKT_FLAG_KEY) != 0) keyframes.push_back(i);
            },
            i == 7);
    }
    CHECK(keyframes == std::vector<int>{0, 7});
}

TEST_CASE("an odd frame size is refused", "[live]") {
    CHECK_THROWS_AS(live::H264Encoder({.width = 65, .height = 48, .fps = 25, .bitrate = 200'000}),
                    live::AvError);
}
