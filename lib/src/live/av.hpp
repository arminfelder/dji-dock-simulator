#pragma once

#include "image/image.hpp"

#include <docksim/config.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>

struct AVCodecContext;
struct AVPacket;

// Video encoding and muxing on the statically linked FFmpeg libraries:
// H.264 through openh264 (or NVENC), AAC for the audio track, FLV/RTSP out.
namespace docksim::live {

class AvError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Text of an FFmpeg error code.
[[nodiscard]] std::string av_error(int code);

// Quiet FFmpeg's own log down to warnings. Called once by the constructors below.
void init_av_logging();

using PacketSink = std::function<void(AVPacket* packet)>;

// RGB frames to H.264, constrained baseline, no B-frames, a keyframe every gop
// frames. NVENC is used if asked for and the driver loads; else openh264.
class H264Encoder {
public:
    struct Settings {
        int width;
        int height;
        int fps;
        std::int64_t bitrate;
        VideoEncoder wanted = VideoEncoder::software;
        // SPS/PPS in the codec extradata (FLV/MP4) instead of in-band (WebRTC, RTSP is fine either
        // way).
        bool global_header = false;
    };

    explicit H264Encoder(const Settings& settings);
    H264Encoder(const H264Encoder&) = delete;
    H264Encoder& operator=(const H264Encoder&) = delete;
    H264Encoder(H264Encoder&&) = delete;
    H264Encoder& operator=(H264Encoder&&) = delete;
    ~H264Encoder();

    // pts counts frames. Every packet the encoder produces goes to sink.
    // keyframe forces an IDR frame, for example after a receiver reports loss.
    void encode(const image::RgbImage& frame, std::int64_t pts, const PacketSink& sink,
                bool keyframe = false);
    void flush(const PacketSink& sink);

    [[nodiscard]] AVCodecContext* context() const noexcept;
    // libopenh264 or h264_nvenc.
    [[nodiscard]] const std::string& codec_name() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// One push to a media server or a file: H.264 video plus a quiet 440 Hz AAC
// tone, as the ffmpeg command of the Python version sent (some servers need audio).
class MuxPush {
public:
    struct Settings {
        std::string url;
        // flv, rtsp, mp4.
        std::string muxer;
        H264Encoder::Settings video;
        bool audio = true;
    };

    explicit MuxPush(const Settings& settings);
    MuxPush(const MuxPush&) = delete;
    MuxPush& operator=(const MuxPush&) = delete;
    MuxPush(MuxPush&&) = delete;
    MuxPush& operator=(MuxPush&&) = delete;
    // Flushes the encoders and writes the trailer.
    ~MuxPush();

    // The next video frame; audio fills up to the same time. Throws AvError
    // when the connection fails.
    void push(const image::RgbImage& frame);
    // Flush and write the trailer now; the destructor then does nothing.
    void finish();

    [[nodiscard]] std::int64_t frames() const noexcept;
    [[nodiscard]] const std::string& codec_name() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace docksim::live
