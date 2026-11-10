#include "live/av.hpp"

#include "cloud/redact.hpp"
#include "log/log.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/channel_layout.h>
#include <libavutil/error.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <array>
#include <cmath>
#include <format>
#include <mutex>
#include <numbers>

namespace docksim::live {

namespace {

const auto& logger() {
    static const auto log = log::get("dock_sim.live");
    return log;
}

// Closes the output connection (unless the muxer writes no file) and frees the context.
struct FormatFree {
    void operator()(AVFormatContext* f) const noexcept {
        if ((f->oformat->flags & AVFMT_NOFILE) == 0) avio_closep(&f->pb);
        avformat_free_context(f);
    }
};

struct CodecFree {
    void operator()(AVCodecContext* c) const noexcept { avcodec_free_context(&c); }
};
struct FrameFree {
    void operator()(AVFrame* f) const noexcept { av_frame_free(&f); }
};
struct PacketFree {
    void operator()(AVPacket* p) const noexcept { av_packet_free(&p); }
};
struct SwsFree {
    void operator()(SwsContext* s) const noexcept { sws_freeContext(s); }
};

void check(int code, std::string_view what) {
    if (code < 0) throw AvError(std::format("{}: {}", what, av_error(code)));
}

void drain(AVCodecContext* ctx, AVPacket* packet, const PacketSink& sink) {
    while (true) {
        const int r = avcodec_receive_packet(ctx, packet);
        if (r == AVERROR(EAGAIN) || r == AVERROR_EOF) return;
        check(r, "avcodec_receive_packet");
        sink(packet);
        av_packet_unref(packet);
    }
}

// Opens the encoder; null if it does not load (for example NVENC without a driver).
std::unique_ptr<AVCodecContext, CodecFree> open_video(const H264Encoder::Settings& s,
                                                      const char* name) {
    const AVCodec* codec = avcodec_find_encoder_by_name(name);
    if (codec == nullptr) return nullptr;
    std::unique_ptr<AVCodecContext, CodecFree> ctx(avcodec_alloc_context3(codec));
    if (!ctx) throw AvError("no codec context");
    ctx->width = s.width;
    ctx->height = s.height;
    ctx->time_base = {.num = 1, .den = s.fps};
    ctx->framerate = {.num = s.fps, .den = 1};
    ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    ctx->bit_rate = s.bitrate;
    ctx->gop_size = 2 * s.fps;
    ctx->max_b_frames = 0;
    // openh264 maps an unset profile to constrained baseline; NVENC gets it as an option.
    if (std::string_view(name) == "h264_nvenc") {
        ctx->profile = AV_PROFILE_H264_CONSTRAINED_BASELINE;
        // Without a VBV limit NVENC makes keyframes twice the openh264 size.
        // Such a burst loses packets on WebRTC and the picture breaks up.
        ctx->rc_buffer_size = static_cast<int>(s.bitrate / 4);
        ctx->rc_max_rate = s.bitrate;
    }
    if (s.global_header) ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    AVDictionary* options = nullptr;
    if (std::string_view(name) == "h264_nvenc") {
        av_dict_set(&options, "preset", "p2", 0);
        av_dict_set(&options, "tune", "ll", 0);
        av_dict_set(&options, "profile", "baseline", 0);
        av_dict_set(&options, "zerolatency", "1", 0);
        // A forced keyframe must be an IDR frame, not only an I frame.
        av_dict_set(&options, "forced-idr", "1", 0);
    } else {
        // Every frame must go out: a skipped frame shows as a stall in the player.
        av_dict_set(&options, "allow_skip_frames", "0", 0);
    }
    const int r = avcodec_open2(ctx.get(), codec, &options);
    av_dict_free(&options);
    if (r < 0) {
        logger()->debug("encoder {} does not open: {}", name, av_error(r));
        return nullptr;
    }
    return ctx;
}

}  // namespace

std::string av_error(int code) {
    std::array<char, AV_ERROR_MAX_STRING_SIZE> text{};
    av_strerror(code, text.data(), text.size());
    return text.data();
}

void init_av_logging() {
    static std::once_flag once;
    std::call_once(once, [] { av_log_set_level(AV_LOG_WARNING); });
}

// -- H264Encoder --------------------------------------------------------------

struct H264Encoder::Impl {
    std::unique_ptr<AVCodecContext, CodecFree> ctx;
    std::unique_ptr<AVFrame, FrameFree> frame{av_frame_alloc()};
    std::unique_ptr<AVPacket, PacketFree> packet{av_packet_alloc()};
    std::unique_ptr<SwsContext, SwsFree> sws;
    std::string name;
    int width = 0;
    int height = 0;
};

H264Encoder::H264Encoder(const Settings& s) : impl_(std::make_unique<Impl>()) {
    init_av_logging();
    if (s.width % 2 != 0 || s.height % 2 != 0) throw AvError("H.264 needs an even frame size");
    if (s.wanted == VideoEncoder::nvenc) {
        impl_->ctx = open_video(s, "h264_nvenc");
        if (!impl_->ctx) logger()->warn("NVENC is not available, encoding with openh264");
    }
    if (!impl_->ctx) impl_->ctx = open_video(s, "libopenh264");
    if (!impl_->ctx) throw AvError("no H.264 encoder opens");
    impl_->name = impl_->ctx->codec->name;
    impl_->width = s.width;
    impl_->height = s.height;
    impl_->frame->format = AV_PIX_FMT_YUV420P;
    impl_->frame->width = s.width;
    impl_->frame->height = s.height;
    check(av_frame_get_buffer(impl_->frame.get(), 0), "av_frame_get_buffer");
    impl_->sws.reset(sws_getContext(s.width, s.height, AV_PIX_FMT_RGB24, s.width, s.height,
                                    AV_PIX_FMT_YUV420P, SWS_BILINEAR, nullptr, nullptr, nullptr));
    if (!impl_->sws) throw AvError("sws_getContext failed");
}

H264Encoder::~H264Encoder() = default;

AVCodecContext* H264Encoder::context() const noexcept {
    return impl_->ctx.get();
}

const std::string& H264Encoder::codec_name() const noexcept {
    return impl_->name;
}

void H264Encoder::encode(const image::RgbImage& frame, std::int64_t pts, const PacketSink& sink,
                         bool keyframe) {
    if (frame.width() != impl_->width || frame.height() != impl_->height) {
        throw AvError(std::format("frame is {}x{}, the encoder {}x{}", frame.width(),
                                  frame.height(), impl_->width, impl_->height));
    }
    check(av_frame_make_writable(impl_->frame.get()), "av_frame_make_writable");
    const std::array<const std::uint8_t*, 1> src{frame.bytes().data()};
    const std::array<int, 1> stride{3 * frame.width()};
    sws_scale(impl_->sws.get(), src.data(), stride.data(), 0, frame.height(), impl_->frame->data,
              impl_->frame->linesize);
    impl_->frame->pts = pts;
    impl_->frame->pict_type = keyframe ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_NONE;
    check(avcodec_send_frame(impl_->ctx.get(), impl_->frame.get()), "avcodec_send_frame");
    drain(impl_->ctx.get(), impl_->packet.get(), sink);
}

void H264Encoder::flush(const PacketSink& sink) {
    if (avcodec_send_frame(impl_->ctx.get(), nullptr) < 0) return;
    drain(impl_->ctx.get(), impl_->packet.get(), sink);
}

// -- MuxPush ------------------------------------------------------------------

struct MuxPush::Impl {
    std::unique_ptr<AVFormatContext, FormatFree> fmt;
    std::unique_ptr<H264Encoder> video;
    AVStream* video_stream = nullptr;
    std::unique_ptr<AVCodecContext, CodecFree> audio;
    AVStream* audio_stream = nullptr;
    std::unique_ptr<AVFrame, FrameFree> audio_frame{av_frame_alloc()};
    std::unique_ptr<AVPacket, PacketFree> audio_packet{av_packet_alloc()};
    std::int64_t frames = 0;
    std::int64_t samples = 0;
    int fps = 25;
    bool finished = false;
    std::string url;

    void write(AVPacket* packet, AVCodecContext* ctx, AVStream* stream) {
        av_packet_rescale_ts(packet, ctx->time_base, stream->time_base);
        packet->stream_index = stream->index;
        check(av_interleaved_write_frame(fmt.get(), packet), "write to " + redact::safe_url(url));
    }

    // AAC frames up to the time of the video frames sent so far.
    void fill_audio() {
        if (!audio) return;
        const int rate = audio->sample_rate;
        const int size = audio->frame_size > 0 ? audio->frame_size : 1024;
        while (samples * fps < frames * rate) {
            check(av_frame_make_writable(audio_frame.get()), "av_frame_make_writable");
            auto* out = reinterpret_cast<float*>(audio_frame->data[0]);
            for (int i = 0; i < size; ++i) {
                const double t = static_cast<double>(samples + i) / rate;
                out[i] = static_cast<float>(0.1 * std::sin(2.0 * std::numbers::pi * 440.0 * t));
            }
            audio_frame->pts = samples;
            samples += size;
            check(avcodec_send_frame(audio.get(), audio_frame.get()), "aac send");
            drain(audio.get(), audio_packet.get(),
                  [this](AVPacket* p) { write(p, audio.get(), audio_stream); });
        }
    }
};

MuxPush::MuxPush(const Settings& s) : impl_(std::make_unique<Impl>()) {
    init_av_logging();
    auto& m = *impl_;
    m.url = s.url;
    m.fps = s.video.fps;
    AVFormatContext* fmt = nullptr;
    check(avformat_alloc_output_context2(&fmt, nullptr, s.muxer.c_str(), s.url.c_str()),
          "output " + s.muxer);
    m.fmt.reset(fmt);
    auto video_settings = s.video;
    video_settings.global_header = (m.fmt->oformat->flags & AVFMT_GLOBALHEADER) != 0;
    m.video = std::make_unique<H264Encoder>(video_settings);
    m.video_stream = avformat_new_stream(m.fmt.get(), nullptr);
    if (m.video_stream == nullptr) throw AvError("no video stream");
    check(avcodec_parameters_from_context(m.video_stream->codecpar, m.video->context()),
          "video parameters");
    m.video_stream->time_base = m.video->context()->time_base;

    if (s.audio) {
        const AVCodec* aac = avcodec_find_encoder(AV_CODEC_ID_AAC);
        if (aac == nullptr) throw AvError("no AAC encoder");
        m.audio.reset(avcodec_alloc_context3(aac));
        if (!m.audio) throw AvError("no AAC context");
        m.audio->sample_rate = 44100;
        m.audio->sample_fmt = AV_SAMPLE_FMT_FLTP;
        m.audio->bit_rate = 64000;
        m.audio->time_base = {.num = 1, .den = 44100};
        av_channel_layout_default(&m.audio->ch_layout, 1);
        if ((m.fmt->oformat->flags & AVFMT_GLOBALHEADER) != 0)
            m.audio->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
        check(avcodec_open2(m.audio.get(), aac, nullptr), "open AAC");
        m.audio_stream = avformat_new_stream(m.fmt.get(), nullptr);
        if (m.audio_stream == nullptr) throw AvError("no audio stream");
        check(avcodec_parameters_from_context(m.audio_stream->codecpar, m.audio.get()),
              "audio parameters");
        m.audio_stream->time_base = m.audio->time_base;
        m.audio_frame->format = AV_SAMPLE_FMT_FLTP;
        m.audio_frame->nb_samples = m.audio->frame_size > 0 ? m.audio->frame_size : 1024;
        m.audio_frame->sample_rate = 44100;
        check(av_channel_layout_copy(&m.audio_frame->ch_layout, &m.audio->ch_layout),
              "channel layout");
        check(av_frame_get_buffer(m.audio_frame.get(), 0), "audio buffer");
    }

    AVDictionary* options = nullptr;
    if (s.muxer == "rtsp") av_dict_set(&options, "rtsp_transport", "tcp", 0);
    if ((m.fmt->oformat->flags & AVFMT_NOFILE) == 0) {
        check(avio_open2(&m.fmt->pb, s.url.c_str(), AVIO_FLAG_WRITE, nullptr, &options),
              "open " + redact::safe_url(s.url));
    }
    const int r = avformat_write_header(m.fmt.get(), &options);
    av_dict_free(&options);
    check(r, "connect " + redact::safe_url(s.url));
}

MuxPush::~MuxPush() {
    try {
        finish();
    } catch (const std::exception& e) {
        logger()->warn("closing {}: {}", redact::safe_url(impl_->url), e.what());
    }
}

void MuxPush::push(const image::RgbImage& frame) {
    auto& m = *impl_;
    m.video->encode(frame, m.frames,
                    [&m](AVPacket* p) { m.write(p, m.video->context(), m.video_stream); });
    ++m.frames;
    m.fill_audio();
}

void MuxPush::finish() {
    auto& m = *impl_;
    if (m.finished || m.fmt == nullptr) return;
    m.finished = true;
    m.video->flush([&m](AVPacket* p) { m.write(p, m.video->context(), m.video_stream); });
    if (m.audio && avcodec_send_frame(m.audio.get(), nullptr) >= 0) {
        drain(m.audio.get(), m.audio_packet.get(),
              [&m](AVPacket* p) { m.write(p, m.audio.get(), m.audio_stream); });
    }
    check(av_write_trailer(m.fmt.get()), "trailer");
}

std::int64_t MuxPush::frames() const noexcept {
    return impl_->frames;
}

const std::string& MuxPush::codec_name() const noexcept {
    return impl_->video->codec_name();
}

}  // namespace docksim::live
