#include "live/whip.hpp"

#include "cloud/redact.hpp"
#include "live/av.hpp"
#include "log/log.hpp"
#include "net/http.hpp"
#include "net/url.hpp"
#include "util/json.hpp"

#include <rtc/rtc.hpp>

extern "C" {
#include <libavcodec/packet.h>
}

#include <atomic>
#include <condition_variable>
#include <format>
#include <random>
#include <sstream>
#include <thread>

namespace docksim::live {

namespace {

using Clock = std::chrono::steady_clock;

const auto& logger() {
    static const auto log = log::get("dock_sim.whip");
    return log;
}

// A gateway in front of an SRS build without the WHIP handler answers like this.
bool legacy_fallback(long status) {
    return status == 404 || status == 405 || status == 501 || status == 502;
}

bool redirect(long status) {
    return status == 301 || status == 302 || status == 303 || status == 307 || status == 308;
}

}  // namespace

HttpCall curl_http() {
    return [](http::Method method, const std::string& url, const std::string& body,
              const std::vector<std::pair<std::string, std::string>>& headers) {
        const auto r = http::perform(
            {.method = method,
             .url = url,
             .headers = headers,
             .body = body,
             .timeout = std::chrono::duration_cast<std::chrono::milliseconds>(whip_timeout),
             .follow_redirects = false});
        return HttpReply{.status = r.status, .body = r.body, .headers = r.headers};
    };
}

std::string rewrite_candidates(const std::string& sdp, const std::optional<std::string>& host,
                               std::optional<int> port) {
    std::istringstream in(sdp);
    std::string out;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.starts_with("a=candidate:")) {
            std::vector<std::string> fields;
            std::istringstream words(line);
            for (std::string w; words >> w;) fields.push_back(w);
            if (fields.size() < 6 || fields[4] == "default") continue;
            if (host && !host->empty()) fields[4] = *host;
            if (port) fields[5] = std::to_string(*port);
            line.clear();
            for (const auto& f : fields) line += (line.empty() ? "" : " ") + f;
        }
        out += line + "\r\n";
    }
    return out;
}

std::pair<std::string, std::string> srs_publish_urls(const std::string& whip_url) {
    const auto parsed = url::Url::parse(whip_url);
    if (!parsed) throw std::invalid_argument("not a WHIP URL: " + redact::safe_url(whip_url));
    auto app = parsed->query_param("app").value_or("");
    if (app.empty()) app = "live";
    const auto stream = parsed->query_param("stream").value_or("");
    const auto token = parsed->query_param("streamToken").value_or("");

    auto api = *parsed;
    auto path = api.get(url::Part::path).value_or("/");
    if (const auto at = path.find("/whip"); at != std::string::npos)
        path.replace(at, 5, "/publish");
    api.set(url::Part::path, path);
    api.set(url::Part::query, std::nullopt);
    api.set(url::Part::fragment, std::nullopt);

    // The stream URL goes into the request body and the log, so it carries no credentials.
    const auto port = parsed->get(url::Part::port);
    const auto netloc = parsed->get(url::Part::host).value_or("") + (port ? ":" + *port : "");
    std::string stream_url = std::format("webrtc://{}/{}/{}", netloc, app, stream);
    if (!token.empty()) stream_url += "?streamToken=" + token;
    return {api.href(), stream_url};
}

std::string join_url(const std::string& base, const std::string& location) {
    const auto parsed = url::Url::parse(base);
    if (!parsed) return location;
    const auto resolved = parsed->resolve(location);
    return resolved ? resolved->href() : location;
}

std::vector<std::byte> without_sei(std::span<const std::byte> annex_b) {
    constexpr std::uint8_t nal_sei = 6;
    const auto at = [&](std::size_t i) { return std::to_integer<std::uint8_t>(annex_b[i]); };
    // A unit starts at its start code; a zero byte before 00 00 01 makes the 4-byte form.
    std::vector<std::size_t> units;
    std::vector<std::size_t> headers;
    for (std::size_t i = 0; i + 3 < annex_b.size(); ++i) {
        if (at(i) == 0 && at(i + 1) == 0 && at(i + 2) == 1) {
            units.push_back(i > 0 && at(i - 1) == 0 ? i - 1 : i);
            headers.push_back(i + 3);
            i += 2;
        }
    }
    if (units.empty()) return {annex_b.begin(), annex_b.end()};
    const auto from = [&](std::size_t i) {
        return annex_b.begin() + static_cast<std::ptrdiff_t>(i);
    };
    std::vector<std::byte> out(annex_b.begin(), from(units.front()));
    for (std::size_t k = 0; k < units.size(); ++k) {
        if ((at(headers[k]) & 0x1FU) == nal_sei) continue;
        out.insert(out.end(), from(units[k]),
                   k + 1 < units.size() ? from(units[k + 1]) : annex_b.end());
    }
    return out;
}

struct WhipSession::Impl {
    Settings settings;
    FrameSource frames;
    HttpCall http;
    std::shared_ptr<rtc::PeerConnection> pc;
    std::shared_ptr<rtc::Track> track;
    std::optional<std::string> resource;
    std::mutex mutex;
    std::condition_variable changed;
    bool gathered = false;
    bool stopping = false;
    std::atomic<bool> connected = false;
    bool given_up = false;
    std::atomic<std::int64_t> sent = 0;
    std::atomic<bool> keyframe_wanted = false;
    std::thread sender;

    HttpReply post(std::string url, const std::string& body,
                   const std::vector<std::pair<std::string, std::string>>& headers) {
        // Repeat the POST on a redirect; a client that turns it into a GET loses the offer.
        for (int i = 0; i < 3; ++i) {
            auto reply = http(http::Method::post, url, body, headers);
            if (!redirect(reply.status)) return reply;
            const auto it = reply.headers.find("location");
            if (it == reply.headers.end()) return reply;
            url = join_url(url, it->second);
            logger()->debug("whip {}: redirected to {}", settings.video_id, redact::safe_url(url));
        }
        throw std::runtime_error("too many redirects");
    }

    std::string exchange_srs(const std::string& offer) {
        const auto [api, stream_url] = srs_publish_urls(settings.url);
        const Json body = {{"api", api}, {"streamurl", stream_url}, {"sdp", offer}};
        const auto reply = post(api, body.dump(), {{"Content-Type", "application/json"}});
        if (reply.status != 200) {
            throw std::runtime_error(
                std::format("SRS publish {}: {}", reply.status, reply.body.substr(0, 200)));
        }
        const auto payload = Json::parse(reply.body, nullptr, false);
        if (!payload.is_object() || payload.value("code", -1) != 0 ||
            payload.value("sdp", "").empty()) {
            throw std::runtime_error("SRS publish rejected: " + reply.body.substr(0, 200));
        }
        logger()->info("whip {} signalled through {}", settings.video_id,
                       redact::safe_url(stream_url));
        return payload["sdp"].get<std::string>();
    }

    std::string exchange(const std::string& offer) {
        const auto reply =
            post(settings.url, offer,
                 {{"Content-Type", "application/sdp"}, {"Accept", "application/sdp"}});
        if (legacy_fallback(reply.status)) {
            logger()->warn("whip {}: WHIP answered {}, trying the SRS publish API",
                           settings.video_id, reply.status);
            return exchange_srs(offer);
        }
        if (reply.status != 200 && reply.status != 201) {
            throw std::runtime_error(
                std::format("WHIP POST {}: {}", reply.status, reply.body.substr(0, 200)));
        }
        if (const auto it = reply.headers.find("location"); it != reply.headers.end()) {
            resource = join_url(settings.url, it->second);
        }
        return reply.body;
    }

    // Sends the newest frame at the stream rate, on a 90 kHz RTP clock.
    void send_loop() {
        H264Encoder encoder({.width = settings.width,
                             .height = settings.height,
                             .fps = settings.fps,
                             .bitrate = settings.bitrate,
                             .wanted = settings.encoder,
                             .global_header = false});
        logger()->info("whip {}: encoding with {}", settings.video_id, encoder.codec_name());
        const auto start = Clock::now();
        for (std::int64_t n = 0;; ++n) {
            {
                std::unique_lock lock(mutex);
                const auto due =
                    start +
                    std::chrono::duration_cast<Clock::duration>(
                        std::chrono::duration<double>(static_cast<double>(n) / settings.fps));
                if (changed.wait_until(lock, due, [this] { return stopping; })) return;
            }
            if (!track || !track->isOpen()) continue;
            const auto frame = frames();
            if (!frame || frame->width() != settings.width || frame->height() != settings.height)
                continue;
            const std::chrono::duration<double> when(static_cast<double>(n) / settings.fps);
            try {
                encoder.encode(
                    *frame, n,
                    [&](AVPacket* p) {
                        const auto unit = without_sei({reinterpret_cast<const std::byte*>(p->data),
                                                       static_cast<std::size_t>(p->size)});
                        track->sendFrame(unit.data(), unit.size(), rtc::FrameInfo(when));
                    },
                    keyframe_wanted.exchange(false));
                if (sent.fetch_add(1) == 0) {
                    const std::scoped_lock lock(mutex);
                    changed.notify_all();
                }
            } catch (const std::exception& e) {
                logger()->warn("whip {}: send failed: {}", settings.video_id, e.what());
            }
        }
    }
};

WhipSession::WhipSession(Settings settings, FrameSource frames, HttpCall http)
    : impl_(std::make_unique<Impl>()) {
    impl_->settings = std::move(settings);
    impl_->frames = std::move(frames);
    impl_->http = std::move(http);
}

WhipSession::~WhipSession() {
    stop();
}

void WhipSession::start() {
    auto& m = *impl_;
    rtc::InitLogger(rtc::LogLevel::Warning);
    rtc::Configuration config;
    config.bindAddress = m.settings.bind_address;
    m.pc = std::make_shared<rtc::PeerConnection>(config);
    m.pc->onGatheringStateChange([&m](rtc::PeerConnection::GatheringState state) {
        if (state != rtc::PeerConnection::GatheringState::Complete) return;
        const std::scoped_lock lock(m.mutex);
        m.gathered = true;
        m.changed.notify_all();
    });
    m.pc->onStateChange([&m](rtc::PeerConnection::State state) {
        {
            const std::scoped_lock lock(m.mutex);
            m.connected = state == rtc::PeerConnection::State::Connected;
            m.given_up = state == rtc::PeerConnection::State::Failed ||
                         state == rtc::PeerConnection::State::Closed;
        }
        m.changed.notify_all();
        logger()->info("whip {} connection {}", m.settings.video_id, static_cast<int>(state));
    });

    std::random_device rd;
    const rtc::SSRC ssrc = rd();
    rtc::Description::Video media("video", rtc::Description::Direction::SendOnly);
    media.addH264Codec(96);
    media.addSSRC(ssrc, "dock-sim");
    m.track = m.pc->addTrack(media);
    auto rtp =
        std::make_shared<rtc::RtpPacketizationConfig>(ssrc, "dock-sim", 96, video_clock_rate);
    auto packetizer =
        std::make_shared<rtc::H264RtpPacketizer>(rtc::NalUnit::Separator::StartSequence, rtp);
    packetizer->addToChain(std::make_shared<rtc::RtcpSrReporter>(rtp));
    packetizer->addToChain(std::make_shared<rtc::RtcpNackResponder>());
    packetizer->addToChain(std::make_shared<rtc::PliHandler>([&m] {
        // MediaMTX asks every 2 s, so this is no sign of loss.
        logger()->debug("whip {}: receiver asked for a keyframe", m.settings.video_id);
        m.keyframe_wanted = true;
    }));
    m.track->setMediaHandler(packetizer);

    m.pc->setLocalDescription();
    {
        std::unique_lock lock(m.mutex);
        if (!m.changed.wait_for(lock, whip_timeout, [&m] { return m.gathered; })) {
            throw std::runtime_error("ICE gathering did not finish");
        }
    }
    const auto description = m.pc->localDescription();
    if (!description) throw std::runtime_error("no local description after ICE gathering");
    const auto offer = std::string(*description);
    auto answer = m.exchange(offer);
    answer = rewrite_candidates(answer, m.settings.candidate_host, m.settings.candidate_port);
    m.pc->setRemoteDescription(rtc::Description(answer, "answer"));
    logger()->info("whip {} published to {}", m.settings.video_id,
                   redact::safe_url(m.settings.url));
    m.sender = std::thread([&m] { m.send_loop(); });
    // MediaMTX makes the path ready only after the first RTP packets. A reader that comes
    // before that gets no stream, and the web UI player does not try again.
    std::unique_lock lock(m.mutex);
    if (!m.changed.wait_for(lock, whip_ready_timeout, [&m] {
            return m.stopping || m.given_up || (m.connected && m.sent > 0);
        })) {
        logger()->warn("whip {}: no media after {} s, answering anyway", m.settings.video_id,
                       whip_ready_timeout.count());
    }
}

void WhipSession::stop() {
    auto& m = *impl_;
    {
        const std::scoped_lock lock(m.mutex);
        if (m.stopping) return;
        m.stopping = true;
    }
    m.changed.notify_all();
    if (m.sender.joinable()) m.sender.join();
    if (m.resource) {
        try {
            const auto reply = m.http(http::Method::del, *m.resource, "", {});
            if (reply.status / 100 != 2) {
                logger()->warn("whip {}: DELETE {} answered {}", m.settings.video_id,
                               redact::safe_url(*m.resource), reply.status);
            }
        } catch (const std::exception& e) {
            logger()->warn("whip {}: DELETE failed: {}", m.settings.video_id, e.what());
        }
    }
    // The callbacks and the PLI handler capture this session. Remove them before the close, so
    // that a late callback from a libdatachannel thread cannot reach a destroyed session.
    if (m.track) {
        m.track->resetCallbacks();
        m.track->setMediaHandler(nullptr);
    }
    if (m.pc) {
        m.pc->resetCallbacks();
        m.pc->close();
    }
    m.track.reset();
    m.pc.reset();
}

bool WhipSession::connected() const {
    return impl_->connected;
}

std::int64_t WhipSession::frames_sent() const {
    return impl_->sent;
}

const std::optional<std::string>& WhipSession::resource() const {
    return impl_->resource;
}

}  // namespace docksim::live
