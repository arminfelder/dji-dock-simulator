#include "live/feed.hpp"
#include "live/whip.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <rtc/rtc.hpp>

#include <atomic>
#include <condition_variable>
#include <thread>

using namespace docksim;
using Catch::Matchers::ContainsSubstring;

TEST_CASE("answer candidates point at the configured address", "[live]") {
    const std::string sdp =
        "v=0\r\na=candidate:1 1 udp 2130706431 10.0.0.5 8000 typ host\r\n"
        "a=candidate:2 1 udp 2130706431 default 8000 typ host\r\na=end\r\n";
    const auto out = live::rewrite_candidates(sdp, "203.0.113.7", 18000);
    CHECK_THAT(out, ContainsSubstring("a=candidate:1 1 udp 2130706431 203.0.113.7 18000 typ host"));
    CHECK(out.find("default") == std::string::npos);
    CHECK_THAT(out, ContainsSubstring("a=end\r\n"));
    CHECK(live::rewrite_candidates(sdp, std::nullopt, std::nullopt).find("10.0.0.5 8000") !=
          std::string::npos);
}

TEST_CASE("SRS publish URLs keep the stream name and token", "[live]") {
    const auto [api, stream] = live::srs_publish_urls(
        "https://media.example/rtc/v1/whip/?app=live&stream=SN-80-0-0&streamToken=T1");
    CHECK(api == "https://media.example/rtc/v1/publish/");
    CHECK(stream == "webrtc://media.example/live/SN-80-0-0?streamToken=T1");
}

TEST_CASE("WHIP Location resolves against the request URL", "[live]") {
    CHECK(live::join_url("https://h/a/whip?x=1", "/res/1") == "https://h/res/1");
    CHECK(live::join_url("https://h/a/whip", "res/1") == "https://h/a/res/1");
    CHECK(live::join_url("https://h/a/whip", "https://o/r") == "https://o/r");
    CHECK(live::join_url("https://h/a/whip?x=1", "?y=2") == "https://h/a/whip?y=2");
    CHECK(live::join_url("https://h/a/whip", "../r") == "https://h/r");
    CHECK(live::join_url("https://h/a/whip", "//cdn.example/r") == "https://cdn.example/r");
    CHECK(live::join_url("https://h:8443/a/whip", "/r") == "https://h:8443/r");
}

TEST_CASE("SRS stream URL carries no credentials and no fragment", "[live]") {
    const auto [api, stream] = live::srs_publish_urls(
        "https://u:pw@media.example:8443/rtc/v1/whip/?app=cam&stream=SN%2F80&streamToken=T1#f");
    CHECK(api == "https://u:pw@media.example:8443/rtc/v1/publish/");
    CHECK(stream == "webrtc://media.example:8443/cam/SN%2F80?streamToken=T1");
}

// The publisher against an in-process answerer, signalled through a fake
// HTTP layer: offer, answer, ICE on loopback, H.264 RTP packets arrive.
TEST_CASE("SEI units leave the access unit, the rest stays byte for byte", "[live]") {
    const auto bytes = [](std::initializer_list<int> v) {
        std::vector<std::byte> out;
        for (const int b : v) out.push_back(static_cast<std::byte>(b));
        return out;
    };
    // SPS, PPS, SEI (4-byte codes), SEI and IDR slice (3-byte codes).
    const auto unit =
        bytes({0,    0,    0, 1, 0x67, 0x42, 0xC0, 0x1F, 0,    0,    0,    1,   0x68, 0xCB,
               0x8F, 0,    0, 0, 1,    0x06, 0x01, 0x04, 0x80, 0,    0,    1,   0x06, 0x00,
               0x07, 0x80, 0, 0, 1,    0x65, 0xB8, 0x04, 0x00, 0x00, 0x03, 0x01});
    CHECK(live::without_sei(unit) ==
          bytes({0,    0,    0, 1, 0x67, 0x42, 0xC0, 0x1F, 0,    0,    0,    1,   0x68,
                 0xCB, 0x8F, 0, 0, 1,    0x65, 0xB8, 0x04, 0x00, 0x00, 0x03, 0x01}));
    const auto plain = bytes({0, 0, 0, 1, 0x41, 0x9A, 0x02});
    CHECK(live::without_sei(plain) == plain);
}

TEST_CASE("WHIP publishes to a local WebRTC answerer", "[live]") {
    std::shared_ptr<rtc::PeerConnection> answerer;
    std::shared_ptr<rtc::Track> incoming;
    std::atomic<int> packets = 0;
    std::atomic<bool> deleted = false;
    std::mutex mutex;
    std::condition_variable gathered;
    bool done = false;

    const live::HttpCall http =
        [&](http::Method method, const std::string& url, const std::string& body,
            const std::vector<std::pair<std::string, std::string>>&) -> live::HttpReply {
        if (method == http::Method::del) {
            deleted = url == "https://whip.test/resource/7";
            return {.status = 200, .body = "", .headers = {}};
        }
        rtc::Configuration config;
        // The sandbox has only loopback, which ICE gathering skips unless bound to it.
        config.bindAddress = "127.0.0.1";
        answerer = std::make_shared<rtc::PeerConnection>(config);
        answerer->onTrack([&](const std::shared_ptr<rtc::Track>& track) {
            incoming = track;
            track->onMessage([&](const rtc::message_variant&) { ++packets; });
        });
        answerer->onGatheringStateChange([&](rtc::PeerConnection::GatheringState s) {
            if (s != rtc::PeerConnection::GatheringState::Complete) return;
            const std::scoped_lock lock(mutex);
            done = true;
            gathered.notify_all();
        });
        answerer->setRemoteDescription(rtc::Description(body, "offer"));
        std::unique_lock lock(mutex);
        gathered.wait_for(lock, std::chrono::seconds{10}, [&] { return done; });
        return {.status = 201,
                .body = std::string(*answerer->localDescription()),
                .headers = {{"location", "/resource/7"}}};
    };

    live::WhipSession session(
        {.video_id = "SN/80-0-0/normal-0",
         .url = "https://whip.test/whip",
         .width = 160,
         .height = 120,
         .fps = 25,
         .bitrate = 300'000,
         .bind_address = "127.0.0.1"},
        [] { return std::make_shared<const image::RgbImage>(live::pattern(160, 120, 0, "w")); },
        http);
    session.start();
    // The cloud plays the stream as soon as live_start_push answers. So start() returns only
    // after media flows.
    CHECK(session.connected());
    CHECK(session.frames_sent() > 0);
    CHECK(session.resource() == "https://whip.test/resource/7");
    for (int i = 0; i < 100 && packets < 5; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds{50});
    CHECK(session.connected());
    CHECK(session.frames_sent() > 0);
    CHECK(packets >= 5);
    session.stop();
    CHECK(deleted);
    if (answerer) answerer->close();
}
