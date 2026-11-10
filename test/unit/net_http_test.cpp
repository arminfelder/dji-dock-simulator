#include "net/http.hpp"
#include "support/http_server.hpp"

#include <docksim/version.hpp>

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <format>
#include <string>
#include <thread>

using namespace docksim;
using namespace std::chrono_literals;
using docksim::test::Reply;
using docksim::test::Seen;
using docksim::test::Server;

TEST_CASE("http GET sends headers and returns status, body and lower-case headers", "[http]") {
    Server const server([](const Seen&, const std::stop_token&) {
        return Reply{.status = 200, .headers = {{"X-Thing", "Value"}}, .body = "hello"};
    });
    const auto r = http::perform({.url = server.url("/a"), .headers = {{"X-Auth-Token", "t1"}}});
    CHECK(r.status == 200);
    CHECK(r.body == "hello");
    CHECK(r.headers.at("x-thing") == "Value");
    CHECK(r.header("X-THING") == "Value");
    CHECK(r.effective_url == server.url("/a"));
    const auto seen = server.seen();
    REQUIRE(seen.size() == 1);
    CHECK(seen[0].method == "GET");
    CHECK(seen[0].head.find("X-Auth-Token: t1") != std::string::npos);
    CHECK(seen[0].head.find(std::format("User-Agent: dock-sim/{}", version())) !=
          std::string::npos);
}

TEST_CASE("http POST and PUT send the body; GET and DELETE do not", "[http]") {
    Server const server([](const Seen&, const std::stop_token&) {
        return Reply{.status = 201, .headers = {}, .body = ""};
    });
    for (const auto method :
         {http::Method::post, http::Method::put, http::Method::get, http::Method::del}) {
        const auto r =
            http::perform({.method = method, .url = server.url("/b"), .body = "payload"});
        CHECK(r.status == 201);
    }
    const auto seen = server.seen();
    REQUIRE(seen.size() == 4);
    CHECK(seen[0].method == "POST");
    CHECK(seen[0].body == "payload");
    CHECK(seen[1].method == "PUT");
    CHECK(seen[1].body == "payload");
    CHECK(seen[2].method == "GET");
    CHECK(seen[2].body.empty());
    CHECK(seen[3].method == "DELETE");
    CHECK(seen[3].body.empty());
}

TEST_CASE("http redirects keep only the final headers", "[http]") {
    Server const server([](const Seen& s, const std::stop_token&) {
        if (s.path == "/from")
            return Reply{
                .status = 302, .headers = {{"Location", "/to"}, {"X-First", "1"}}, .body = ""};
        return Reply{.status = 200, .headers = {{"X-Last", "2"}}, .body = "done"};
    });
    const auto followed = http::perform({.url = server.url("/from")});
    CHECK(followed.status == 200);
    CHECK(followed.body == "done");
    CHECK(followed.effective_url == server.url("/to"));
    CHECK_FALSE(followed.header("x-first"));
    CHECK(followed.header("x-last") == "2");

    const auto stayed = http::perform({.url = server.url("/from"), .follow_redirects = false});
    CHECK(stayed.status == 302);
    CHECK(stayed.header("location") == "/to");
}

TEST_CASE("http error status is a normal response", "[http]") {
    Server const server([](const Seen&, const std::stop_token&) {
        return Reply{.status = 404, .headers = {}, .body = "missing"};
    });
    const auto r = http::perform({.url = server.url("/c")});
    CHECK(r.status == 404);
    CHECK(r.body == "missing");
}

TEST_CASE("http transport failures throw TransportError", "[http]") {
    std::string closed;
    {
        Server const server([](const Seen&, const std::stop_token&) {
            return Reply{.status = 200, .headers = {}, .body = ""};
        });
        closed = server.url("/gone");
    }
    CHECK_THROWS_AS(http::perform({.url = closed, .timeout = 2s}), http::TransportError);
}

TEST_CASE("http stop request and timeout end a slow transfer", "[http]") {
    Server const server([](const Seen&, const std::stop_token& stop) {
        const auto until = std::chrono::steady_clock::now() + 10s;
        while (!stop.stop_requested() && std::chrono::steady_clock::now() < until)
            std::this_thread::sleep_for(20ms);
        return Reply{.status = 200, .headers = {}, .body = "late"};
    });

    std::stop_source source;
    std::jthread const stopper([&source] {
        std::this_thread::sleep_for(300ms);
        source.request_stop();
    });
    auto started = std::chrono::steady_clock::now();
    CHECK_THROWS_AS(http::perform({.url = server.url("/slow"), .stop = source.get_token()}),
                    http::TransportError);
    CHECK(std::chrono::steady_clock::now() - started < 3s);

    started = std::chrono::steady_clock::now();
    CHECK_THROWS_AS(http::perform({.url = server.url("/slow"), .timeout = 300ms}),
                    http::TransportError);
    CHECK(std::chrono::steady_clock::now() - started < 3s);
}

TEST_CASE("https checks the server certificate unless verification is off", "[http]") {
    const docksim::test::TestCertificate cert;
    Server const server(
        [](const Seen&, const std::stop_token&) {
            return Reply{.status = 200, .headers = {}, .body = "secure"};
        },
        &cert);
    REQUIRE(server.url("/").starts_with("https://"));

    const auto trusted = http::perform({.url = server.url("/t"), .ca_file = cert.cert_path()});
    CHECK(trusted.status == 200);
    CHECK(trusted.body == "secure");

    CHECK_THROWS_AS(
        http::perform({.url = server.url("/u"), .ca_file = std::nullopt, .timeout = 5s}),
        http::TransportError);

    const auto unchecked = http::perform({.url = server.url("/v"), .verify_tls = false});
    CHECK(unchecked.status == 200);
}
