#include "net/url.hpp"

#include <catch2/catch_test_macros.hpp>

using namespace docksim;

TEST_CASE("url parses absolute URLs of any scheme and refuses the rest", "[url]") {
    const auto u = url::Url::parse("rtmp://user:pw@host:1935/live/s?streamToken=T&x=1#f");
    REQUIRE(u);
    CHECK(u->get(url::Part::scheme) == "rtmp");
    CHECK(u->get(url::Part::user) == "user");
    CHECK(u->get(url::Part::password) == "pw");
    CHECK(u->get(url::Part::host) == "host");
    CHECK(u->get(url::Part::port) == "1935");
    CHECK(u->get(url::Part::path) == "/live/s");
    CHECK(u->get(url::Part::query) == "streamToken=T&x=1");
    CHECK(u->get(url::Part::fragment) == "f");
    CHECK_FALSE(url::Url::parse("/relative/path"));
    CHECK_FALSE(url::Url::parse("not a url"));
    CHECK_FALSE(url::Url::parse(""));
}

TEST_CASE("url parts that are absent read as empty, and setting them back clears them", "[url]") {
    auto u = url::Url::parse("rtsp://host:8554/live");
    REQUIRE(u);
    CHECK_FALSE(u->get(url::Part::password));
    CHECK_FALSE(u->get(url::Part::query));
    u->set(url::Part::query, "a=1");
    CHECK(u->href() == "rtsp://host:8554/live?a=1");
    u->set(url::Part::query, std::nullopt);
    CHECK(u->href() == "rtsp://host:8554/live");
}

TEST_CASE("url copies are independent", "[url]") {
    const auto a = url::Url::parse("https://h/a");
    REQUIRE(a);
    auto b = *a;
    b.set(url::Part::path, "/b");
    CHECK(a->href() == "https://h/a");
    CHECK(b.href() == "https://h/b");
}

TEST_CASE("url query parameters stay encoded", "[url]") {
    const auto u = url::Url::parse("https://h/w?app=&stream=SN%2F80&flag&streamToken=T1");
    REQUIRE(u);
    CHECK(u->query_param("stream") == "SN%2F80");
    CHECK(u->query_param("app") == "");
    CHECK(u->query_param("flag") == "");
    CHECK(u->query_param("streamToken") == "T1");
    CHECK_FALSE(u->query_param("missing"));
    CHECK_FALSE(url::Url::parse("https://h/w")->query_param("x"));
}
