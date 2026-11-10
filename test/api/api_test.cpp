#include "api/server.hpp"
#include "control/sim_control.hpp"
#include "golden/harness.hpp"
#include "net/http.hpp"

#include <asio/connect.hpp>
#include <asio/ip/tcp.hpp>
#include <asio/read_until.hpp>
#include <asio/streambuf.hpp>
#include <asio/write.hpp>

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <format>
#include <functional>
#include <future>

using namespace docksim;

namespace {

constexpr const char* token = "s3cret";

// Runs the dock executor until the call on another thread returns.
template <class F>
auto serve_while(golden::Driver& d, F call) {
    auto result = std::async(std::launch::async, std::move(call));
    while (result.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {
        d.io().restart();
        d.io().run_for(std::chrono::milliseconds(5));
    }
    return result.get();
}

http::Response request(golden::Driver& d, int port, http::Method method, const std::string& path,
                       const std::string& body = {}, bool auth = true) {
    http::Request r{.method = method, .url = std::format("http://127.0.0.1:{}{}", port, path)};
    if (auth) r.headers.emplace_back("Authorization", std::format("Bearer {}", token));
    if (!body.empty()) {
        r.headers.emplace_back("Content-Type", "application/json");
        r.body = body;
    }
    r.timeout = std::chrono::milliseconds(10'000);
    return serve_while(d, [r] { return http::perform(r); });
}

// The upgrade answer and the first frame of the WebSocket, as raw text.
std::string websocket_first_bytes(golden::Driver& d, int port) {
    return serve_while(d, [port] {
        asio::io_context io;
        asio::ip::tcp::socket socket(io);
        socket.connect({asio::ip::make_address("127.0.0.1"), static_cast<unsigned short>(port)});
        const auto hello = std::format(
            "GET /api/ws?token={} HTTP/1.1\r\nHost: 127.0.0.1\r\nUpgrade: websocket\r\n"
            "Connection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n",
            token);
        asio::write(socket, asio::buffer(hello));
        asio::streambuf buffer;
        asio::read_until(socket, buffer, "\"type\":\"state\"");
        return std::string(asio::buffers_begin(buffer.data()), asio::buffers_end(buffer.data()));
    });
}

}  // namespace

// Drogon is one per process, so every check shares one server.
TEST_CASE("api: REST and WebSocket over a running server") {
    golden::Driver d;
    d.connect();
    control::WireLog wire;
    control::SimControl control(d.sim(), nullptr, wire);
    ApiConfig cfg{.enabled = true, .bind = "127.0.0.1", .port = 0, .token = token,
                  .web_root = std::string("/nonexistent")};
    asio::thread_pool workers(1);
    api::ApiServer server(d.io().get_executor(), control, workers, cfg);
    REQUIRE(server.start());
    const int port = server.port();
    REQUIRE(port > 0);
    using http::Method;

    const auto refused = request(d, port, Method::get, "/api/state", {}, false);
    CHECK(refused.status == 401);

    const auto state = request(d, port, Method::get, "/api/state");
    REQUIRE(state.status == 200);
    CHECK(Json::parse(state.body)["dock"]["sn"] == golden::dock_sn);

    const auto windy = request(d, port, Method::put, "/api/weather", R"({"wind_speed": 15})");
    REQUIRE(windy.status == 200);
    CHECK(d.sim().ctx().dock.weather.wind_speed == 15.0);
    CHECK(Json::parse(windy.body)["dock"]["launch_refusal"] == 514135);

    const auto calm = request(d, port, Method::del, "/api/weather");
    CHECK(calm.status == 200);
    CHECK(d.sim().ctx().dock.weather.noise);

    CHECK(request(d, port, Method::put, "/api/battery", "{nope").status == 400);
    CHECK(request(d, port, Method::put, "/api/battery", R"({"percent": 101})").status == 400);
    CHECK(request(d, port, Method::put, "/api/battery", R"({"percent": 55})").status == 200);
    CHECK(d.sim().ctx().drone.battery_level == 55.0);

    CHECK(request(d, port, Method::put, "/api/drone/position",
                  R"({"latitude": 47.0, "longitude": 11.0})")
              .status == 409);
    CHECK(request(d, port, Method::put, "/api/gnss/drone", R"({"gps_number": 3})").status ==
          200);
    CHECK(d.sim().ctx().drone.gnss.gps_number == 3);
    CHECK(request(d, port, Method::put, "/api/clock", R"({"time_scale": 2})").status == 409);

    CHECK(request(d, port, Method::post, "/api/hms", R"({"code": "0x1234"})").status == 200);
    CHECK(request(d, port, Method::del, "/api/hms/0x1234").status == 200);

    const auto camera = request(d, port, Method::get, "/api/camera/drone-wide.jpg?w=320");
    REQUIRE(camera.status == 200);
    CHECK(camera.header("content-type") == "image/jpeg");
    CHECK(camera.body.starts_with("\xFF\xD8"));
    CHECK(request(d, port, Method::get, "/api/camera/nothing.jpg").status == 404);

    const auto root = request(d, port, Method::get, "/", {}, false);
    CHECK(root.status == 200);
    CHECK(root.body.contains("/api/state"));

    const auto ws = websocket_first_bytes(d, port);
    CHECK(ws.starts_with("HTTP/1.1 101"));
    CHECK(ws.contains(golden::dock_sn));

    server.stop();
}
