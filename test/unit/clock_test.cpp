#include "sim/clock.hpp"

#include <asio/co_spawn.hpp>
#include <asio/detached.hpp>
#include <asio/io_context.hpp>
#include <catch2/catch_test_macros.hpp>

#include <vector>

using namespace docksim::sim;

TEST_CASE("virtual clock wakes sleepers in deadline order") {
    asio::io_context io;
    VirtualClock clock(WallTime{std::chrono::seconds{1'760'000'000}});
    std::vector<int> order;

    const auto sleeper = [&](int id, double seconds) -> asio::awaitable<void> {
        co_await clock.sleep(seconds);
        order.push_back(id);
    };
    asio::co_spawn(io, sleeper(1, 3600.0), asio::detached);
    asio::co_spawn(io, sleeper(2, 0.5), asio::detached);
    asio::co_spawn(io, sleeper(3, 0.5), asio::detached);
    io.poll();
    CHECK(clock.waiting() == 3);

    while (clock.jump()) io.poll();
    CHECK(order == std::vector{2, 3, 1});
    CHECK(clock.monotonic() == 1000.0 + 3600.0);
    CHECK(clock.now() == WallTime{std::chrono::seconds{1'760'003'600}});
}

TEST_CASE("a zero sleep only yields") {
    asio::io_context io;
    VirtualClock clock(WallTime{});
    bool done = false;
    asio::co_spawn(
        io,
        [&]() -> asio::awaitable<void> {
            co_await clock.sleep(0.0);
            done = true;
        },
        asio::detached);
    io.poll();
    CHECK(done);
    CHECK(clock.waiting() == 0);
}

TEST_CASE("real clock rejects a non-positive time scale") {
    CHECK_THROWS(RealClock(0.0));
    RealClock clock(2.0);
    CHECK_THROWS(clock.set_time_scale(-1.0));
    const double before = clock.monotonic();
    clock.set_time_scale(10.0);
    CHECK(clock.monotonic() >= before);
}
