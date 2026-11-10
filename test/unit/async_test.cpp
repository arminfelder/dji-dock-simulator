#include "sim/async.hpp"

#include "device/state.hpp"
#include "sim/views.hpp"

#include <asio/io_context.hpp>
#include <asio/steady_timer.hpp>
#include <asio/use_awaitable.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <tuple>

using namespace docksim;
using namespace std::chrono_literals;

TEST_CASE("a task that finishes after its TaskGroup is gone touches no freed memory", "[async]") {
    asio::io_context io;
    auto finished = std::make_shared<bool>(false);
    {
        sim::TaskGroup group(io.get_executor(),
                             [](const std::string&, const std::exception_ptr&) {});
        std::ignore = group.spawn(
            [](asio::any_io_executor ex, std::shared_ptr<bool> done) -> asio::awaitable<void> {
                asio::steady_timer timer(ex, 10ms);
                co_await timer.async_wait(asio::use_awaitable);
                *done = true;
            }(io.get_executor(), finished),
            "late");
        io.poll();
    }
    io.run();
    CHECK(io.stopped());
}

TEST_CASE("the drone sampler never waits for the dock thread", "[async]") {
    asio::io_context io;
    const DeviceConfig cfg;
    device::DroneState drone(cfg);
    drone.latitude = 47.0;
    sim::DroneSampler const sampler(io.get_executor(), drone);
    const auto sample = sampler.function();

    // The dock thread does not run. The old sampler blocked here until it did.
    auto first = std::async(std::launch::async, sample);
    REQUIRE(first.wait_for(2s) == std::future_status::ready);
    CHECK(first.get().latitude == 47.0);

    drone.latitude = 48.0;
    auto queued = std::async(std::launch::async, sample);
    REQUIRE(queued.wait_for(2s) == std::future_status::ready);
    CHECK(queued.get().latitude == 47.0);
    io.run();
    auto fresh = std::async(std::launch::async, sample);
    REQUIRE(fresh.wait_for(2s) == std::future_status::ready);
    CHECK(fresh.get().latitude == 48.0);
}

TEST_CASE("the drone sampler stops reading the drone after it is destroyed", "[async]") {
    asio::io_context io;
    const DeviceConfig cfg;
    auto drone = std::make_unique<device::DroneState>(cfg);
    drone->latitude = 47.0;
    auto sampler = std::make_unique<sim::DroneSampler>(io.get_executor(), *drone);
    const auto sample = sampler->function();
    auto queued = std::async(std::launch::async, sample);
    REQUIRE(queued.wait_for(2s) == std::future_status::ready);
    sampler.reset();
    drone.reset();
    io.run();
    CHECK(sample().latitude == 47.0);
}
