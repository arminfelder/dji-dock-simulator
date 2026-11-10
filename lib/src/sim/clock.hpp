#pragma once

#include <asio/awaitable.hpp>
#include <asio/steady_timer.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <queue>
#include <vector>

// Simulation time. Every delay in the core goes through a Clock.
namespace docksim::sim {

using WallTime = std::chrono::system_clock::time_point;

class Clock {
public:
    Clock() = default;
    Clock(const Clock&) = delete;
    Clock& operator=(const Clock&) = delete;
    Clock(Clock&&) = delete;
    Clock& operator=(Clock&&) = delete;
    virtual ~Clock() = default;

    // Seconds of simulation time. Only differences have a meaning.
    [[nodiscard]] virtual double monotonic() const = 0;
    // Simulation wall time, UTC.
    [[nodiscard]] virtual WallTime now() const = 0;
    virtual asio::awaitable<void> sleep(double seconds) = 0;
};

// Follows the wall clock, optionally faster (time_scale) for training.
class RealClock final : public Clock {
public:
    explicit RealClock(double time_scale = 1.0);

    [[nodiscard]] double time_scale() const noexcept { return scale_; }
    // Re-anchors, so that the simulation time does not jump.
    void set_time_scale(double scale);

    [[nodiscard]] double monotonic() const override;
    [[nodiscard]] WallTime now() const override;
    asio::awaitable<void> sleep(double seconds) override;

private:
    [[nodiscard]] double elapsed() const;

    double scale_;
    double sim0_ = 0.0;
    std::chrono::steady_clock::time_point mono0_;
    WallTime wall0_;
};

// Time that moves only when the owner calls jump(). Sleepers wake in deadline
// order, first in first out on equal deadlines. A test runs the executor until
// every task waits on the clock, then jumps to the earliest deadline.
class VirtualClock final : public Clock {
public:
    explicit VirtualClock(WallTime start, double monotonic = 1000.0);

    [[nodiscard]] double monotonic() const override { return mono_; }
    [[nodiscard]] WallTime now() const override;
    asio::awaitable<void> sleep(double seconds) override;

    // Number of sleepers that wait and are not cancelled.
    [[nodiscard]] std::size_t waiting() const;
    // Move to the earliest deadline and wake its sleeper. False if none waits.
    bool jump();

private:
    struct Sleeper {
        double deadline;
        std::uint64_t seq;
        std::shared_ptr<asio::steady_timer> timer;
        std::shared_ptr<bool> pending;
    };
    struct Later {
        bool operator()(const Sleeper& a, const Sleeper& b) const noexcept {
            return a.deadline != b.deadline ? a.deadline > b.deadline : a.seq > b.seq;
        }
    };

    WallTime start_;
    double mono0_;
    double mono_;
    std::uint64_t seq_ = 0;
    std::priority_queue<Sleeper, std::vector<Sleeper>, Later> sleepers_;
};

// Virtual monotonic time with a wall clock held at one instant, so that every
// run gives the same timestamps.
class FrozenWallClock final : public Clock {
public:
    FrozenWallClock(VirtualClock& inner, WallTime wall) : inner_(inner), wall_(wall) {}

    [[nodiscard]] double monotonic() const override { return inner_.monotonic(); }
    [[nodiscard]] WallTime now() const override { return wall_; }
    asio::awaitable<void> sleep(double seconds) override { return inner_.sleep(seconds); }

private:
    VirtualClock& inner_;
    WallTime wall_;
};

}  // namespace docksim::sim
