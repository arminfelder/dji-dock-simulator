#include "sim/clock.hpp"

#include <asio/as_tuple.hpp>
#include <asio/post.hpp>
#include <asio/this_coro.hpp>
#include <asio/use_awaitable.hpp>

#include <algorithm>
#include <stdexcept>
#include <system_error>

namespace docksim::sim {

namespace {

using std::chrono::duration;
using std::chrono::duration_cast;

void require_positive(double scale) {
    if (!(scale > 0.0)) throw std::invalid_argument("time_scale must be positive");
}

}  // namespace

RealClock::RealClock(double time_scale)
    : scale_(time_scale),
      mono0_(std::chrono::steady_clock::now()),
      wall0_(std::chrono::system_clock::now()) {
    require_positive(time_scale);
}

double RealClock::elapsed() const {
    return duration<double>(std::chrono::steady_clock::now() - mono0_).count() * scale_;
}

void RealClock::set_time_scale(double scale) {
    require_positive(scale);
    sim0_ = monotonic();
    wall0_ = now();
    mono0_ = std::chrono::steady_clock::now();
    scale_ = scale;
}

double RealClock::monotonic() const {
    return sim0_ + elapsed();
}

WallTime RealClock::now() const {
    return wall0_ + duration_cast<WallTime::duration>(duration<double>(elapsed()));
}

asio::awaitable<void> RealClock::sleep(double seconds) {
    asio::steady_timer timer(co_await asio::this_coro::executor);
    timer.expires_after(duration_cast<std::chrono::steady_clock::duration>(
        duration<double>(std::max(0.0, seconds) / scale_)));
    co_await timer.async_wait(asio::use_awaitable);
}

VirtualClock::VirtualClock(WallTime start, double monotonic)
    : start_(start), mono0_(monotonic), mono_(monotonic) {}

WallTime VirtualClock::now() const {
    return start_ + duration_cast<WallTime::duration>(duration<double>(mono_ - mono0_));
}

asio::awaitable<void> VirtualClock::sleep(double seconds) {
    auto executor = co_await asio::this_coro::executor;
    if (seconds <= 0.0) {
        co_await asio::post(executor, asio::use_awaitable);
        co_return;
    }
    auto timer = std::make_shared<asio::steady_timer>(executor);
    timer->expires_at(asio::steady_timer::time_point::max());
    auto pending = std::make_shared<bool>(true);
    sleepers_.push({mono_ + seconds, ++seq_, timer, pending});

    co_await timer->async_wait(asio::as_tuple(asio::use_awaitable));
    if (*pending) {
        // Woken by cancellation of the awaiting task, not by jump().
        *pending = false;
        throw std::system_error(std::make_error_code(std::errc::operation_canceled));
    }
}

std::size_t VirtualClock::waiting() const {
    auto copy = sleepers_;
    std::size_t n = 0;
    for (; !copy.empty(); copy.pop()) n += *copy.top().pending ? 1U : 0U;
    return n;
}

bool VirtualClock::jump() {
    while (!sleepers_.empty()) {
        const Sleeper next = sleepers_.top();
        sleepers_.pop();
        if (!*next.pending) continue;
        mono_ = std::max(mono_, next.deadline);
        *next.pending = false;
        next.timer->cancel();
        return true;
    }
    return false;
}

}  // namespace docksim::sim
