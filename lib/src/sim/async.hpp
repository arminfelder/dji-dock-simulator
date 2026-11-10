#pragma once

#include <asio/as_tuple.hpp>
#include <asio/awaitable.hpp>
#include <asio/bind_cancellation_slot.hpp>
#include <asio/cancellation_signal.hpp>
#include <asio/co_spawn.hpp>
#include <asio/post.hpp>
#include <asio/steady_timer.hpp>
#include <asio/this_coro.hpp>
#include <asio/use_awaitable.hpp>

#include <chrono>
#include <exception>
#include <functional>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>

// Coroutine building blocks on one asio executor. Everything here runs on a
// single thread, as the asyncio core did. Blocking work goes to to_thread().
namespace docksim::sim {

using Executor = asio::any_io_executor;

[[noreturn]] inline void throw_cancelled() {
    throw std::system_error(std::make_error_code(std::errc::operation_canceled));
}

// A flag that coroutines can wait for. set() wakes every waiter.
class Event {
public:
    explicit Event(Executor ex) : ex_(std::move(ex)) {}

    [[nodiscard]] bool is_set() const noexcept { return set_; }

    void set() {
        set_ = true;
        auto waiters = std::exchange(waiters_, {});
        for (const auto& w : waiters) w->cancel();
    }

    void clear() noexcept { set_ = false; }

    asio::awaitable<void> wait() {
        while (!set_) {
            auto timer =
                std::make_shared<asio::steady_timer>(ex_, asio::steady_timer::time_point::max());
            waiters_.push_back(timer);
            co_await timer->async_wait(asio::as_tuple(asio::use_awaitable));
            if (std::erase(waiters_, timer) > 0) throw_cancelled();
        }
    }

    // Wait at most timeout. True if the event is set.
    asio::awaitable<bool> wait_for(std::chrono::steady_clock::duration timeout) {
        if (set_) co_return true;
        auto timer = std::make_shared<asio::steady_timer>(ex_, timeout);
        waiters_.push_back(timer);
        const auto [ec] = co_await timer->async_wait(asio::as_tuple(asio::use_awaitable));
        const bool woken = std::erase(waiters_, timer) == 0;
        if (!woken && ec == asio::error::operation_aborted) throw_cancelled();
        co_return set_;
    }

private:
    Executor ex_;
    bool set_ = false;
    std::vector<std::shared_ptr<asio::steady_timer>> waiters_;
};

// A value that one side sets once and the other side awaits.
template <class T>
class OneShot {
public:
    explicit OneShot(Executor ex) : ready_(std::move(ex)) {}

    void set(T value) {
        if (value_) return;
        value_ = std::move(value);
        ready_.set();
    }

    // Empty if the timeout passes first.
    asio::awaitable<std::optional<T>> wait_for(std::chrono::steady_clock::duration timeout) {
        if (!co_await ready_.wait_for(timeout)) co_return std::nullopt;
        co_return std::move(value_);
    }

private:
    Event ready_;
    std::optional<T> value_;
};

// Runs a blocking callable on a thread pool and resumes on the caller's executor.
template <class Pool, class F>
auto to_thread(Pool& pool, F fn) -> asio::awaitable<std::invoke_result_t<F>> {
    using R = std::invoke_result_t<F>;
    co_return co_await asio::co_spawn(
        pool.get_executor(),
        [fn = std::move(fn)]() mutable -> asio::awaitable<R> { co_return fn(); },
        asio::use_awaitable);
}

// Background coroutines with a name. A failure is logged, never lost.
// cancel_all() delivers terminal cancellation to every running task.
class TaskGroup {
public:
    using ErrorSink = std::function<void(const std::string& name, std::exception_ptr)>;

    TaskGroup(Executor ex, ErrorSink on_error)
        : ex_(ex), core_(std::make_shared<Core>(std::move(ex), std::move(on_error))) {}
    TaskGroup(const TaskGroup&) = delete;
    TaskGroup& operator=(const TaskGroup&) = delete;
    TaskGroup(TaskGroup&&) = delete;
    TaskGroup& operator=(TaskGroup&&) = delete;
    // Tasks can finish after the group is gone. Their completion holds core_, not the group.
    ~TaskGroup() { cancel(); }

    class Handle {
    public:
        Handle() = default;
        [[nodiscard]] bool done() const { return !state_ || state_->done; }
        void cancel() const {
            if (state_ && !state_->done) state_->signal.emit(asio::cancellation_type::terminal);
        }
        asio::awaitable<void> wait() const {
            if (state_) co_await state_->finished.wait();
        }

    private:
        friend class TaskGroup;
        struct State {
            explicit State(Executor ex, std::string n)
                : name(std::move(n)), finished(std::move(ex)) {}
            std::string name;
            asio::cancellation_signal signal;
            Event finished;
            bool done = false;
        };
        explicit Handle(std::shared_ptr<State> s) : state_(std::move(s)) {}
        std::shared_ptr<State> state_;
    };

    Handle spawn(asio::awaitable<void> work, std::string name) {
        auto state = std::make_shared<Handle::State>(ex_, std::move(name));
        core_->tasks.push_back(state);
        // The work starts on a later turn of the executor, never inside
        // spawn(). A service reply then goes out before the work it starts.
        auto deferred = [](asio::awaitable<void> w) -> asio::awaitable<void> {
            co_await asio::post(co_await asio::this_coro::executor, asio::use_awaitable);
            co_await std::move(w);
        };
        asio::co_spawn(
            ex_, deferred(std::move(work)),
            asio::bind_cancellation_slot(
                state->signal.slot(),
                [core = core_, state](const std::exception_ptr& e) { finish(*core, state, e); }));
        return Handle(state);
    }

    [[nodiscard]] std::size_t size() const noexcept { return core_->tasks.size(); }

    // Wait until all background work has finished. For tests and shutdown.
    asio::awaitable<void> settle() {
        const auto core = core_;
        while (!core->tasks.empty()) {
            core->changed.clear();
            co_await core->changed.wait();
        }
    }

    void cancel() {
        const auto tasks = core_->tasks;
        for (const auto& t : tasks) {
            if (!t->done) t->signal.emit(asio::cancellation_type::terminal);
        }
    }

    asio::awaitable<void> cancel_all() {
        cancel();
        co_await settle();
    }

private:
    struct Core {
        Core(Executor ex, ErrorSink sink) : changed(std::move(ex)), on_error(std::move(sink)) {}
        Event changed;
        ErrorSink on_error;
        std::vector<std::shared_ptr<Handle::State>> tasks;
    };

    static void finish(Core& core, const std::shared_ptr<Handle::State>& state,
                       const std::exception_ptr& e) {
        state->done = true;
        state->finished.set();
        std::erase(core.tasks, state);
        if (e && core.on_error && !is_cancellation(e)) core.on_error(state->name, e);
        core.changed.set();
    }

    static bool is_cancellation(std::exception_ptr e) {
        try {
            std::rethrow_exception(std::move(e));
        } catch (const std::system_error& err) {
            return err.code() == std::errc::operation_canceled ||
                   err.code() == asio::error::operation_aborted;
        } catch (...) {
            return false;
        }
    }

    Executor ex_;
    std::shared_ptr<Core> core_;
};

}  // namespace docksim::sim
