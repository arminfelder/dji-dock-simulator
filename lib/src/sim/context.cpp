#include "sim/context.hpp"

#include "log/log.hpp"
#include "protocol/envelope.hpp"
#include "protocol/topics.hpp"

#include <utility>

namespace docksim::sim {

namespace {

void log_failure(const std::string& name, std::exception_ptr e) {
    static const auto logger = log::get("dock_sim");
    try {
        std::rethrow_exception(std::move(e));
    } catch (const std::exception& error) {
        logger->error("{} failed: {}", name, error.what());
    } catch (...) {
        logger->error("{} failed", name);
    }
}

}  // namespace

DockContext::DockContext(const Executor& ex, const Config& cfg, Clock& clock,
                         transport::Transport& transport, asio::thread_pool& workers)
    : dock(cfg.device),
      drone(cfg.device),
      ex_(ex),
      cfg_(&cfg),
      clock_(&clock),
      transport_(&transport),
      workers_(&workers),
      requests_(ex, transport, topic::requests(cfg.device.dock_sn), cfg.device.dock_sn),
      tasks_(ex, log_failure) {}

asio::awaitable<void> DockContext::emit_event(std::string method, Json data, EventOptions options) {
    Json extra = {{"gateway", dock_sn()}, {"need_reply", options.need_reply ? 1 : 0}};
    if (!options.bid.empty()) extra["bid"] = std::move(options.bid);
    co_await transport_->publish(topic::events(dock_sn()),
                                 envelope::make(std::move(data), method, std::move(extra)));
}

asio::awaitable<Json> DockContext::request(std::string method, Json data,
                                           std::chrono::steady_clock::duration timeout) {
    co_return co_await requests_.request(std::move(method), std::move(data), timeout);
}

TaskGroup::Handle DockContext::spawn(asio::awaitable<void> work, std::string name) {
    return tasks_.spawn(std::move(work), std::move(name));
}

}  // namespace docksim::sim
