#pragma once

#include "sim/async.hpp"
#include "sim/clock.hpp"
#include "sim/dock_sim.hpp"
#include "transport/transport.hpp"
#include "util/json.hpp"

#include <docksim/config.hpp>

#include <asio/executor_work_guard.hpp>
#include <asio/io_context.hpp>
#include <asio/thread_pool.hpp>

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

// Test doubles for the broker, the cloud, S3 and the clock, so that one
// golden scenario always gives the same messages.
namespace docksim::golden {

inline constexpr std::int64_t frozen_epoch = 1'760'000'000;
inline const sim::WallTime frozen_now{std::chrono::seconds{frozen_epoch}};
inline constexpr const char* dock_sn = "SIMDOCK0000001";
inline constexpr const char* drone_sn = "SIMDRONE0000001";

// A custom flight area file: a no-fly circle of 200 m about 1 km north of the dock.
inline constexpr const char* areas_url = "https://files.example.invalid/areas.json";
[[nodiscard]] std::string areas_geojson();
// The flight_areas_get or offline_map_get reply for one file with this content.
[[nodiscard]] Json file_list_reply(const std::string& name, const std::string& url,
                                   const std::string& content);

// Answers the requests topic with canned replies, as the backend would.
class FakeCloud {
public:
    FakeCloud();
    std::map<std::string, Json> replies;
    [[nodiscard]] std::optional<std::pair<std::string, Json>> reply_for(const std::string& topic,
                                                                        const Json& payload) const;
};

struct Published {
    std::string topic;
    int qos;
    Json payload;
};

// Records publishes and runs the handlers of injected messages in order, inline.
class FakeTransport final : public transport::Session {
public:
    FakeTransport(sim::Executor ex, int default_qos, std::string client_id)
        : ex_(std::move(ex)), qos_(default_qos), client_id_(std::move(client_id)), closed_(ex_) {}

    void on(std::string suffix, transport::Handler handler) override;
    void subscribe(std::string topic, std::optional<int> qos) override;
    asio::awaitable<void> publish(std::string topic, Json payload, std::optional<int> qos) override;
    asio::awaitable<void> run(bool reconnect) override;
    [[nodiscard]] bool connected() const override { return true; }
    asio::awaitable<void> wait_connected() override { co_return; }
    asio::awaitable<void> cancel_handlers() override { co_return; }

    asio::awaitable<void> inject(std::string topic, Json payload);

    const FakeCloud* responder = nullptr;
    std::vector<std::pair<std::string, int>> subscriptions;
    std::vector<Published> published;
    [[nodiscard]] const std::string& client_id() const { return client_id_; }

private:
    sim::Executor ex_;
    int qos_;
    std::string client_id_;
    sim::Event closed_;
    std::deque<std::pair<std::string, transport::Handler>> handlers_;
};

// Answers live_start_push with success without a media server.
class OkStreamer final : public live::Streamer {
public:
    code::Result start(const std::string& /*video_id*/, const std::string& /*url*/,
                       code::UrlType /*type*/) override {
        return code::Result::ok;
    }
    code::Result stop(const std::string& /*video_id*/) override { return code::Result::ok; }
    void set_lens(const std::string& /*video_id*/, const std::string& /*lens*/) override {}
    void stop_all() override {}
};

[[nodiscard]] Config golden_config();

// Replaces values that change on every run: ids, timestamps, weather noise.
[[nodiscard]] Json normalize(const Json& value);
[[nodiscard]] Json wire(const std::vector<Published>& published);

class Driver {
public:
    explicit Driver(Config cfg = golden_config());
    Driver(const Driver&) = delete;
    Driver& operator=(const Driver&) = delete;
    Driver(Driver&&) = delete;
    Driver& operator=(Driver&&) = delete;
    ~Driver();

    void connect();
    void osd_tick();
    void service(const std::string& method, Json data);
    void property_set(const std::string& sn, Json data);
    void wait_mission();
    void upload_media(const std::string& flight_id);
    void drc_inject(const std::string& topic, Json payload);
    // Let simulation time pass. Background work runs meanwhile.
    void advance(double seconds);
    void wait_manual();
    // Run any step on the executor, with the clock jumps of advance().
    void run(asio::awaitable<void> step) { drive(std::move(step)); }

    [[nodiscard]] FakeTransport& main_transport() { return *transport_; }
    [[nodiscard]] FakeTransport& drc_session() { return *drc_sessions_.back(); }
    [[nodiscard]] std::size_t drc_session_count() const { return drc_sessions_.size(); }

    [[nodiscard]] Json subscriptions() const;
    [[nodiscard]] Json main_wire() const { return wire(transport_->published); }
    [[nodiscard]] Json drc_wire() const;
    void clear() { transport_->published.clear(); }

    bool run_spawned = false;
    std::vector<std::pair<std::string, std::size_t>> puts;
    // The put with this 1-based number throws once. 0 means no failure.
    int fail_put = 0;
    // Bodies of the file URLs that the fake download knows. Any other URL fails.
    std::map<std::string, std::string> files;
    std::vector<std::string> fetched;
    // Wayline URLs in download order. A URL that ends in broken.kmz fails.
    std::vector<std::string> downloads;
    [[nodiscard]] FakeCloud& cloud() { return cloud_; }

    [[nodiscard]] sim::DockSim& sim() { return *sim_; }
    // For tests that serve requests from other threads: run it while they wait.
    [[nodiscard]] asio::io_context& io() { return io_; }

private:
    void drive(asio::awaitable<void> step);

    asio::io_context io_;
    asio::executor_work_guard<asio::io_context::executor_type> guard_;
    asio::thread_pool workers_{2};
    Config cfg_;
    sim::VirtualClock virtual_;
    sim::FrozenWallClock clock_;
    FakeCloud cloud_;
    std::unique_ptr<FakeTransport> transport_;
    std::vector<std::shared_ptr<FakeTransport>> drc_sessions_;
    std::unique_ptr<sim::DockSim> sim_;
};

// Compares as JSON values: key order and int against float do not matter.
[[nodiscard]] bool same_json(const Json& a, const Json& b);
[[nodiscard]] Json load_snapshot(const std::string& name);
// Writes the actual value next to the snapshot, for a diff by hand.
void dump_actual(const std::string& name, const Json& actual);

}  // namespace docksim::golden
