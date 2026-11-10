#include "transport/mqtt.hpp"

#include "log/log.hpp"

#include <MQTTAsync.h>
#include <asio/post.hpp>

#include <algorithm>
#include <format>
#include <mutex>
#include <tuple>
#include <utility>

namespace docksim::transport {

namespace {

constexpr double backoff_max_s = 30.0;

const auto& logger() {
    static const auto log = log::get("dock_sim.mqtt");
    return log;
}

// Every message on the wire, in full. Silent unless dock-sim runs with --log-wire.
const auto& wire() {
    static const auto log = log::get("dock_sim.wire");
    return log;
}

std::string server_uri(const BrokerConfig& cfg) {
    if (cfg.transport == BrokerTransport::websockets) {
        return std::format("{}://{}:{}{}", cfg.tls ? "wss" : "ws", cfg.host, cfg.port, cfg.ws_path);
    }
    return std::format("{}://{}:{}", cfg.tls ? "ssl" : "tcp", cfg.host, cfg.port);
}

}  // namespace

// State that Paho threads touch. It outlives the transport until Paho is done.
struct MqttTransport::Impl : std::enable_shared_from_this<Impl> {
    explicit Impl(sim::Executor executor) : ex(std::move(executor)) {}

    sim::Executor ex;
    MQTTAsync client = nullptr;
    std::mutex mutex;
    // Set from Paho threads through post(), read on the executor.
    std::shared_ptr<sim::OneShot<int>> connect_result;
    std::shared_ptr<sim::Event> lost;
    std::function<void(std::string, std::string)> on_message;
    bool alive = true;

    static void connection_lost(void* context, char* cause) {
        auto* self = static_cast<Impl*>(context);
        const std::string why = cause != nullptr ? cause : "connection lost";
        asio::post(self->ex, [self = self->shared_from_this(), why] {
            logger()->warn("disconnected: {}", why);
            if (self->lost) self->lost->set();
        });
    }

    static int message_arrived(void* context, char* topic, int topic_len,
                               MQTTAsync_message* message) {
        auto* self = static_cast<Impl*>(context);
        std::string name = topic_len > 0 ? std::string(topic, static_cast<std::size_t>(topic_len))
                                         : std::string(topic);
        std::string body(static_cast<const char*>(message->payload),
                         static_cast<std::size_t>(message->payloadlen));
        MQTTAsync_freeMessage(&message);
        MQTTAsync_free(topic);
        asio::post(self->ex, [self = self->shared_from_this(), name = std::move(name),
                              body = std::move(body)]() mutable {
            if (self->alive && self->on_message) self->on_message(std::move(name), std::move(body));
        });
        return 1;
    }

    static void connect_ok(void* context, MQTTAsync_successData* /*unused*/) {
        auto* self = static_cast<Impl*>(context);
        asio::post(self->ex, [self = self->shared_from_this()] {
            if (self->connect_result) self->connect_result->set(MQTTASYNC_SUCCESS);
        });
    }

    static void connect_failed(void* context, MQTTAsync_failureData* data) {
        auto* self = static_cast<Impl*>(context);
        const int code = data != nullptr && data->code != 0 ? data->code : MQTTASYNC_FAILURE;
        asio::post(self->ex, [self = self->shared_from_this(), code] {
            if (self->connect_result) self->connect_result->set(code);
        });
    }
};

MqttTransport::MqttTransport(const sim::Executor& ex, BrokerConfig cfg, std::string client_id)
    : ex_(ex),
      cfg_(std::move(cfg)),
      client_id_(std::move(client_id)),
      connected_(ex),
      impl_(std::make_shared<Impl>(ex)),
      handler_tasks_(ex, [](const std::string& topic, std::exception_ptr e) {
          try {
              std::rethrow_exception(std::move(e));
          } catch (const std::exception& error) {
              logger()->error("handler failed for {}: {}", topic, error.what());
          } catch (...) {
              logger()->error("handler failed for {}", topic);
          }
      }) {
    MQTTAsync_createOptions create = MQTTAsync_createOptions_initializer;
    create.MQTTVersion = MQTTVERSION_3_1_1;
    const auto uri = server_uri(cfg_);
    if (const int rc = MQTTAsync_createWithOptions(&impl_->client, uri.c_str(), client_id_.c_str(),
                                                   MQTTCLIENT_PERSISTENCE_NONE, nullptr, &create);
        rc != MQTTASYNC_SUCCESS) {
        throw std::runtime_error(
            std::format("MQTT client for {}: {}", uri, MQTTAsync_strerror(rc)));
    }
    MQTTAsync_setCallbacks(impl_->client, impl_.get(), &Impl::connection_lost,
                           &Impl::message_arrived, nullptr);
    impl_->on_message = [this](std::string topic, std::string body) {
        Json payload;
        try {
            payload = Json::parse(body);
        } catch (const Json::exception&) {
            logger()->warn("<- {} unparsable payload", topic);
            return;
        }
        wire()->debug("<- {} {}", topic, body);
        for (const auto& [suffix, handler] : handlers_) {
            if (!topic.ends_with(suffix)) continue;
            // One task per message. A slow handler must not stall the session.
            std::ignore = handler_tasks_.spawn(handler(topic, payload), topic);
        }
    };
}

MqttTransport::~MqttTransport() {
    impl_->alive = false;
    if (impl_->client != nullptr) {
        if (MQTTAsync_isConnected(impl_->client) != 0) {
            MQTTAsync_disconnectOptions opts = MQTTAsync_disconnectOptions_initializer;
            opts.timeout = 1000;
            MQTTAsync_disconnect(impl_->client, &opts);
        }
        MQTTAsync_destroy(&impl_->client);
    }
}

void MqttTransport::on(std::string suffix, Handler handler) {
    handlers_.emplace_back(std::move(suffix), std::move(handler));
}

void MqttTransport::subscribe(std::string topic, std::optional<int> qos) {
    const int level = qos.value_or(cfg_.qos);
    if (auto it = std::ranges::find(subscriptions_, topic, &std::pair<std::string, int>::first);
        it != subscriptions_.end()) {
        it->second = level;
    } else {
        subscriptions_.emplace_back(std::move(topic), level);
    }
}

asio::awaitable<void> MqttTransport::publish(std::string topic, Json payload,
                                             std::optional<int> qos) {
    if (!connected_.is_set() || MQTTAsync_isConnected(impl_->client) == 0) {
        logger()->warn("{}: not connected, dropped publish to {}", client_id_, topic);
        co_return;
    }
    std::string body = payload.dump();
    wire()->debug("-> {} {}", topic, body);
    MQTTAsync_message message = MQTTAsync_message_initializer;
    message.payload = body.data();
    message.payloadlen = static_cast<int>(body.size());
    message.qos = qos.value_or(cfg_.qos);
    MQTTAsync_responseOptions opts = MQTTAsync_responseOptions_initializer;
    if (const int rc = MQTTAsync_sendMessage(impl_->client, topic.c_str(), &message, &opts);
        rc != MQTTASYNC_SUCCESS) {
        logger()->warn("{}: publish to {} failed: {}", client_id_, topic, MQTTAsync_strerror(rc));
    }
    co_return;
}

asio::awaitable<void> MqttTransport::run(bool reconnect) {
    double backoff = 1.0;
    bool ever_connected = false;
    while (true) {
        impl_->connect_result = std::make_shared<sim::OneShot<int>>(ex_);
        impl_->lost = std::make_shared<sim::Event>(ex_);

        const MQTTAsync_connectOptions plain = MQTTAsync_connectOptions_initializer;
        const MQTTAsync_connectOptions ws = MQTTAsync_connectOptions_initializer_ws;
        MQTTAsync_connectOptions opts = cfg_.transport == BrokerTransport::websockets ? ws : plain;
        MQTTAsync_SSLOptions ssl = MQTTAsync_SSLOptions_initializer;
        opts.keepAliveInterval = cfg_.keepalive;
        opts.cleansession = 1;
        opts.MQTTVersion = MQTTVERSION_3_1_1;
        opts.automaticReconnect = 0;
        opts.connectTimeout = 20;
        opts.context = impl_.get();
        opts.onSuccess = &Impl::connect_ok;
        opts.onFailure = &Impl::connect_failed;
        if (cfg_.username) opts.username = cfg_.username->c_str();
        if (cfg_.password) opts.password = cfg_.password->c_str();
        if (cfg_.tls) {
            ssl.enableServerCertAuth = cfg_.insecure ? 0 : 1;
            ssl.verify = cfg_.insecure ? 0 : 1;
            if (cfg_.ca_file) ssl.trustStore = cfg_.ca_file->c_str();
            opts.ssl = &ssl;
        }

        int rc = MQTTAsync_connect(impl_->client, &opts);
        if (rc == MQTTASYNC_SUCCESS) {
            rc = (co_await impl_->connect_result->wait_for(std::chrono::seconds{30}))
                     .value_or(MQTTASYNC_FAILURE);
        }
        if (rc == MQTTASYNC_SUCCESS) {
            for (const auto& [topic, qos] : subscriptions_) {
                MQTTAsync_responseOptions sub = MQTTAsync_responseOptions_initializer;
                MQTTAsync_subscribe(impl_->client, topic.c_str(), qos, &sub);
            }
            logger()->info("{} connected to {}:{}", client_id_, cfg_.host, cfg_.port);
            ever_connected = true;
            backoff = 1.0;
            connected_.set();
            co_await impl_->lost->wait();
            connected_.clear();
        } else if (!ever_connected && rc > 0) {
            // A CONNACK return code: bad credentials or not authorized.
            throw ConnectionRefused(
                std::format("broker refused {}: CONNACK code {}", client_id_, rc));
        } else {
            logger()->warn("{} cannot connect to {}:{}: {}", client_id_, cfg_.host, cfg_.port,
                           MQTTAsync_strerror(rc));
        }
        if (!reconnect) throw ConnectionLost(client_id_);
        asio::steady_timer timer(ex_,
                                 std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                     std::chrono::duration<double>(backoff)));
        co_await timer.async_wait(asio::use_awaitable);
        backoff = std::min(backoff * 2, backoff_max_s);
    }
}

}  // namespace docksim::transport
