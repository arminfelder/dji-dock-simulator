#include "api/server.hpp"

#include "control/sim_control.hpp"
#include "image/image.hpp"
#include "log/log.hpp"
#include "media/photo.hpp"
#include "sim/dock_sim.hpp"
#include "sim/views.hpp"
#include "util/json.hpp"

#include <asio/ip/tcp.hpp>
#include <asio/post.hpp>

#include <drogon/HttpAppFramework.h>
#include <drogon/WebSocketController.h>

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <charconv>
#include <format>
#include <mutex>
#include <set>
#include <thread>

namespace docksim::api {

namespace {

const auto& logger() {
    static const auto log = log::get("dock_sim.api");
    return log;
}

constexpr double push_interval_s = 0.5;
constexpr std::size_t wire_backlog = 100;
constexpr int max_camera_width = 1920;
constexpr int jpeg_quality = 80;

using drogon::HttpRequestPtr;
using drogon::HttpResponsePtr;
using Callback = std::function<void(const HttpResponsePtr&)>;

HttpResponsePtr json_response(drogon::HttpStatusCode status, const Json& body) {
    auto r = drogon::HttpResponse::newHttpResponse();
    r->setStatusCode(status);
    r->setContentTypeCode(drogon::CT_APPLICATION_JSON);
    r->setBody(body.dump());
    r->addHeader("Cache-Control", "no-store");
    return r;
}

HttpResponsePtr error_response(drogon::HttpStatusCode status, std::string_view message) {
    return json_response(status, {{"error", message}});
}

Json body_of(const HttpRequestPtr& req) {
    const auto text = req->body();
    if (text.empty()) return Json::object();
    try {
        return Json::parse(text);
    } catch (const Json::exception&) {
        throw control::BadRequest("body is not valid JSON");
    }
}

// Constant-time compare, so that the time of a refusal does not leak the token.
bool same(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    unsigned char diff = 0;
    for (std::size_t i = 0; i < a.size(); ++i)
        diff |= static_cast<unsigned char>(a[i] ^ b[i]);
    return diff == 0;
}

bool port_free(const std::string& bind, int port) {
    if (port == 0) return true;
    try {
        asio::io_context io;
        asio::ip::tcp::acceptor acceptor(io);
        const asio::ip::tcp::endpoint ep(asio::ip::make_address(bind),
                                         static_cast<unsigned short>(port));
        acceptor.open(ep.protocol());
        acceptor.set_option(asio::ip::tcp::acceptor::reuse_address(true));
        acceptor.bind(ep);
        return true;
    } catch (const std::exception& e) {
        logger()->error("api: cannot listen on {}:{}: {}", bind, port, e.what());
        return false;
    }
}

bool loopback(const std::string& bind) {
    try {
        return asio::ip::make_address(bind).is_loopback();
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace

struct ApiServer::State : std::enable_shared_from_this<ApiServer::State> {
    sim::Executor ex;
    // Null after stop(). Read and written on the executor only.
    control::SimControl* control = nullptr;
    asio::thread_pool* workers = nullptr;
    ApiConfig cfg;
    std::filesystem::path root;
    std::jthread thread;
    std::atomic<int> port = 0;

    std::mutex clients_mutex;
    std::set<drogon::WebSocketConnectionPtr> clients;
    std::uint64_t wire_seq = 0;

    // One render per view at a time; an extra request gets 429.
    std::mutex camera_mutex;
    std::set<std::string> rendering;

    // Run fn on the executor of the dock and answer with its JSON.
    template <class F>
    void on_dock(Callback cb, F fn) {
        asio::post(ex, [self = shared_from_this(), cb = std::move(cb), fn = std::move(fn)]() mutable {
            if (self->control == nullptr) {
                cb(error_response(drogon::k503ServiceUnavailable, "the simulator stops"));
                return;
            }
            try {
                cb(json_response(drogon::k200OK, fn(*self->control)));
            } catch (const control::BadRequest& e) {
                cb(error_response(drogon::k400BadRequest, e.what()));
            } catch (const control::Conflict& e) {
                cb(error_response(drogon::k409Conflict, e.what()));
            } catch (const std::exception& e) {
                logger()->warn("api: {}", e.what());
                cb(error_response(drogon::k500InternalServerError, e.what()));
            }
        });
    }

    // Body-parsing variant: a JSON error is a 400 too.
    template <class F>
    void on_dock_body(const HttpRequestPtr& req, Callback cb, F fn) {
        Json body;
        try {
            body = body_of(req);
        } catch (const control::BadRequest& e) {
            cb(error_response(drogon::k400BadRequest, e.what()));
            return;
        }
        on_dock(std::move(cb), [body = std::move(body), fn = std::move(fn)](
                                   control::SimControl& c) { return fn(c, body); });
    }

    [[nodiscard]] bool authorized(const HttpRequestPtr& req) const {
        if (!cfg.token) return true;
        const auto header = req->getHeader("authorization");
        constexpr std::string_view bearer = "Bearer ";
        if (header.starts_with(bearer) &&
            same(std::string_view(header).substr(bearer.size()), *cfg.token))
            return true;
        // Browsers cannot set headers on a WebSocket or an <img>.
        return same(req->getParameter("token"), *cfg.token);
    }

    void push() {
        asio::post(ex, [self = shared_from_this()] {
            if (self->control == nullptr) return;
            std::vector<drogon::WebSocketConnectionPtr> targets;
            {
                const std::scoped_lock lock(self->clients_mutex);
                targets.assign(self->clients.begin(), self->clients.end());
            }
            const auto wire = self->control->wire(self->wire_seq);
            self->wire_seq = wire["last_seq"].get<std::uint64_t>();
            if (targets.empty()) return;
            const auto state =
                Json{{"type", "state"}, {"state", self->control->snapshot()}}.dump();
            const auto entries = wire["entries"];
            const auto wire_text =
                entries.empty() ? std::string{}
                                : Json{{"type", "wire"}, {"entries", entries}}.dump();
            for (const auto& c : targets) {
                if (!c->connected()) continue;
                c->send(state);
                if (!wire_text.empty()) c->send(wire_text);
            }
        });
    }

    void greet(const drogon::WebSocketConnectionPtr& conn) {
        asio::post(ex, [self = shared_from_this(), conn] {
            if (self->control == nullptr) return;
            conn->send(Json{{"type", "state"}, {"state", self->control->snapshot()}}.dump());
            const auto last = self->control->wire_log().last_seq();
            const auto since = last > wire_backlog ? last - wire_backlog : 0;
            const auto wire = self->control->wire(since);
            conn->send(Json{{"type", "wire"}, {"entries", wire["entries"]}}.dump());
        });
    }

    void camera(const std::string& view, int width, Callback cb);
};

namespace {

class LiveSocket final : public drogon::WebSocketController<LiveSocket, false> {
public:
    explicit LiveSocket(ApiServer::State& state) : state_(state) {}

    void handleNewMessage(const drogon::WebSocketConnectionPtr& /*conn*/, std::string&& /*msg*/,
                          const drogon::WebSocketMessageType& /*type*/) override {}

    void handleNewConnection(const HttpRequestPtr& req,
                             const drogon::WebSocketConnectionPtr& conn) override {
        if (!state_.authorized(req)) {
            conn->shutdown(drogon::CloseCode::kViolation, "unauthorized");
            return;
        }
        {
            const std::scoped_lock lock(state_.clients_mutex);
            state_.clients.insert(conn);
        }
        state_.greet(conn);
    }

    void handleConnectionClosed(const drogon::WebSocketConnectionPtr& conn) override {
        const std::scoped_lock lock(state_.clients_mutex);
        state_.clients.erase(conn);
    }

    WS_PATH_LIST_BEGIN
    WS_PATH_ADD("/api/ws");
    WS_PATH_LIST_END

private:
    ApiServer::State& state_;
};

}  // namespace

void ApiServer::State::camera(const std::string& view, int width, Callback cb) {
    if (view != "drone-wide" && view != "drone-zoom" && view != "drone-ir" && view != "dock") {
        cb(error_response(drogon::k404NotFound, "views: drone-wide, drone-zoom, drone-ir, dock"));
        return;
    }
    {
        const std::scoped_lock lock(camera_mutex);
        if (!rendering.insert(view).second) {
            cb(error_response(drogon::k429TooManyRequests, "this view renders already"));
            return;
        }
    }
    asio::post(ex, [self = shared_from_this(), view, width, cb = std::move(cb)]() mutable {
        const auto release = [self, view] {
            const std::scoped_lock lock(self->camera_mutex);
            self->rendering.erase(view);
        };
        if (self->control == nullptr) {
            release();
            cb(error_response(drogon::k503ServiceUnavailable, "the simulator stops"));
            return;
        }
        auto cameras = self->control->dock().shared_cameras();
        asio::post(*self->workers, [cameras = std::move(cameras), view, width, release,
                                    cb = std::move(cb)] {
            const int height = width * 9 / 16;
            HttpResponsePtr response;
            try {
                const auto v = view == "dock" ? cameras->dock_view()
                                              : cameras->drone_view(view.substr(6));
                image::RgbImage frame =
                    v.pose ? cameras->renderer().render(*v.pose, width, height, v.dfov,
                                                        render::FrameOptions{
                                                            .wait_for_data = false,
                                                            .supersample = 1,
                                                            .march_scale = 4,
                                                            .profile = nullptr,
                                                        })
                           : media::synthetic_image({width, height}, v.idle_text);
                if (v.rain && v.pose && v.lens != "ir") {
                    const auto now = std::chrono::steady_clock::now().time_since_epoch();
                    render::apply_rain(frame, *v.rain, std::chrono::duration<double>(now).count());
                }
                const auto jpeg = image::encode_jpeg(frame, jpeg_quality);
                response = drogon::HttpResponse::newHttpResponse();
                response->setContentTypeCode(drogon::CT_IMAGE_JPG);
                response->addHeader("Cache-Control", "no-store");
                response->setBody(
                    std::string(reinterpret_cast<const char*>(jpeg.data()), jpeg.size()));
            } catch (const std::exception& e) {
                logger()->warn("api: camera {}: {}", view, e.what());
                response = error_response(drogon::k500InternalServerError, e.what());
            }
            release();
            cb(response);
        });
    });
}

std::filesystem::path web_root(const ApiConfig& cfg) {
    if (cfg.web_root && !cfg.web_root->empty()) return *cfg.web_root;
    std::error_code ec;
    const auto exe = std::filesystem::read_symlink("/proc/self/exe", ec);
    const auto installed = ec ? std::filesystem::path("web")
                              : exe.parent_path().parent_path() / "share" / "dock-sim" / "web";
#ifdef DOCKSIM_WEB_DIST
    if (!std::filesystem::exists(installed / "index.html", ec) &&
        std::filesystem::exists(std::filesystem::path(DOCKSIM_WEB_DIST) / "index.html", ec))
        return DOCKSIM_WEB_DIST;
#endif
    return installed;
}

ApiServer::ApiServer(sim::Executor ex, control::SimControl& control, asio::thread_pool& workers,
                     ApiConfig cfg)
    : state_(std::make_shared<State>()) {
    state_->ex = std::move(ex);
    state_->control = &control;
    state_->workers = &workers;
    state_->root = web_root(cfg);
    state_->cfg = std::move(cfg);
}

ApiServer::~ApiServer() {
    if (state_->thread.joinable()) {
        drogon::app().quit();
        state_->thread.join();
    }
}

int ApiServer::port() const {
    return state_->port.load();
}

bool ApiServer::start() {
    auto& s = *state_;
    const auto& cfg = s.cfg;
    if (!port_free(cfg.bind, cfg.port)) return false;
    if (!loopback(cfg.bind) && !cfg.token) {
        logger()->warn(
            "api: listening on {} without api.token. Anyone on the network can move the dock, "
            "change the weather and raise faults",
            cfg.bind);
    }
    auto& app = drogon::app();
    const auto uploads =
        std::filesystem::temp_directory_path() / std::format("dock-sim-api-{}", ::getpid());
    app.setThreadNum(1)
        .disableSigtermHandling()
        .setLogLevel(trantor::Logger::kWarn)
        .enableServerHeader(false)
        .setUploadPath(uploads.string())
        .addListener(cfg.bind, static_cast<std::uint16_t>(cfg.port));

    const bool has_ui = std::filesystem::exists(s.root / "index.html");
    if (has_ui) {
        app.setDocumentRoot(s.root.string());
        app.setStaticFilesCacheTime(0);
    } else {
        logger()->warn("api: no web UI in {}; only /api is served", s.root.string());
    }

    State* st = &s;
    // Auth for /api, and the single-page-app fallback for client routes.
    app.registerPreRoutingAdvice([st, has_ui](const HttpRequestPtr& req,
                                              drogon::AdviceCallback&& reply,
                                              drogon::AdviceChainCallback&& next) {
        const auto& path = req->path();
        if (path.starts_with("/api")) {
            if (req->method() != drogon::Options && !st->authorized(req)) {
                reply(error_response(drogon::k401Unauthorized, "missing or wrong api token"));
                return;
            }
        } else if (has_ui && req->method() == drogon::Get) {
            std::error_code ec;
            const auto file = st->root / std::filesystem::path(path).relative_path();
            if (path == "/" || !std::filesystem::is_regular_file(file, ec))
                req->setPath("/index.html");
        }
        next();
    });

    using drogon::Delete;
    using drogon::Get;
    using drogon::Post;
    using drogon::Put;
    using control::SimControl;

    if (!has_ui) {
        app.registerHandler(
            "/",
            [](const HttpRequestPtr&, Callback&& cb) {
                auto r = drogon::HttpResponse::newHttpResponse();
                r->setContentTypeCode(drogon::CT_TEXT_HTML);
                r->setBody("<!doctype html><title>dock-sim</title><p>dock-sim API. The web UI "
                           "is not installed. See <a href=\"/api/state\">/api/state</a>.</p>");
                cb(r);
            },
            {Get});
    }
    app.registerHandler(
        "/api/state",
        [st](const HttpRequestPtr&, Callback&& cb) {
            st->on_dock(std::move(cb), [](SimControl& c) { return c.snapshot(); });
        },
        {Get});
    app.registerHandler(
        "/api/dock/position",
        [st](const HttpRequestPtr& req, Callback&& cb) {
            st->on_dock_body(req, std::move(cb),
                             [](SimControl& c, const Json& b) { return c.move_dock(b); });
        },
        {Put});
    app.registerHandler(
        "/api/drone/position",
        [st](const HttpRequestPtr& req, Callback&& cb) {
            st->on_dock_body(req, std::move(cb),
                             [](SimControl& c, const Json& b) { return c.teleport_drone(b); });
        },
        {Put});
    app.registerHandler(
        "/api/battery",
        [st](const HttpRequestPtr& req, Callback&& cb) {
            st->on_dock_body(req, std::move(cb),
                             [](SimControl& c, const Json& b) { return c.set_battery(b); });
        },
        {Put});
    app.registerHandler(
        "/api/weather",
        [st](const HttpRequestPtr& req, Callback&& cb) {
            if (req->method() == drogon::Delete) {
                st->on_dock(std::move(cb), [](SimControl& c) { return c.reset_weather(); });
                return;
            }
            st->on_dock_body(req, std::move(cb),
                             [](SimControl& c, const Json& b) { return c.set_weather(b); });
        },
        {Put, Delete});
    app.registerHandler(
        "/api/gnss/{target}",
        [st](const HttpRequestPtr& req, Callback&& cb, const std::string& target) {
            st->on_dock_body(req, std::move(cb), [target](SimControl& c, const Json& b) {
                return c.set_gnss(target, b);
            });
        },
        {Put});
    app.registerHandler(
        "/api/clock",
        [st](const HttpRequestPtr& req, Callback&& cb) {
            st->on_dock_body(req, std::move(cb),
                             [](SimControl& c, const Json& b) { return c.set_time_scale(b); });
        },
        {Put});
    app.registerHandler(
        "/api/faults",
        [st](const HttpRequestPtr& req, Callback&& cb) {
            st->on_dock_body(req, std::move(cb),
                             [](SimControl& c, const Json& b) { return c.set_faults(b); });
        },
        {Put});
    app.registerHandler(
        "/api/hms",
        [st](const HttpRequestPtr& req, Callback&& cb) {
            st->on_dock_body(req, std::move(cb),
                             [](SimControl& c, const Json& b) { return c.raise_hms(b); });
        },
        {Post});
    app.registerHandler(
        "/api/hms/{code}",
        [st](const HttpRequestPtr&, Callback&& cb, const std::string& code) {
            st->on_dock(std::move(cb), [code](SimControl& c) { return c.clear_hms(code); });
        },
        {Delete});
    app.registerHandler(
        "/api/wire",
        [st](const HttpRequestPtr& req, Callback&& cb) {
            std::uint64_t since = 0;
            const auto text = req->getParameter("since");
            std::from_chars(text.data(), text.data() + text.size(), since);
            st->on_dock(std::move(cb), [since](SimControl& c) { return c.wire(since); });
        },
        {Get});
    app.registerHandler(
        "/api/camera/{view}",
        [st](const HttpRequestPtr& req, Callback&& cb, const std::string& view) {
            int width = 640;
            const auto text = req->getParameter("w");
            std::from_chars(text.data(), text.data() + text.size(), width);
            width = std::clamp(width, 64, max_camera_width);
            std::string name = view;
            if (name.ends_with(".jpg")) name.resize(name.size() - 4);
            st->camera(name, width, std::move(cb));
        },
        {Get});
    app.registerController(std::make_shared<LiveSocket>(s));

    std::promise<void> ready;
    auto started = ready.get_future();
    s.thread = std::jthread([st, ready = std::move(ready)]() mutable {
        auto& a = drogon::app();
        a.getLoop()->queueInLoop([st, &ready] {
            const auto listeners = drogon::app().getListeners();
            if (!listeners.empty()) st->port.store(listeners.front().toPort());
            ready.set_value();
        });
        a.getLoop()->runEvery(push_interval_s, [st] { st->push(); });
        a.run();
    });
    started.wait();
    logger()->info("api: http://{}:{}/ (web UI {})", cfg.bind, s.port.load(),
                   has_ui ? s.root.string() : "not installed");
    return true;
}

void ApiServer::stop() {
    auto& s = *state_;
    s.control = nullptr;
    if (s.thread.joinable()) {
        {
            const std::scoped_lock lock(s.clients_mutex);
            for (const auto& c : s.clients) c->forceClose();
            s.clients.clear();
        }
        drogon::app().quit();
        s.thread.join();
    }
}

}  // namespace docksim::api
