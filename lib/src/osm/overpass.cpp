#include "osm/overpass.hpp"

#include "log/log.hpp"
#include "net/http.hpp"
#include "osm/features.hpp"
#include "osm/footprints.hpp"
#include "util/numeric.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <format>
#include <mutex>
#include <sstream>
#include <tuple>

namespace docksim::osm {

namespace {

const auto& logger() {
    static const auto log = log::get("dock_sim.osm");
    return log;
}

std::string host(std::string_view url) {
    const auto scheme = url.find("://");
    if (scheme != std::string_view::npos) url.remove_prefix(scheme + 3);
    return std::string(url.substr(0, url.find('/')));
}

// "HTTP 502 x3" style summary of the answers of one mirror.
std::string summary(const std::vector<std::string>& answers) {
    std::vector<std::pair<std::string, int>> parts;
    for (const auto& a : answers) {
        if (!parts.empty() && parts.back().first == a) {
            ++parts.back().second;
        } else {
            parts.emplace_back(a, 1);
        }
    }
    std::string out;
    for (const auto& [answer, count] : parts) {
        if (!out.empty()) out += ", ";
        out += count > 1 ? std::format("{} x{}", answer, count) : answer;
    }
    return out;
}

std::string form_encode(std::string_view text) {
    std::string out;
    for (const char ch : text) {
        const auto c = static_cast<unsigned char>(ch);
        if (std::isalnum(c) != 0 || c == '-' || c == '_' || c == '.' || c == '~') {
            out += ch;
        } else if (c == ' ') {
            out += '+';
        } else {
            out += std::format("%{:02X}", c);
        }
    }
    return out;
}

bool transient(long status) {
    return status == 500 || status == 502 || status == 503 || status == 504;
}

}  // namespace

std::string format_query(double south, double west, double north, double east) {
    return format_query(overpass_query, south, west, north, east);
}

std::string format_query(std::string_view query, double south, double west, double north,
                         double east) {
    std::string out(query);
    const std::array<std::pair<std::string_view, double>, 4> fields{
        {{"{s}", south}, {"{w}", west}, {"{n}", north}, {"{e}", east}}};
    for (const auto& [key, value] : fields) {
        const auto text = std::format("{:.4f}", value);
        for (auto pos = out.find(key); pos != std::string::npos;
             pos = out.find(key, pos + text.size())) {
            out.replace(pos, key.size(), text);
        }
    }
    return out;
}

std::string area_query() {
    // features_query without its settings: "[...];(" starts the union.
    const auto body = features_query.substr(features_query.find("];(") + 2);
    return std::string(overpass_query) + std::string(body);
}

double retry_after(std::optional<std::string_view> value,
                   std::chrono::system_clock::time_point now) {
    if (!value || value->empty()) return retry_after_default_s;
    if (const auto seconds = num::parse_double(*value)) return std::max(0.0, *seconds);
    std::istringstream in{std::string(*value)};
    std::chrono::sys_seconds when;
    in >> std::chrono::parse("%a, %d %b %Y %H:%M:%S GMT", when);
    if (in.fail()) return retry_after_default_s;
    return std::max(0.0, std::chrono::duration<double>(when - now).count());
}

Post curl_post() {
    return [](const std::string& url, const std::string& query, std::string& problem,
              const std::stop_token& stop) -> std::optional<PostReply> {
        try {
            const auto response = http::perform({
                .method = http::Method::post,
                .url = url,
                .headers = {{"Content-Type", "application/x-www-form-urlencoded"},
                            {"User-Agent", std::string(overpass_user_agent)}},
                .body = "data=" + form_encode(query),
                .timeout = std::chrono::seconds{60},
                .stop = stop,
            });
            return PostReply{
                .status = response.status, .body = response.body, .headers = response.headers};
        } catch (const http::TransportError& e) {
            if (stop.stop_requested()) throw Stopped();
            problem = e.what();
            return std::nullopt;
        }
    };
}

OverpassClient::OverpassClient(std::vector<std::string> urls, Post post, Sleep sleep, Clock clock)
    : urls_(std::move(urls)),
      post_(std::move(post)),
      sleep_(std::move(sleep)),
      clock_(std::move(clock)) {
    if (!sleep_) {
        sleep_ = [](double s, const std::stop_token& stop) {
            std::mutex m;
            std::condition_variable_any cv;
            std::unique_lock lock(m);
            std::ignore =
                cv.wait_for(lock, stop, std::chrono::duration<double>(s), [] { return false; });
        };
    }
    if (!clock_) {
        clock_ = [] {
            return std::chrono::duration<double>(
                       std::chrono::steady_clock::now().time_since_epoch())
                .count();
        };
    }
}

std::vector<std::string> OverpassClient::mirrors() {
    const std::scoped_lock lock(state_mutex_);
    const double now = clock_();
    std::vector<std::string> up;
    for (const auto& url : urls_) {
        const auto it = down_until_.find(url);
        if (it == down_until_.end() || it->second <= now) up.push_back(url);
    }
    if (up.empty()) up = urls_;
    if (const auto it = std::ranges::find(up, preferred_); it != up.end())
        std::rotate(up.begin(), it, std::next(it));
    return up;
}

std::optional<PostReply> OverpassClient::post(const std::string& url, const std::string& query,
                                              std::string& problem, const std::stop_token& stop) {
    {
        const std::scoped_lock lock(gap_mutex_);
        const double wait = last_request_ + request_gap_s - clock_();
        if (wait > 0) sleep_(wait, stop);
        if (stop.stop_requested()) throw Stopped();
        last_request_ = clock_();
    }
    return post_(url, query, problem, stop);
}

std::optional<std::vector<Json>> OverpassClient::buildings(double south, double west, double north,
                                                           double east,
                                                           const std::stop_token& stop) {
    const auto elements = fetch(format_query(south, west, north, east), south, west, stop);
    if (!elements) return std::nullopt;
    std::vector<Json> out;
    for (const auto& element : *elements) out.push_back(simplify(element));
    return out;
}

std::optional<std::vector<Json>> OverpassClient::features(double south, double west, double north,
                                                          double east,
                                                          const std::stop_token& stop) {
    const auto elements =
        fetch(format_query(features_query, south, west, north, east), south, west, stop);
    if (!elements) return std::nullopt;
    std::vector<Json> out;
    for (const auto& element : *elements) {
        if (auto item = simplify_feature(element)) out.push_back(std::move(*item));
    }
    return out;
}

std::optional<Area> OverpassClient::area(double south, double west, double north, double east,
                                         const std::stop_token& stop) {
    const auto elements =
        fetch(format_query(area_query(), south, west, north, east), south, west, stop);
    if (!elements) return std::nullopt;
    Area out{.buildings = {}, .features = std::vector<Json>{}};
    for (const auto& element : *elements) {
        if (element.value("type", "") == "node") {
            if (auto item = simplify_feature(element)) out.features->push_back(std::move(*item));
        } else {
            out.buildings.push_back(simplify(element));
        }
    }
    return out;
}

std::optional<Json> OverpassClient::fetch(const std::string& query, double south, double west,
                                          const std::stop_token& stop) {
    std::vector<std::string> outcomes;
    for (const auto& url : mirrors()) {
        std::vector<std::string> answers;
        for (int attempt = 1; attempt <= attempts; ++attempt) {
            std::string problem;
            const auto response = post(url, query, problem, stop);
            if (response && response->status == 200) {
                const auto body = Json::parse(response->body, nullptr, false);
                if (body.is_discarded() || !body.is_object()) {
                    answers.emplace_back("bad JSON");
                    break;
                }
                {
                    const std::scoped_lock lock(state_mutex_);
                    down_until_.erase(url);
                    preferred_ = url;
                }
                return body.value("elements", Json::array());
            }
            if (!response) {
                answers.push_back(problem.empty() ? "transport error" : problem);
                break;
            }
            answers.push_back(std::format("HTTP {}", response->status));
            double delay = 0;
            if (response->status == 429) {
                const auto it = response->headers.find("retry-after");
                delay = retry_after(it == response->headers.end()
                                        ? std::nullopt
                                        : std::optional<std::string_view>(it->second));
            } else if (transient(response->status)) {
                delay = busy_backoff_s * attempt;
            } else {
                break;
            }
            if (attempt < attempts) {
                logger()->info("overpass {}: {}, retry in {:.0f} s", url, answers.back(), delay);
                sleep_(delay, stop);
                if (stop.stop_requested()) throw Stopped();
            }
        }
        {
            const std::scoped_lock lock(state_mutex_);
            down_until_[url] = clock_() + mirror_cooldown_s;
        }
        outcomes.push_back(std::format("{} {}", host(url), summary(answers)));
    }
    std::string joined;
    for (const auto& o : outcomes) joined += (joined.empty() ? "" : "; ") + o;
    logger()->warn("overpass: no answer for {:.2f},{:.2f}: {}", south, west, joined);
    return std::nullopt;
}

}  // namespace docksim::osm
