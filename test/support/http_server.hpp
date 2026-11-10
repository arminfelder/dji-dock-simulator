#pragma once

#include <httplib.h>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/x509v3.h>

#include <cctype>
#include <cstdio>
#include <filesystem>
#include <format>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

// An HTTP or HTTPS server on 127.0.0.1 for tests, over cpp-httplib. It records every request.
namespace docksim::test {

struct Seen {
    std::string method;
    std::string path;
    // Request line and headers as "Name: value" lines, names as the client sent them.
    std::string head;
    std::string body;
    httplib::Headers headers;

    [[nodiscard]] std::string header(const std::string& name) const {
        const auto it = headers.find(name);
        return it == headers.end() ? std::string{} : it->second;
    }
};

struct Reply {
    int status = 200;
    std::vector<std::pair<std::string, std::string>> headers;
    std::string body;
};

// A self-signed certificate for 127.0.0.1 and its key, as PEM files in a temporary directory.
class TestCertificate {
public:
    TestCertificate() {
        dir_ = std::filesystem::temp_directory_path() /
               std::format("docksim-tls-{}", std::random_device{}());
        std::filesystem::create_directories(dir_);
        const std::unique_ptr<EVP_PKEY, decltype(&EVP_PKEY_free)> key(EVP_EC_gen("P-256"),
                                                                      &EVP_PKEY_free);
        const std::unique_ptr<X509, decltype(&X509_free)> cert(X509_new(), &X509_free);
        if (!key || !cert) throw std::runtime_error("test certificate: OpenSSL allocation failed");
        X509_set_version(cert.get(), 2);
        ASN1_INTEGER_set(X509_get_serialNumber(cert.get()), 1);
        X509_gmtime_adj(X509_getm_notBefore(cert.get()), -60);
        X509_gmtime_adj(X509_getm_notAfter(cert.get()), 3600);
        X509_set_pubkey(cert.get(), key.get());
        X509_NAME* name = X509_get_subject_name(cert.get());
        X509_NAME_add_entry_by_txt(name, "CN", MBSTRING_ASC,
                                   reinterpret_cast<const unsigned char*>("127.0.0.1"), -1, -1, 0);
        X509_set_issuer_name(cert.get(), name);
        add_extension(cert.get(), NID_subject_alt_name, "IP:127.0.0.1");
        add_extension(cert.get(), NID_basic_constraints, "critical,CA:TRUE");
        if (X509_sign(cert.get(), key.get(), EVP_sha256()) == 0)
            throw std::runtime_error("test certificate: sign failed");
        write(cert_path(), [&](FILE* f) { return PEM_write_X509(f, cert.get()); });
        write(key_path(), [&](FILE* f) {
            return PEM_write_PrivateKey(f, key.get(), nullptr, nullptr, 0, nullptr, nullptr);
        });
    }
    TestCertificate(const TestCertificate&) = delete;
    TestCertificate& operator=(const TestCertificate&) = delete;
    TestCertificate(TestCertificate&&) = delete;
    TestCertificate& operator=(TestCertificate&&) = delete;
    ~TestCertificate() {
        std::error_code ignored;
        std::filesystem::remove_all(dir_, ignored);
    }

    [[nodiscard]] std::string cert_path() const { return (dir_ / "cert.pem").string(); }
    [[nodiscard]] std::string key_path() const { return (dir_ / "key.pem").string(); }

private:
    static void add_extension(X509* cert, int nid, const char* value) {
        X509V3_CTX ctx;
        X509V3_set_ctx_nodb(&ctx);
        X509V3_set_ctx(&ctx, cert, cert, nullptr, nullptr, 0);
        X509_EXTENSION* ext = X509V3_EXT_conf_nid(nullptr, &ctx, nid, value);
        if (ext == nullptr) throw std::runtime_error("test certificate: extension failed");
        X509_add_ext(cert, ext, -1);
        X509_EXTENSION_free(ext);
    }

    static void write(const std::string& path, const std::function<int(FILE*)>& out) {
        struct Close {
            void operator()(FILE* f) const noexcept { std::ignore = std::fclose(f); }
        };
        const std::unique_ptr<FILE, Close> f(std::fopen(path.c_str(), "wb"));
        if (!f || out(f.get()) == 0)
            throw std::runtime_error("test certificate: cannot write " + path);
    }

    std::filesystem::path dir_;
};

class Server {
public:
    // The stop token is set when the server shuts down, so that slow handlers can end.
    using Handler = std::function<Reply(const Seen&, const std::stop_token&)>;

    // With a certificate, the server speaks HTTPS.
    explicit Server(Handler handler, const TestCertificate* tls = nullptr)
        : handler_(std::move(handler)) {
        if (tls != nullptr) {
            server_ = std::make_unique<httplib::SSLServer>(tls->cert_path().c_str(),
                                                           tls->key_path().c_str());
            scheme_ = "https";
        } else {
            server_ = std::make_unique<httplib::Server>();
        }
        if (!server_->is_valid()) throw std::runtime_error("test server: invalid TLS setup");
        const auto route = [this](const httplib::Request& req, httplib::Response& res) {
            serve(req, res);
        };
        server_->Get(".*", route);
        server_->Post(".*", route);
        server_->Put(".*", route);
        server_->Patch(".*", route);
        server_->Delete(".*", route);
        server_->Options(".*", route);
        port_ = server_->bind_to_any_port("127.0.0.1");
        if (port_ <= 0) throw std::runtime_error("test server: bind failed");
        thread_ = std::thread([this] { server_->listen_after_bind(); });
        server_->wait_until_ready();
    }
    Server(const Server&) = delete;
    Server& operator=(const Server&) = delete;
    Server(Server&&) = delete;
    Server& operator=(Server&&) = delete;
    ~Server() {
        stop_.request_stop();
        server_->stop();
        if (thread_.joinable()) thread_.join();
    }

    [[nodiscard]] std::string url(std::string_view path) const {
        return std::format("{}://127.0.0.1:{}{}", scheme_, port_, path);
    }

    [[nodiscard]] std::vector<Seen> seen() const {
        const std::scoped_lock lock(mutex_);
        return seen_;
    }

private:
    static std::string lower(std::string s) {
        for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return s;
    }

    void serve(const httplib::Request& req, httplib::Response& res) {
        Seen seen{.method = req.method,
                  .path = req.path,
                  .head = {},
                  .body = req.body,
                  .headers = req.headers};
        seen.head = std::format("{} {}\r\n", req.method, req.path);
        for (const auto& [name, value] : req.headers)
            seen.head += std::format("{}: {}\r\n", name, value);
        {
            const std::scoped_lock lock(mutex_);
            seen_.push_back(seen);
        }
        const auto reply = handler_(seen, stop_.get_token());
        res.status = reply.status;
        std::string content_type = "text/plain";
        for (const auto& [name, value] : reply.headers) {
            if (lower(name) == "content-type") {
                content_type = value;
            } else {
                res.set_header(name, value);
            }
        }
        res.set_content(reply.body, content_type);
    }

    Handler handler_;
    std::unique_ptr<httplib::Server> server_;
    std::string scheme_ = "http";
    int port_ = 0;
    std::stop_source stop_;
    mutable std::mutex mutex_;
    std::vector<Seen> seen_;
    std::thread thread_;
};

}  // namespace docksim::test
