#include "media/s3.hpp"

#include "log/log.hpp"

#include <aws/core/Aws.h>
#include <aws/core/auth/AWSCredentials.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/model/PutObjectRequest.h>

#include <format>
#include <sstream>
#include <stdexcept>

namespace docksim::s3 {

namespace {

// Aws::InitAPI must run once before the first client, and ShutdownAPI after the last one.
class SdkLifetime {
public:
    SdkLifetime() { Aws::InitAPI(options_); }
    SdkLifetime(const SdkLifetime&) = delete;
    SdkLifetime& operator=(const SdkLifetime&) = delete;
    SdkLifetime(SdkLifetime&&) = delete;
    SdkLifetime& operator=(SdkLifetime&&) = delete;
    ~SdkLifetime() { Aws::ShutdownAPI(options_); }

private:
    Aws::SDKOptions options_;
};

void ensure_sdk() {
    static const SdkLifetime sdk;
}

std::string trim_slashes(std::string s) {
    while (!s.empty() && s.front() == '/') s.erase(0, 1);
    while (!s.empty() && s.back() == '/') s.pop_back();
    return s;
}

}  // namespace

S3Storage::S3Storage(const Json& output, const MediaConfig& cfg)
    : bucket_(output.at("bucket").get<std::string>()),
      prefix_(trim_slashes(output.value("object_key_prefix", ""))) {
    ensure_sdk();
    std::string region = output.value("region", "");
    if (region.empty()) region = "us-east-1";
    std::string endpoint = cfg.endpoint_override.value_or(output.value("endpoint", ""));
    while (endpoint.ends_with('/')) endpoint.pop_back();

    // The SDK must not ask EC2 metadata or a profile for settings that storage_config_get gives.
    Aws::S3::S3ClientConfiguration config({.shouldDisableIMDS = true});
    config.region = region;
    if (!endpoint.empty()) config.endpointOverride = endpoint;
    config.useVirtualAddressing = false;
    config.verifySSL = cfg.verify_tls;
    config.connectTimeoutMs = 10'000;
    config.requestTimeoutMs = 30'000;
    // Optional checksums use aws-chunked uploads, and some S3-compatible servers refuse them.
    config.checksumConfig.requestChecksumCalculation =
        Aws::Client::RequestChecksumCalculation::WHEN_REQUIRED;
    config.checksumConfig.responseChecksumValidation =
        Aws::Client::ResponseChecksumValidation::WHEN_REQUIRED;

    const auto& c = output.at("credentials");
    const Aws::Auth::AWSCredentials credentials(c.at("access_key_id").get<std::string>(),
                                                c.at("access_key_secret").get<std::string>(),
                                                c.value("security_token", ""));
    client_ = std::make_unique<Aws::S3::S3Client>(credentials, nullptr, config);
}

S3Storage::~S3Storage() = default;

std::string S3Storage::object_key(std::string_view name) const {
    return prefix_.empty() ? std::string(name) : std::format("{}/{}", prefix_, name);
}

void S3Storage::put(const std::string& key, std::span<const std::byte> blob,
                    std::string_view content_type) {
    log::get("dock_sim.media")->info("upload s3://{}/{} ({} bytes)", bucket_, key, blob.size());
    Aws::S3::Model::PutObjectRequest request;
    request.SetBucket(bucket_);
    request.SetKey(key);
    request.SetContentType(std::string(content_type));
    auto body = std::make_shared<std::stringstream>(
        std::string(reinterpret_cast<const char*>(blob.data()), blob.size()));
    request.SetBody(body);
    const auto outcome = client_->PutObject(request);
    if (!outcome.IsSuccess()) {
        const auto& error = outcome.GetError();
        throw std::runtime_error(std::format("S3 PUT {}: HTTP {} {}: {}", key,
                                             static_cast<int>(error.GetResponseCode()),
                                             error.GetExceptionName(), error.GetMessage()));
    }
}

}  // namespace docksim::s3
