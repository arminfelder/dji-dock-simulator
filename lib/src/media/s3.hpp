#pragma once

#include "media/uploads.hpp"

#include <docksim/config.hpp>

#include <memory>
#include <string>
#include <string_view>

namespace Aws::S3 {
class S3Client;
}

// S3 PutObject with the credentials that storage_config_get returns, over the AWS SDK for C++.
// Path-style addressing, so MinIO and other S3-compatible servers work.
namespace docksim::s3 {

class S3Storage final : public media::Storage {
public:
    // output is the storage_config_get output; endpoint_override wins over its endpoint.
    S3Storage(const Json& output, const MediaConfig& cfg);
    S3Storage(const S3Storage&) = delete;
    S3Storage& operator=(const S3Storage&) = delete;
    S3Storage(S3Storage&&) = delete;
    S3Storage& operator=(S3Storage&&) = delete;
    ~S3Storage() override;

    [[nodiscard]] std::string object_key(std::string_view name) const override;
    void put(const std::string& key, std::span<const std::byte> blob,
             std::string_view content_type) override;

private:
    std::string bucket_;
    std::string prefix_;
    std::unique_ptr<Aws::S3::S3Client> client_;
};

}  // namespace docksim::s3
