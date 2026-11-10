#include "util/digest.hpp"

#include <miniz.h>
#include <openssl/evp.h>

#include <array>
#include <format>
#include <stdexcept>

namespace docksim::digest {

namespace {

std::string hex_digest(const EVP_MD* md, std::span<const std::byte> data) {
    std::array<unsigned char, EVP_MAX_MD_SIZE> out{};
    unsigned int size = 0;
    if (EVP_Digest(data.data(), data.size(), out.data(), &size, md, nullptr) != 1)
        throw std::runtime_error("digest failed");
    std::string text;
    text.reserve(static_cast<std::size_t>(size) * 2);
    for (unsigned int i = 0; i < size; ++i) text += std::format("{:02x}", out[i]);
    return text;
}

}  // namespace

std::string sha256_hex(std::span<const std::byte> data) {
    return hex_digest(EVP_sha256(), data);
}

std::string md5_hex(std::span<const std::byte> data) {
    return hex_digest(EVP_md5(), data);
}

std::span<const std::byte> bytes(std::string_view text) {
    return std::as_bytes(std::span(text.data(), text.size()));
}

std::vector<std::byte> zip(const std::vector<std::pair<std::string, std::string>>& members) {
    mz_zip_archive archive{};
    if (mz_zip_writer_init_heap(&archive, 0, 0) == 0) throw std::runtime_error("zip init failed");
    for (const auto& [name, content] : members) {
        if (mz_zip_writer_add_mem(&archive, name.c_str(), content.data(), content.size(),
                                  static_cast<mz_uint>(MZ_DEFAULT_LEVEL)) == 0) {
            mz_zip_writer_end(&archive);
            throw std::runtime_error(std::format("zip: cannot add {}", name));
        }
    }
    void* buffer = nullptr;
    std::size_t size = 0;
    if (mz_zip_writer_finalize_heap_archive(&archive, &buffer, &size) == 0) {
        mz_zip_writer_end(&archive);
        throw std::runtime_error("zip finalize failed");
    }
    const auto* first = static_cast<const std::byte*>(buffer);
    std::vector<std::byte> out(first, first + size);
    // finalize hands the buffer to the caller.
    mz_free(buffer);
    mz_zip_writer_end(&archive);
    return out;
}

}  // namespace docksim::digest
