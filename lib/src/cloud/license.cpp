#include "cloud/license.hpp"

#include "util/base64.hpp"

#include <openssl/evp.h>
#include <openssl/pem.h>

#include <memory>
#include <string>

namespace docksim::license {

namespace {

struct BioFree {
    void operator()(BIO* b) const noexcept { BIO_free(b); }
};
struct KeyFree {
    void operator()(EVP_PKEY* k) const noexcept { EVP_PKEY_free(k); }
};
struct MdCtxFree {
    void operator()(EVP_MD_CTX* c) const noexcept { EVP_MD_CTX_free(c); }
};

using Key = std::unique_ptr<EVP_PKEY, KeyFree>;

Key read_key(std::string_view pem) {
    const std::unique_ptr<BIO, BioFree> bio(
        BIO_new_mem_buf(pem.data(), static_cast<int>(pem.size())));
    if (!bio) return nullptr;
    return Key(PEM_read_bio_PUBKEY(bio.get(), nullptr, nullptr, nullptr));
}

}  // namespace

std::optional<std::string> key_error(std::string_view public_key_pem) {
    const auto key = read_key(public_key_pem);
    if (!key) return "not a PEM public key";
    if (EVP_PKEY_base_id(key.get()) != EVP_PKEY_RSA) return "not an RSA key";
    return std::nullopt;
}

bool verify(std::string_view public_key_pem, std::string_view app_id, std::string_view app_key,
            std::string_view app_license) {
    // The dock decodes leniently: characters outside the alphabet are dropped.
    const auto signature = base64::decode_lenient(app_license);
    if (!signature || signature->empty()) return false;

    const auto key = read_key(public_key_pem);
    const std::unique_ptr<EVP_MD_CTX, MdCtxFree> ctx(EVP_MD_CTX_new());
    if (!key || EVP_PKEY_base_id(key.get()) != EVP_PKEY_RSA || !ctx) return false;

    const std::string message = std::string(app_id) + std::string(app_key);
    if (EVP_DigestVerifyInit(ctx.get(), nullptr, EVP_sha256(), nullptr, key.get()) != 1) {
        return false;
    }
    return EVP_DigestVerify(ctx.get(), signature->data(), signature->size(),
                            reinterpret_cast<const unsigned char*>(message.data()),
                            message.size()) == 1;
}

}  // namespace docksim::license
