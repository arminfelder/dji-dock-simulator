#pragma once

#include <optional>
#include <string>
#include <string_view>

// Cloud API app license check, as the Dock 2 does it. The dock reads app_id,
// app_key and app_license from the config reply. The license is a base64 RSA
// PKCS#1 v1.5 signature over SHA-256(app_id + app_key). The dock verifies it
// with the DJI public key. This repository does not contain that key.
namespace docksim::license {

// Empty if public_key_pem is an RSA public key in PEM form, else the reason.
[[nodiscard]] std::optional<std::string> key_error(std::string_view public_key_pem);

// False also for a key that key_error refuses.
[[nodiscard]] bool verify(std::string_view public_key_pem, std::string_view app_id,
                          std::string_view app_key, std::string_view app_license);

}  // namespace docksim::license
