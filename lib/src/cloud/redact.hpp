#pragma once

#include <string>
#include <string_view>

// Keep cloud-issued secrets out of the log: stream tokens, presigned S3 signatures.
namespace docksim::redact {

// The URL with its password and secret query values replaced by ***, and without its fragment.
// Path and other keys stay. A query key is secret if it holds token, signature, credential,
// password, secret or key, in any case.
[[nodiscard]] std::string safe_url(std::string_view text);

}  // namespace docksim::redact
