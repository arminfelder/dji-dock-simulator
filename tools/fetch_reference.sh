#!/usr/bin/env bash
# Gets the DJI Cloud API documentation for offline use. The documentation is
# vendor material, so this repository does not hold a copy.
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$here/reference"
if [ -d "$here/reference/Cloud-API-Doc/.git" ]; then
  git -C "$here/reference/Cloud-API-Doc" pull --ff-only
else
  git clone --depth 1 https://github.com/dji-sdk/Cloud-API-Doc.git \
    "$here/reference/Cloud-API-Doc"
fi
