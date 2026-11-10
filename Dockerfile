# syntax=docker/dockerfile:1.7
# Build and runtime share one Ubuntu release so the glibc versions match.
ARG UBUNTU=26.04

# Angular 21 needs Node 20.19, 22.12 or 24 and later.
FROM node:24-slim AS web
WORKDIR /web
COPY web/package.json web/package-lock.json ./
RUN --mount=type=cache,target=/root/.npm npm ci --no-audit --no-fund
COPY web/ ./
RUN npx ng build

FROM ubuntu:${UBUNTU} AS build
ARG DEBIAN_FRONTEND=noninteractive
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build git ca-certificates nasm perl pkg-config python3 uuid-dev
WORKDIR /src
COPY . .
# s2n-tls in the AWS SDK probes libcrypto when CMake configures.
# Build OpenSSL first, then configure again, so that the probes find it.
RUN --mount=type=cache,target=/root/.cache/CPM \
    cmake --preset gcc-release-static -DCPM_SOURCE_CACHE=/root/.cache/CPM \
 && cmake --build --preset gcc-release-static --target openssl-build \
 && cmake --preset gcc-release-static -DCPM_SOURCE_CACHE=/root/.cache/CPM \
 && cmake --build --preset gcc-release-static \
 && ctest --preset gcc-release-static \
 && cmake --install out/cmake/gcc-release-static --prefix /opt/dock-sim --strip

FROM ubuntu:${UBUNTU} AS runtime
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates tzdata \
 && rm -rf /var/lib/apt/lists/* \
 && useradd --system --create-home --home-dir /var/lib/dock-sim dock-sim
COPY --from=build /opt/dock-sim/bin/dock-sim /usr/local/bin/dock-sim
COPY --from=web /web/dist/dock-sim/browser /usr/local/share/dock-sim/web
COPY LICENSE COPYING THIRD_PARTY.md /usr/share/doc/dock-sim/
USER dock-sim
WORKDIR /var/lib/dock-sim
# The instructor console. Set DOCKSIM_API_TOKEN: the API listens on every interface here.
ENV DOCKSIM_API__BIND=0.0.0.0
EXPOSE 8080
ENTRYPOINT ["dock-sim"]
CMD ["--config", "/config/config.yaml"]

# Same binary. The Vulkan and OpenCL loaders are opened at run time,
# so this stage only adds the driver plumbing.
FROM runtime AS runtime-gpu
USER root
RUN apt-get update && apt-get install -y --no-install-recommends libvulkan1 mesa-vulkan-drivers \
 && rm -rf /var/lib/apt/lists/* \
 && mkdir -p /etc/OpenCL/vendors
ENV NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics \
    NVIDIA_VISIBLE_DEVICES=all
USER dock-sim

FROM runtime
