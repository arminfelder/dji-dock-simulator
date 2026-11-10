# Third-party components

dock-sim is GPL-3.0-or-later (see LICENSE). Every component below is compatible
with GPLv3.
The release binary links these components statically. Each keeps its license.

| Component | Version | License | Use |
|---|---|---|---|
| OpenSSL | 3.6.x (openssl-cmake 3.6.2) | Apache-2.0 | TLS, license check, SHA/HMAC |
| curl | 8.23.0 | curl (MIT-style) | HTTP: tiles, Overpass, KMZ, S3, login |
| cpr | 1.15.1 | MIT | C++ HTTP client over curl |
| AWS SDK for C++ (core, s3; bundles TinyXML2 zlib license, cJSON MIT) | 1.11.924 | Apache-2.0 | S3 PutObject for media upload |
| AWS Common Runtime (aws-crt-cpp, aws-c-*, aws-checksums) | 0.44.2 (as pinned by the SDK) | Apache-2.0 | SDK runtime |
| s2n-tls | as pinned by the SDK | Apache-2.0 | Linked by the CRT; the S3 upload uses curl and OpenSSL |
| Eclipse Paho MQTT C | 1.3.16 | EPL-2.0 or EDL-1.0 (BSD-3-Clause); used under EDL-1.0 | MQTT 3.1.1 |
| asio (standalone) | 1.38.2 | BSL-1.0 | Coroutines, timers |
| nlohmann/json | 3.12.0 | MIT | JSON |
| rapidyaml | 0.16.0 | MIT | YAML config |
| spdlog | 1.17.0 | MIT | Logging |
| CLI11 | 2.7.2 | BSD-3-Clause | Command line |
| pugixml | 1.16 | MIT | WPML/KML |
| Exiv2 | 0.28.9 | GPL-2.0-or-later | Photo EXIF and XMP |
| Adobe XMP Toolkit (bundled in Exiv2) | Exiv2 copy | BSD-3-Clause (notice below) | XMP serialization |
| Expat | 2.10.0 | MIT | XML parser for the Exiv2 XMP support |
| OpenCV (core, imgproc; flann, geometry as dependencies) | 5.1.0 | Apache-2.0 | Tree-mask morphology (erode, dilate, box filter) |
| zlib (bundled in OpenCV) | 1.3.2 | zlib license | OpenCV core; also linked by Drogon |
| Drogon | 1.9.14 | MIT | Instructor REST API and WebSocket |
| trantor (bundled in Drogon) | Drogon copy | BSD-3-Clause | Drogon network library |
| JsonCpp | 1.9.8 | MIT or public domain | Required by Drogon |
| libuuid (util-linux, Ubuntu uuid-dev) | system | BSD-3-Clause | Required by Drogon |
| HarfBuzz (bundled in OpenCV) | OpenCV copy | MIT ("Old MIT") | Linked by OpenCV imgproc; not used by dock-sim |
| miniz | 3.1.2 | MIT | KMZ zip, TIFF deflate |
| mp-units | 2.5.0 | MIT | Physical units (header-only) |
| stb (image, image_write, truetype) | a7d4c21 | MIT or public domain | JPEG/PNG, overlay text |
| earcut.hpp | 3.2.4 | ISC | Roof triangulation |
| volk | 1.4.350 | MIT | Vulkan loader (dlopen) |
| Vulkan-Headers | 1.4.350 | Apache-2.0 or MIT; used under MIT | Vulkan API |
| Vulkan Memory Allocator | 3.4.0 | MIT | GPU memory |
| FFmpeg (libavformat, libavcodec, libavutil, libswscale) | 8.1.3 | LGPL-2.1-or-later (configured without --enable-gpl/nonfree) | Live video: H.264/AAC encode, FLV/RTSP mux |
| openh264 | 2.6.0 | BSD-2-Clause | H.264 encoder. Built from source: no Cisco patent license. Internal use only. |
| nv-codec-headers | 12.2.72.0 | MIT | NVENC API headers; the driver loads at run time |
| libdatachannel (+ libjuice, usrsctp, libsrtp, plog) | 0.24.6 | MPL-2.0 (libdatachannel, libjuice), BSD-3-Clause (usrsctp, libsrtp), MIT (plog) | WHIP / WebRTC |

Data and build-time tools:

| Component | License | Use |
|---|---|---|
| DejaVu Sans Mono | Bitstream Vera license (lib/resources/fonts/LICENSE-DejaVu.txt) | Embedded overlay font |
| Poly Haven wall materials | CC0 (lib/resources/materials/SOURCES.md) | Embedded textures |
| Slang compiler (slangc) | Apache-2.0 with LLVM exception | Build time only; not linked |
| Catch2 | BSL-1.0 | Tests only |
| cpp-httplib 0.65.0 | MIT | Tests only: HTTP and HTTPS test servers |

Runtime data the program downloads (orthophoto, DEM, OpenStreetMap, BEV laser
scan) keeps the terms of its provider; see README.md.

## Notices (Apache-2.0, section 4(d))

AWS SDK for C++
Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
This product includes software developed by Amazon Technologies, Inc (http://www.amazon.com/).
It includes third party software: XML parsing and utility functions from TinyXml2 - Lee Thomason;
JSON support from cJSON - Copyright (c) 2009-2017 Dave Gamble and cJSON contributors.

AWS Crt Cpp
Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.

s2n
Copyright 2014-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.

Adobe XMP Toolkit (in Exiv2)

Copyright (c) 1999 - 2008, Adobe Systems Incorporated

The BSD License

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
 
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
 
* Neither the name of Adobe Systems Incorporated, nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
