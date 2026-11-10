include(CPM)

# OpenSSL 3.x on purpose: OpenSSL 4 removes APIs that Paho and curl may still use.
CPMAddPackage(
    NAME openssl-cmake
    SYSTEM YES
    GITHUB_REPOSITORY jimmy-park/openssl-cmake
    GIT_TAG 3.6.2
    OPTIONS "OPENSSL_CONFIGURE_OPTIONS no-docs\\\\;no-apps\\\\;no-legacy\\\\;no-engine")

# OpenSSL::SSL and OpenSSL::Crypto exist now, so find_package(OpenSSL) in
# the packages below resolves to the static build above.
CPMAddPackage(
    NAME CURL
    SYSTEM YES
    GITHUB_REPOSITORY curl/curl
    GIT_TAG curl-8_23_0
    OPTIONS "BUILD_CURL_EXE OFF" "BUILD_SHARED_LIBS OFF" "BUILD_STATIC_LIBS ON" "BUILD_TESTING OFF"
            "BUILD_LIBCURL_DOCS OFF" "BUILD_MISC_DOCS OFF" "ENABLE_CURL_MANUAL OFF" "CURL_DISABLE_INSTALL ON"
            "HTTP_ONLY ON" "CURL_USE_OPENSSL ON" "CURL_USE_LIBPSL OFF" "CURL_USE_LIBSSH2 OFF"
            "USE_LIBIDN2 OFF" "USE_NGHTTP2 OFF" "CURL_BROTLI OFF" "CURL_ZSTD OFF" "CURL_ZLIB OFF"
            "ENABLE_ARES OFF" "USE_LIBRTMP OFF" "CURL_DISABLE_LDAP ON" "CURL_DISABLE_LDAPS ON"
            "USE_OPENLDAP OFF" "CURL_USE_GSSAPI OFF" "USE_WIN32_LDAP OFF")

CPMAddPackage(
    NAME paho-mqtt-c
    SYSTEM YES
    GITHUB_REPOSITORY eclipse-paho/paho.mqtt.c
    GIT_TAG v1.3.16
    OPTIONS "PAHO_WITH_SSL ON" "PAHO_BUILD_STATIC ON" "PAHO_BUILD_SHARED OFF" "PAHO_ENABLE_TESTING OFF"
            "PAHO_BUILD_SAMPLES OFF" "PAHO_BUILD_DOCUMENTATION OFF" "PAHO_ENABLE_CPACK OFF")

# OpenSSL::SSL and OpenSSL::Crypto are IMPORTED targets with no build edge to
# the OpenSSL build. Without this, a clean parallel build compiles curl and
# Paho before opensslconf.h exists ("impossible API compatibility level").
function(docksim_build_after_openssl directory)
    get_property(targets DIRECTORY "${directory}" PROPERTY BUILDSYSTEM_TARGETS)
    foreach(target IN LISTS targets)
        get_target_property(type ${target} TYPE)
        if(NOT type STREQUAL "INTERFACE_LIBRARY" AND NOT type STREQUAL "UTILITY")
            add_dependencies(${target} openssl-build)
        endif()
    endforeach()
endfunction()
docksim_build_after_openssl("${CURL_SOURCE_DIR}/lib")

# AWS SDK for C++, S3 only (Apache-2.0). It uses our static OpenSSL and curl.
# s2n-tls in the AWS CRT takes a target named crypto as its libcrypto.
# Without it, the CRT builds AWS-LC, and its unprefixed symbols clash with OpenSSL.
# The s2n feature probes run try_compile; a link chain with Threads::Threads breaks them.
get_target_property(DOCKSIM_CRYPTO_LIBRARY OpenSSL::Crypto IMPORTED_LOCATION)
get_target_property(DOCKSIM_CRYPTO_INCLUDE OpenSSL::Crypto INTERFACE_INCLUDE_DIRECTORIES)
add_library(crypto STATIC IMPORTED GLOBAL)
set_target_properties(crypto PROPERTIES
    IMPORTED_LOCATION "${DOCKSIM_CRYPTO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${DOCKSIM_CRYPTO_INCLUDE}"
    INTERFACE_LINK_LIBRARIES "-pthread;${CMAKE_DL_LIBS}")
# The SDK exports its targets and uses curl in try_compile checks. A build target or a link chain
# with Threads::Threads breaks both, so the SDK gets plain imported files.
get_target_property(DOCKSIM_SSL_LIBRARY OpenSSL::SSL IMPORTED_LOCATION)
get_target_property(DOCKSIM_CURL_NAME libcurl_static OUTPUT_NAME)
add_library(docksim_curl STATIC IMPORTED GLOBAL)
set_target_properties(docksim_curl PROPERTIES
    IMPORTED_CONFIGURATIONS "DEBUG;RELEASE"
    IMPORTED_LOCATION_DEBUG "${CURL_BINARY_DIR}/lib/${DOCKSIM_CURL_NAME}-d.a"
    IMPORTED_LOCATION_RELEASE "${CURL_BINARY_DIR}/lib/${DOCKSIM_CURL_NAME}.a"
    MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE
    MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE
    INTERFACE_INCLUDE_DIRECTORIES "${CURL_SOURCE_DIR}/include"
    INTERFACE_LINK_LIBRARIES "${DOCKSIM_SSL_LIBRARY};crypto")
# The SDK sets CMP0077 to OLD, so option() ignores normal variables. Cache entries take effect.
set(BUILD_ONLY "s3" CACHE STRING "AWS SDK services to build")
set(ENABLE_TESTING OFF CACHE BOOL "AWS SDK tests")
set(AUTORUN_UNIT_TESTS OFF CACHE BOOL "AWS SDK tests")
set(FORCE_SHARED_CRT OFF CACHE BOOL "AWS SDK runtime linkage")
set(USE_OPENSSL ON CACHE BOOL "AWS SDK and CRT use OpenSSL libcrypto")
# FORCE: with ON the SDK links ZLIB::ZLIB from whatever zlib paths the cache holds.
set(ENABLE_ZLIB_REQUEST_COMPRESSION OFF CACHE BOOL "AWS SDK request compression" FORCE)
set(AWS_SDK_WARNINGS_ARE_ERRORS OFF CACHE BOOL "AWS SDK warnings")
set(ENABLE_CURL_LOGGING OFF CACHE BOOL "AWS SDK curl log")
set(CURL_NO_CURL_CMAKE ON)
# The SDK runs curl to probe it while curl is not built yet. Our curl has no HTTP/2 and has OpenSSL.
set(CURL_HAS_H2 0 CACHE INTERNAL "")
set(CURL_HAS_TLS_PROXY 1 CACHE INTERNAL "")
set(CURL_LIBRARY docksim_curl CACHE STRING "curl for the AWS SDK")
set(CURL_INCLUDE_DIR "${CURL_SOURCE_DIR}/include" CACHE PATH "curl headers for the AWS SDK")
# A BUILD_ZLIB cache entry makes the SDK build its own zlib, which fails on CMake 4.
# An interrupted configure can leave the OpenCV entry below in the cache.
unset(BUILD_ZLIB CACHE)
# That failed zlib build leaves INTERNAL entries that point to /usr/local/external-install.
if(DEFINED CACHE{ZLIB_INSTALL_DIR})
    foreach(stale IN ITEMS ZLIB_SOURCE_DIR ZLIB_INSTALL_DIR ZLIB_INCLUDE_DIR ZLIB_LIBRARY_DIR
                           ZLIB_INCLUDE_FLAGS ZLIB_LINKER_FLAGS)
        unset(${stale} CACHE)
    endforeach()
endif()
CPMAddPackage(
    NAME aws-sdk-cpp
    SYSTEM YES
    GITHUB_REPOSITORY aws/aws-sdk-cpp
    GIT_TAG 1.11.924
    GIT_SHALLOW YES)
if(NOT EXISTS "${DOCKSIM_CRYPTO_LIBRARY}")
    message(WARNING "OpenSSL is not built yet, so the s2n-tls libcrypto probes failed. "
                    "Run 'cmake --build <dir> --target openssl-build', then configure again.")
endif()
foreach(dir IN ITEMS crt/aws-crt-cpp crt/aws-crt-cpp/crt/s2n crt/aws-crt-cpp/crt/aws-c-cal
                     crt/aws-crt-cpp/crt/aws-c-io crt/aws-crt-cpp/crt/aws-c-auth crt/aws-crt-cpp/crt/aws-c-http
                     crt/aws-crt-cpp/crt/aws-c-s3 src/aws-cpp-sdk-core generated/src/aws-cpp-sdk-s3)
    docksim_build_after_openssl("${aws-sdk-cpp_SOURCE_DIR}/${dir}")
endforeach()

# C++ wrapper over the curl target above (MIT); it does not fetch its own curl.
CPMAddPackage(
    NAME cpr
    SYSTEM YES
    GITHUB_REPOSITORY libcpr/cpr
    GIT_TAG 1.15.1
    OPTIONS "CPR_USE_EXISTING_CURL_TARGET ON" "CPR_FORCE_OPENSSL_BACKEND ON" "CPR_CURL_NOSIGNAL ON"
            "CPR_CURL_USE_LIBPSL OFF" "CPR_ENABLE_INSTALL OFF" "CPR_BUILD_TESTS OFF" "BUILD_SHARED_LIBS OFF")
docksim_build_after_openssl("${cpr_SOURCE_DIR}/cpr")
docksim_build_after_openssl("${paho-mqtt-c_SOURCE_DIR}/src")

CPMAddPackage(
    NAME asio
    SYSTEM YES
    GITHUB_REPOSITORY chriskohlhoff/asio
    GIT_TAG asio-1-38-2
    DOWNLOAD_ONLY YES)
add_library(asio INTERFACE)
add_library(asio::asio ALIAS asio)
target_include_directories(asio SYSTEM INTERFACE "${asio_SOURCE_DIR}/asio/include")
target_compile_definitions(asio INTERFACE ASIO_STANDALONE ASIO_NO_DEPRECATED)
find_package(Threads REQUIRED)
target_link_libraries(asio INTERFACE Threads::Threads)

CPMAddPackage(
    NAME nlohmann_json
    SYSTEM YES
    GITHUB_REPOSITORY nlohmann/json
    VERSION 3.12.0
    OPTIONS "JSON_BuildTests OFF" "JSON_Install OFF")

CPMAddPackage(
    NAME ryml
    SYSTEM YES
    GITHUB_REPOSITORY biojppm/rapidyaml
    GIT_TAG v0.16.0
    OPTIONS "RYML_BUILD_TESTS OFF" "RYML_DEFAULT_CALLBACK_USES_EXCEPTIONS ON" "RYML_INSTALL OFF")

CPMAddPackage(
    NAME spdlog
    SYSTEM YES
    GITHUB_REPOSITORY gabime/spdlog
    VERSION 1.17.0
    OPTIONS "SPDLOG_USE_STD_FORMAT ON" "SPDLOG_INSTALL OFF")

# Header-only physical units (MIT). Contract checks off, so no gsl-lite dependency.
CPMAddPackage(
    NAME mp-units
    SYSTEM YES
    GITHUB_REPOSITORY mpusz/mp-units
    GIT_TAG v2.5.0
    SOURCE_SUBDIR src
    OPTIONS "MP_UNITS_API_CONTRACTS NONE" "MP_UNITS_API_STD_FORMAT ON" "MP_UNITS_BUILD_INSTALL OFF"
            "MP_UNITS_BUILD_CXX_MODULES OFF")

# OpenCV core and imgproc only, for the tree-mask morphology (Apache-2.0). Bundled zlib, so no
# system libz; no codecs, GUI, video, OpenCL, downloaded binaries (IPP, ITT) or run-time plugins (TBB, OpenMP).
foreach(off IN ITEMS BUILD_SHARED_LIBS BUILD_TESTS BUILD_PERF_TESTS BUILD_EXAMPLES BUILD_DOCS BUILD_opencv_apps
                     BUILD_JAVA BUILD_opencv_python3 BUILD_PROTOBUF WITH_IPP WITH_ITT WITH_OPENCL WITH_LAPACK WITH_EIGEN
                     WITH_PROTOBUF WITH_ADE WITH_FFMPEG WITH_GSTREAMER WITH_GTK WITH_QT WITH_V4L WITH_VA WITH_VA_INTEL
                     WITH_JPEG WITH_PNG WITH_TIFF WITH_WEBP WITH_AVIF WITH_OPENJPEG WITH_JASPER WITH_OPENEXR WITH_IMGCODEC_GIF
                     WITH_IMGCODEC_HDR WITH_IMGCODEC_SUNRASTER WITH_IMGCODEC_PXM WITH_IMGCODEC_PFM WITH_CAROTENE
                     WITH_KLEIDICV WITH_FLATBUFFERS WITH_OBSENSOR WITH_1394 WITH_VTK WITH_CUDA CV_TRACE ENABLE_CCACHE)
    set(${off} OFF CACHE BOOL "OpenCV feature")
endforeach()
# BUILD_ZLIB is also an AWS SDK switch; set it for OpenCV only, and remove it after.
set(BUILD_ZLIB ON CACHE BOOL "OpenCV bundled zlib")
set(BUILD_LIST "core,imgproc" CACHE STRING "OpenCV modules")
# OpenCV caches EXECUTABLE_OUTPUT_PATH as <build>/bin, which moves every executable of the project.
# The cache entry stays empty (the default places apps/dock-sim/, test/); OpenCV, which needs a value,
# gets a normal variable that is removed again after it.
set(EXECUTABLE_OUTPUT_PATH "" CACHE PATH "Output directory for applications" FORCE)
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/_deps/opencv-build/bin")
# FORCE, because build trees configured before have it ON; it loads TBB or OpenMP plugins at run time.
set(PARALLEL_ENABLE_PLUGINS OFF CACHE BOOL "OpenCV parallel plugins" FORCE)
CPMAddPackage(
    NAME opencv
    SYSTEM YES
    GITHUB_REPOSITORY opencv/opencv
    GIT_TAG 5.1.0
    GIT_SHALLOW YES)
unset(BUILD_ZLIB CACHE)
unset(EXECUTABLE_OUTPUT_PATH)
# OpenCV adds its module headers as PRIVATE includes; consumers in this build need them too.
# opencv_modules.hpp and cvconfig.h go to the top build directory.
foreach(module IN ITEMS core imgproc)
    target_include_directories(opencv_${module} SYSTEM INTERFACE
        "$<BUILD_INTERFACE:${opencv_SOURCE_DIR}/modules/${module}/include>")
endforeach()
target_include_directories(opencv_core SYSTEM INTERFACE "$<BUILD_INTERFACE:${opencv_SOURCE_DIR}/include>"
                                                        "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>")

# XML parser for the Exiv2 XMP support (MIT).
CPMAddPackage(
    NAME expat
    SYSTEM YES
    GITHUB_REPOSITORY libexpat/libexpat
    GIT_TAG R_2_10_0
    SOURCE_SUBDIR expat
    OPTIONS "EXPAT_SHARED_LIBS OFF" "EXPAT_BUILD_TOOLS OFF" "EXPAT_BUILD_EXAMPLES OFF" "EXPAT_BUILD_TESTS OFF"
            "EXPAT_BUILD_DOCS OFF" "EXPAT_BUILD_PKGCONFIG OFF" "EXPAT_ENABLE_INSTALL OFF")

# EXIF and XMP for photos (GPL-2.0-or-later). Exiv2 exports its targets, so expat must reach it as
# an imported file. find_package(EXPAT) finds the CPM redirect of expat, which has no EXPAT::EXPAT.
set(EXPAT_INCLUDE_DIR "${expat_SOURCE_DIR}/expat/lib" CACHE PATH "expat headers for Exiv2")
set(EXPAT_LIBRARY "${expat_BINARY_DIR}/libexpat.a" CACHE FILEPATH "expat for Exiv2")
add_library(EXPAT::EXPAT STATIC IMPORTED GLOBAL)
set_target_properties(EXPAT::EXPAT PROPERTIES
    IMPORTED_LOCATION "${EXPAT_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${EXPAT_INCLUDE_DIR}")
# Exiv2 asks for CMake 3.11, so option() ignores normal variables. Cache entries take effect.
foreach(off IN ITEMS EXIV2_ENABLE_PNG EXIV2_ENABLE_BMFF EXIV2_ENABLE_BROTLI EXIV2_ENABLE_VIDEO EXIV2_ENABLE_INIH
                     EXIV2_ENABLE_LENSDATA EXIV2_ENABLE_NLS EXIV2_ENABLE_WEBREADY EXIV2_BUILD_EXIV2_COMMAND
                     EXIV2_BUILD_SAMPLES EXIV2_BUILD_UNIT_TESTS EXIV2_BUILD_DOC)
    set(${off} OFF CACHE BOOL "Exiv2 feature")
endforeach()
set(EXIV2_ENABLE_XMP ON CACHE BOOL "Exiv2 XMP support")
CPMAddPackage(
    NAME exiv2
    SYSTEM YES
    GITHUB_REPOSITORY Exiv2/exiv2
    GIT_TAG v0.28.9)
add_dependencies(exiv2-xmp expat)
# Exiv2 writes exv_conf.h and exiv2lib_export.h to the top build directory.
target_include_directories(exiv2lib SYSTEM INTERFACE "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>")

CPMAddPackage(
    NAME pugixml
    SYSTEM YES
    GITHUB_REPOSITORY zeux/pugixml
    VERSION 1.16
    OPTIONS "PUGIXML_BUILD_TESTS OFF" "PUGIXML_INSTALL OFF")

CPMAddPackage(
    NAME miniz
    SYSTEM YES
    GITHUB_REPOSITORY richgel999/miniz
    GIT_TAG 3.1.2
    OPTIONS "BUILD_EXAMPLES OFF" "BUILD_FUZZERS OFF" "BUILD_TESTS OFF" "INSTALL_PROJECT OFF")

# Header-only: stb_image (JPEG/PNG decode), stb_image_write (JPEG encode)
# and stb_truetype (overlay text).
CPMAddPackage(
    NAME stb
    GITHUB_REPOSITORY nothings/stb
    GIT_TAG a7d4c2175630e1ea5b6f8dd00ab96fed0b97b96d
    DOWNLOAD_ONLY YES)
add_library(stb INTERFACE)
add_library(stb::stb ALIAS stb)
target_include_directories(stb SYSTEM INTERFACE "${stb_SOURCE_DIR}")

# Header-only polygon triangulation for the ray tracing roofs (ISC license).
CPMAddPackage(
    NAME earcut
    GITHUB_REPOSITORY mapbox/earcut.hpp
    GIT_TAG v3.2.4
    DOWNLOAD_ONLY YES)
add_library(earcut INTERFACE)
add_library(earcut::earcut ALIAS earcut)
target_include_directories(earcut SYSTEM INTERFACE "${earcut_SOURCE_DIR}/include")

if(DOCKSIM_WITH_VULKAN_RT)
    # volk loads libvulkan.so.1 with dlopen at run time; nothing Vulkan is linked.
    CPMAddPackage(
        NAME VulkanHeaders
        SYSTEM YES
        GITHUB_REPOSITORY KhronosGroup/Vulkan-Headers
        GIT_TAG vulkan-sdk-1.4.350.0)
    CPMAddPackage(
        NAME volk
        SYSTEM YES
        GITHUB_REPOSITORY zeux/volk
        GIT_TAG 1.4.350
        OPTIONS "VOLK_PULL_IN_VULKAN ON" "VOLK_INSTALL OFF")
    CPMAddPackage(
        NAME VulkanMemoryAllocator
        SYSTEM YES
        GITHUB_REPOSITORY GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator
        GIT_TAG v3.4.0
        OPTIONS "VMA_ENABLE_INSTALL OFF")
    # The Slang compiler runs at build time only, to turn render.slang into SPIR-V.
    CPMAddPackage(
        NAME slang
        URL https://github.com/shader-slang/slang/releases/download/v2026.20/slang-2026.20-linux-x86_64-glibc-2.28.tar.gz
        URL_HASH SHA256=84ebd777968caceea26b21bfd014341078be6047be6c5a963a69f984f896cebd
        DOWNLOAD_ONLY YES)
    set(DOCKSIM_SLANGC "${slang_SOURCE_DIR}/bin/slangc" CACHE FILEPATH "Slang compiler" FORCE)
endif()

if(DOCKSIM_WITH_LIVE)
    include(ExternalFFmpeg)
    # WebRTC for WHIP (MPL-2.0; libjuice MPL-2.0, usrsctp and libsrtp BSD-3-Clause).
    # libsrtp includes its own "Warnings" module; hide ours while it configures.
    set(DOCKSIM_SAVED_MODULE_PATH "${CMAKE_MODULE_PATH}")
    list(REMOVE_ITEM CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
    CPMAddPackage(
        NAME libdatachannel
        SYSTEM YES
        GITHUB_REPOSITORY paullouisageneau/libdatachannel
        GIT_TAG v0.24.6
        GIT_SUBMODULES "deps/libjuice" "deps/usrsctp" "deps/libsrtp" "deps/plog"
        OPTIONS "NO_WEBSOCKET ON" "NO_EXAMPLES ON" "NO_TESTS ON" "NO_MEDIA OFF" "USE_GNUTLS OFF"
                "USE_MBEDTLS OFF" "USE_NICE OFF" "PREFER_SYSTEM_LIB OFF" "BUILD_SHARED_LIBS OFF")
    set(CMAKE_MODULE_PATH "${DOCKSIM_SAVED_MODULE_PATH}")
    docksim_build_after_openssl("${libdatachannel_SOURCE_DIR}")
    docksim_build_after_openssl("${libdatachannel_SOURCE_DIR}/deps/libjuice")
    docksim_build_after_openssl("${libdatachannel_SOURCE_DIR}/deps/libsrtp")
endif()

if(DOCKSIM_WITH_API)
    # Drogon (MIT) needs zlib, jsoncpp and libuuid. find_package() in Drogon must find
    # the static libraries below, so each one gets the cache variables of its Find module.
    # Imported wrapper targets keep the export set of Drogon valid.
    # zlib: the static zlib that OpenCV builds (target zlib). A second zlib would clash at link time.
    add_library(ZLIB::ZLIB INTERFACE IMPORTED GLOBAL)
    set_target_properties(ZLIB::ZLIB PROPERTIES
        INTERFACE_LINK_LIBRARIES zlib
        INTERFACE_INCLUDE_DIRECTORIES "${opencv_SOURCE_DIR}/3rdparty/zlib;${zlib_BINARY_DIR}")
    set(ZLIB_INCLUDE_DIR "${opencv_SOURCE_DIR}/3rdparty/zlib" CACHE INTERNAL "")
    set(ZLIB_LIBRARY zlib CACHE INTERNAL "")

    CPMAddPackage(
        NAME jsoncpp
        SYSTEM YES
        GITHUB_REPOSITORY open-source-parsers/jsoncpp
        GIT_TAG 1.9.8
        OPTIONS "JSONCPP_WITH_TESTS OFF" "JSONCPP_WITH_POST_BUILD_UNITTEST OFF"
                "JSONCPP_WITH_PKGCONFIG_SUPPORT OFF" "JSONCPP_WITH_CMAKE_PACKAGE OFF"
                "JSONCPP_WITH_EXAMPLE OFF" "BUILD_SHARED_LIBS OFF" "BUILD_STATIC_LIBS ON"
                "BUILD_OBJECT_LIBS OFF")
    # FetchContent redirects find_package(Jsoncpp) to an empty config, so FindJsoncpp of
    # Drogon never runs and never makes Jsoncpp_lib. The target is made here instead.
    add_library(Jsoncpp_lib INTERFACE IMPORTED GLOBAL)
    set_target_properties(Jsoncpp_lib PROPERTIES
        INTERFACE_LINK_LIBRARIES jsoncpp_static
        INTERFACE_INCLUDE_DIRECTORIES "${jsoncpp_SOURCE_DIR}/include")

    # libuuid of util-linux (BSD-3-Clause). Linux only, from the uuid-dev package, linked statically.
    find_path(DOCKSIM_UUID_INCLUDE_DIR uuid.h PATH_SUFFIXES uuid REQUIRED)
    find_library(DOCKSIM_UUID_LIBRARY NAMES libuuid.a REQUIRED)
    set(UUID_INCLUDE_DIRS "${DOCKSIM_UUID_INCLUDE_DIR}" CACHE PATH "libuuid headers" FORCE)
    set(UUID_LIBRARIES "${DOCKSIM_UUID_LIBRARY}" CACHE FILEPATH "libuuid" FORCE)

    # CPM makes find_package(jsoncpp CONFIG) succeed with the plain jsoncpp_static target,
    # which breaks the export set. Drogon then links Jsoncpp_lib.
    set(CMAKE_DISABLE_FIND_PACKAGE_jsoncpp TRUE)
    CPMAddPackage(
        NAME drogon
        SYSTEM YES
        GITHUB_REPOSITORY drogonframework/drogon
        GIT_TAG v1.9.14
        GIT_SUBMODULES "trantor"
        OPTIONS "BUILD_CTL OFF" "BUILD_EXAMPLES OFF" "BUILD_ORM OFF" "BUILD_BROTLI OFF"
                "BUILD_YAML_CONFIG OFF" "BUILD_SHARED_LIBS OFF" "BUILD_TESTING OFF" "BUILD_DOC OFF"
                "USE_SUBMODULE ON" "USE_SPDLOG OFF" "TRANTOR_USE_TLS none" "BUILD_C-ARES OFF"
                "USE_STATIC_LIBS_ONLY ON" "COZ_PROFILING OFF")
    unset(CMAKE_DISABLE_FIND_PACKAGE_jsoncpp)
endif()

CPMAddPackage(
    NAME CLI11
    SYSTEM YES
    GITHUB_REPOSITORY CLIUtils/CLI11
    VERSION 2.7.2
    OPTIONS "CLI11_PRECOMPILED OFF" "CLI11_BUILD_TESTS OFF" "CLI11_BUILD_EXAMPLES OFF"
            "CLI11_BUILD_DOCS OFF")

if(DOCKSIM_BUILD_TESTS)
    CPMAddPackage(
        NAME Catch2
        GITHUB_REPOSITORY catchorg/Catch2
        VERSION 3.16.1
        OPTIONS "CATCH_INSTALL_DOCS OFF" "CATCH_INSTALL_EXTRAS ON")
    list(APPEND CMAKE_MODULE_PATH "${Catch2_SOURCE_DIR}/extras")
    # HTTP and HTTPS test servers (MIT, header-only). Tests only; with our OpenSSL, no compression libraries.
    CPMAddPackage(
        NAME httplib
        SYSTEM YES
        GITHUB_REPOSITORY yhirose/cpp-httplib
        GIT_TAG v0.65.0
        OPTIONS "HTTPLIB_REQUIRE_OPENSSL ON" "HTTPLIB_USE_ZLIB_IF_AVAILABLE OFF" "HTTPLIB_USE_BROTLI_IF_AVAILABLE OFF"
                "HTTPLIB_USE_ZSTD_IF_AVAILABLE OFF" "HTTPLIB_INSTALL OFF" "HTTPLIB_COMPILE OFF")
endif()
