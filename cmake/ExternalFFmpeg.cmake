# FFmpeg (LGPL-2.1-or-later) with the openh264 encoder (BSD-2-Clause) and the
# NVENC headers, built from source as static libraries. Neither builds with
# CMake, so they are ExternalProjects; the result is the imported target
# docksim::av. Configure options keep FFmpeg LGPL: no --enable-gpl or nonfree.
# openh264 built from source carries no Cisco patent license: internal use only.

include(ExternalProject)

set(DOCKSIM_AV_PREFIX "${CMAKE_BINARY_DIR}/_deps/av")
set(DOCKSIM_AV_DOWNLOADS "${CPM_SOURCE_CACHE}/av-downloads")
file(MAKE_DIRECTORY "${DOCKSIM_AV_PREFIX}/include" "${DOCKSIM_AV_PREFIX}/lib")
cmake_host_system_information(RESULT DOCKSIM_AV_JOBS QUERY NUMBER_OF_LOGICAL_CORES)
find_program(DOCKSIM_MAKE NAMES gmake make REQUIRED)
find_program(DOCKSIM_PKG_CONFIG NAMES pkg-config pkgconf REQUIRED)

ExternalProject_Add(docksim_openh264
    URL https://github.com/cisco/openh264/archive/refs/tags/v2.6.0.tar.gz
    URL_HASH SHA256=558544ad358283a7ab2930d69a9ceddf913f4a51ee9bf1bfb9e377322af81a69
    DOWNLOAD_DIR "${DOCKSIM_AV_DOWNLOADS}"
    DOWNLOAD_EXTRACT_TIMESTAMP OFF
    BUILD_IN_SOURCE ON
    CONFIGURE_COMMAND ""
    BUILD_COMMAND "${DOCKSIM_MAKE}" -j${DOCKSIM_AV_JOBS} OS=linux ARCH=x86_64 USE_ASM=Yes BUILDTYPE=Release
                  "CC=${CMAKE_C_COMPILER}" "CXX=${CMAKE_CXX_COMPILER}" libopenh264.a
    INSTALL_COMMAND "${DOCKSIM_MAKE}" OS=linux ARCH=x86_64 "PREFIX=${DOCKSIM_AV_PREFIX}" install-static
    BUILD_BYPRODUCTS "${DOCKSIM_AV_PREFIX}/lib/libopenh264.a")

ExternalProject_Add(docksim_nvcodec_headers
    URL https://github.com/FFmpeg/nv-codec-headers/archive/refs/tags/n12.2.72.0.tar.gz
    URL_HASH SHA256=dbeaec433d93b850714760282f1d0992b1254fc3b5a6cb7d76fc1340a1e47563
    DOWNLOAD_DIR "${DOCKSIM_AV_DOWNLOADS}"
    DOWNLOAD_EXTRACT_TIMESTAMP OFF
    BUILD_IN_SOURCE ON
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND "${DOCKSIM_MAKE}" "PREFIX=${DOCKSIM_AV_PREFIX}" install)

set(DOCKSIM_AV_LIBS avformat avcodec swscale avutil)
set(DOCKSIM_AV_BYPRODUCTS "")
foreach(lib IN LISTS DOCKSIM_AV_LIBS)
    list(APPEND DOCKSIM_AV_BYPRODUCTS "${DOCKSIM_AV_PREFIX}/lib/lib${lib}.a")
endforeach()

# Only what dock-sim uses: H.264 (openh264 or NVENC) and AAC into FLV/RTMP,
# RTSP/RTP; MP4 and the demuxers serve the tests. NVENC loads the NVIDIA
# driver with dlopen at run time.
ExternalProject_Add(docksim_ffmpeg
    URL https://github.com/FFmpeg/FFmpeg/archive/refs/tags/n8.1.3.tar.gz
    URL_HASH SHA256=09f990289327d3ebfedf4fc15cae3f884a65dab7a5aac7bbb5b01ff83900efd9
    DOWNLOAD_DIR "${DOCKSIM_AV_DOWNLOADS}"
    DOWNLOAD_EXTRACT_TIMESTAMP OFF
    DEPENDS docksim_openh264 docksim_nvcodec_headers
    CONFIGURE_COMMAND ${CMAKE_COMMAND} -E env "PKG_CONFIG_PATH=${DOCKSIM_AV_PREFIX}/lib/pkgconfig"
        <SOURCE_DIR>/configure
        "--prefix=${DOCKSIM_AV_PREFIX}" "--cc=${CMAKE_C_COMPILER}" "--cxx=${CMAKE_CXX_COMPILER}"
        "--pkg-config=${DOCKSIM_PKG_CONFIG}" --pkg-config-flags=--static
        --enable-static --disable-shared --enable-pic --disable-programs --disable-doc --disable-debug
        --disable-autodetect --disable-everything --disable-avdevice --disable-avfilter --disable-swresample
        --enable-avformat --enable-avcodec --enable-swscale --enable-network
        --enable-libopenh264 --enable-ffnvcodec --enable-nvenc
        --enable-encoder=libopenh264,aac,h264_nvenc
        --enable-muxer=flv,rtsp,rtp,mp4
        --enable-demuxer=flv,mov
        --enable-parser=h264,aac
        --enable-protocol=file,tcp,udp,rtmp,rtp
        --enable-bsf=h264_mp4toannexb,extract_extradata
    BUILD_COMMAND "${DOCKSIM_MAKE}" -j${DOCKSIM_AV_JOBS}
    INSTALL_COMMAND "${DOCKSIM_MAKE}" install
    BUILD_BYPRODUCTS ${DOCKSIM_AV_BYPRODUCTS})

# libatomic by path: -latomic would add libatomic.so.1 to the binary's NEEDED list.
execute_process(COMMAND "${CMAKE_C_COMPILER}" -print-file-name=libatomic.a
                OUTPUT_VARIABLE DOCKSIM_LIBATOMIC OUTPUT_STRIP_TRAILING_WHITESPACE)
find_package(Threads REQUIRED)

add_library(docksim_av INTERFACE)
add_library(docksim::av ALIAS docksim_av)
add_dependencies(docksim_av docksim_ffmpeg)
target_include_directories(docksim_av SYSTEM INTERFACE "${DOCKSIM_AV_PREFIX}/include")
foreach(lib IN LISTS DOCKSIM_AV_LIBS)
    add_library(docksim_av_${lib} STATIC IMPORTED GLOBAL)
    set_target_properties(docksim_av_${lib} PROPERTIES IMPORTED_LOCATION "${DOCKSIM_AV_PREFIX}/lib/lib${lib}.a")
    add_dependencies(docksim_av_${lib} docksim_ffmpeg)
    target_link_libraries(docksim_av INTERFACE docksim_av_${lib})
endforeach()
add_library(docksim_av_openh264 STATIC IMPORTED GLOBAL)
set_target_properties(docksim_av_openh264 PROPERTIES IMPORTED_LOCATION "${DOCKSIM_AV_PREFIX}/lib/libopenh264.a")
add_dependencies(docksim_av_openh264 docksim_openh264)
target_link_libraries(docksim_av INTERFACE docksim_av_openh264 Threads::Threads m "${DOCKSIM_LIBATOMIC}" ${CMAKE_DL_LIBS})
