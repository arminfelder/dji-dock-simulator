cmake_minimum_required(VERSION 3.30)

project(docksim
    VERSION 0.1.0
    DESCRIPTION "DJI Dock 2 emulator that speaks the DJI Cloud API to a cloud backend"
    LANGUAGES C CXX)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
    message(FATAL_ERROR "In-source builds are not supported. Use a preset: cmake --preset gcc-debug")
endif()

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "docksim links all third-party libraries statically" FORCE)

option(DOCKSIM_STATIC_RUNTIME "Link libstdc++ and libgcc statically into executables" OFF)
option(DOCKSIM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(DOCKSIM_BUILD_TESTS "Build the Catch2 test suite" ${PROJECT_IS_TOP_LEVEL})
option(DOCKSIM_WITH_OPENCL "OpenCL heightfield renderer" ON)
option(DOCKSIM_WITH_VULKAN_RT "Vulkan hardware ray tracing renderer" ON)
option(DOCKSIM_WITH_LIVE "Live video push (RTMP, RTSP, WHIP)" ON)
option(DOCKSIM_WITH_API "Instructor REST API and web UI (Drogon)" ON)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(Warnings)
include(Optimize)
include(Dependencies)

add_subdirectory(lib)
add_subdirectory(apps/dock-sim)

if(DOCKSIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
