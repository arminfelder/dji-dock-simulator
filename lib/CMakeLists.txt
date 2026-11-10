add_library(docksim STATIC)
add_library(docksim::docksim ALIAS docksim)

configure_file(src/build_info.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/generated/build_info.hpp" @ONLY)

target_sources(docksim
    PRIVATE
        src/version.cpp
        src/app/render_tools.cpp
        src/app/run.cpp
        src/cloud/login.cpp
        src/cloud/license.cpp
        src/cloud/redact.cpp
        src/control/sim_control.cpp
        src/control/wire_log.cpp
        src/config/config.cpp
        src/config/yaml.cpp
        src/device/catalog.cpp
        src/device/geo.cpp
        src/device/state.cpp
        src/device/weather.cpp
        src/image/image.cpp
        src/image/stb_impl.cpp
        src/log/log.cpp
        src/media/photo.cpp
        src/media/s3.cpp
        src/media/uploads.cpp
        src/net/http.cpp
        src/net/url.cpp
        src/protocol/envelope.cpp
        src/protocol/payloads.cpp
        src/protocol/routing.cpp
        src/raster/mercator.cpp
        src/services/areas.cpp
        src/services/camera.cpp
        src/services/hms.cpp
        src/services/logs.cpp
        src/services/maintenance.cpp
        src/services/peripherals.cpp
        src/services/registry.cpp
        src/services/services.cpp
        src/flight/manual.cpp
        src/sim/clock.cpp
        src/sim/context.cpp
        src/sim/dock_sim.cpp
        src/sim/views.cpp
        src/transport/mqtt.cpp
        src/transport/requester.cpp
        src/util/base64.cpp
        src/util/digest.cpp
        src/util/numeric.cpp
        src/wayline/engine.cpp
        src/wayline/wpml.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS include
        FILES
            include/docksim/app.hpp
            include/docksim/config.hpp
            include/docksim/render.hpp
            include/docksim/version.hpp)

include(src/geodata.cmake)
include(src/osm.cmake)
include(src/render.cmake)
include(src/rtmesh.cmake)

# Internal headers and third-party deps. The library and the white-box tests
# use them. Users of docksim::docksim see only include/docksim.
add_library(docksim_internal INTERFACE)
add_library(docksim::internal ALIAS docksim_internal)
target_include_directories(docksim_internal INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(docksim_internal INTERFACE
    asio::asio
    nlohmann_json::nlohmann_json
    spdlog::spdlog
    OpenSSL::Crypto
    ryml::ryml
    CURL::libcurl
    cpr::cpr
    aws-cpp-sdk-s3
    aws-cpp-sdk-core
    aws-crt-cpp
    eclipse-paho-mqtt-c::paho-mqtt3as-static
    OpenSSL::SSL
    pugixml::static
    Exiv2::exiv2lib
    opencv_imgproc
    opencv_core
    miniz
    mp-units::mp-units
    stb::stb
    earcut::earcut)

target_compile_features(docksim PUBLIC cxx_std_26)
target_link_libraries(docksim PRIVATE docksim::internal docksim::warnings docksim::optimize)

# stb is C-style code; keep the project warnings off its implementation file.
set_source_files_properties(src/image/stb_impl.cpp PROPERTIES COMPILE_OPTIONS "-w")

if(DOCKSIM_WITH_VULKAN_RT)
    include(Shaders)
    target_sources(docksim PRIVATE
        src/render/rt/shaders.cpp
        src/render/rt/frame.cpp
        src/render/rt/scene.cpp
        src/render/rt/vk.cpp)
    add_dependencies(docksim docksim_shaders)
    set_source_files_properties(src/render/rt/shaders.cpp PROPERTIES
        COMPILE_OPTIONS "--embed-dir=${DOCKSIM_SHADER_DIR}"
        OBJECT_DEPENDS "${DOCKSIM_SHADER_OUTPUTS}")
    target_link_libraries(docksim_internal INTERFACE Vulkan::Headers volk::volk GPUOpen::VulkanMemoryAllocator)
    target_compile_definitions(docksim_internal INTERFACE DOCKSIM_WITH_VULKAN_RT)
endif()

if(DOCKSIM_WITH_LIVE)
    include(src/live.cmake)
endif()

if(DOCKSIM_WITH_API)
    target_sources(docksim PRIVATE src/api/server.cpp)
    # Last fallback for the web UI: the npm output in the source tree, for runs from the build tree.
    set_source_files_properties(src/api/server.cpp PROPERTIES
        COMPILE_DEFINITIONS "DOCKSIM_WEB_DIST=\"${PROJECT_SOURCE_DIR}/web/dist/dock-sim/browser\"")
    target_link_libraries(docksim PRIVATE drogon)
    target_compile_definitions(docksim_internal INTERFACE DOCKSIM_WITH_API)
endif()
