include(Catch)

add_executable(docksim_tests
    golden/golden_test.cpp
    golden/harness.cpp
    unit/async_test.cpp
    unit/camera_test.cpp
    unit/clock_test.cpp
    unit/config_test.cpp
    unit/control_test.cpp
    unit/rain_test.cpp
    unit/foundation_test.cpp
    unit/maintenance_test.cpp
    unit/manual_flight_test.cpp
    unit/media_test.cpp
    unit/net_http_test.cpp
    unit/net_url_test.cpp
    unit/poi_wayline_test.cpp
    unit/version_test.cpp)
target_link_libraries(docksim_tests PRIVATE
    docksim::docksim docksim::internal docksim::warnings docksim::optimize Catch2::Catch2WithMain httplib::httplib)
target_compile_definitions(docksim_tests PRIVATE DOCKSIM_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
target_include_directories(docksim_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

include(geodata.cmake)
include(osm.cmake)
include(render.cmake)
include(rtmesh.cmake)

catch_discover_tests(docksim_tests)

# Needs a broker: docker with emqx/emqx:5.8.8, or DOCKSIM_TEST_BROKER=host:port.
add_executable(docksim_integration_tests
    integration/broker_test.cpp
    golden/harness.cpp)
target_link_libraries(docksim_integration_tests PRIVATE
    docksim::docksim docksim::internal docksim::warnings docksim::optimize Catch2::Catch2WithMain)
target_compile_definitions(docksim_integration_tests PRIVATE DOCKSIM_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
catch_discover_tests(docksim_integration_tests PROPERTIES LABELS integration SKIP_RETURN_CODE 4)

if(DOCKSIM_WITH_VULKAN_RT)
    target_sources(docksim_tests PRIVATE unit/rt_vk_test.cpp)
endif()

if(DOCKSIM_WITH_LIVE)
    include(live.cmake)
endif()

# Drogon runs once per process, so the API test has its own executable.
if(DOCKSIM_WITH_API)
    add_executable(docksim_api_tests api/api_test.cpp golden/harness.cpp)
    target_link_libraries(docksim_api_tests PRIVATE
        docksim::docksim docksim::internal docksim::warnings docksim::optimize Catch2::Catch2WithMain)
    target_compile_definitions(docksim_api_tests PRIVATE DOCKSIM_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
    target_include_directories(docksim_api_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    catch_discover_tests(docksim_api_tests)
endif()
