add_executable(dock-sim main.cpp)
target_link_libraries(dock-sim PRIVATE docksim::docksim docksim::warnings docksim::optimize CLI11::CLI11)

if(DOCKSIM_STATIC_RUNTIME)
    target_link_options(dock-sim PRIVATE -static-libstdc++ -static-libgcc)
endif()

include(GNUInstallDirs)
install(TARGETS dock-sim RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(DOCKSIM_WITH_API)
    # The web UI is a separate npm build. ON runs it as part of the CMake build.
    option(DOCKSIM_BUILD_WEB "Build the Angular web UI with npm" OFF)
    set(DOCKSIM_WEB_DIST "${PROJECT_SOURCE_DIR}/web/dist/dock-sim/browser")
    if(DOCKSIM_BUILD_WEB)
        find_program(DOCKSIM_NPM npm REQUIRED)
        add_custom_target(web ALL
            COMMAND "${DOCKSIM_NPM}" ci --no-audit --no-fund
            COMMAND "${DOCKSIM_NPM}" run build
            WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}/web"
            USES_TERMINAL)
    endif()
    install(DIRECTORY "${DOCKSIM_WEB_DIST}/" DESTINATION ${CMAKE_INSTALL_DATADIR}/dock-sim/web OPTIONAL)
endif()
