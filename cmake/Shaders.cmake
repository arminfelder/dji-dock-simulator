# render.slang to SPIR-V, one module per entry point, plus Slang's reflection
# JSON. The library embeds both; the Vulkan code builds its descriptor and
# push-constant layouts from the reflection, so the shader can change freely.

set(DOCKSIM_SHADER_DIR "${CMAKE_BINARY_DIR}/generated/shaders")
set(DOCKSIM_SHADER_SOURCE "${PROJECT_SOURCE_DIR}/lib/resources/shaders/render.slang")
file(MAKE_DIRECTORY "${DOCKSIM_SHADER_DIR}")

# A host tool that writes the constants module from the C++ headers.
add_executable(docksim_rtconst_gen "${PROJECT_SOURCE_DIR}/lib/tools/rtconst_gen.cpp")
target_link_libraries(docksim_rtconst_gen PRIVATE docksim::internal)
target_compile_features(docksim_rtconst_gen PRIVATE cxx_std_26)

add_custom_command(
    OUTPUT "${DOCKSIM_SHADER_DIR}/rtconst.slang"
    COMMAND docksim_rtconst_gen "${DOCKSIM_SHADER_DIR}/rtconst.slang"
    DEPENDS docksim_rtconst_gen
    COMMENT "Writing rtconst.slang")

set(DOCKSIM_SHADER_OUTPUTS "")
foreach(entry IN ITEMS trace shade subsample downsample)
    set(spv "${DOCKSIM_SHADER_DIR}/${entry}.spv")
    set(json "${DOCKSIM_SHADER_DIR}/${entry}.json")
    add_custom_command(
        OUTPUT "${spv}" "${json}"
        COMMAND "${DOCKSIM_SLANGC}" "${DOCKSIM_SHADER_SOURCE}" -I "${DOCKSIM_SHADER_DIR}"
                -target spirv -profile spirv_1_5+spvRayQueryKHR -entry ${entry} -stage compute
                -o "${spv}" -reflection-json "${json}"
        DEPENDS "${DOCKSIM_SHADER_SOURCE}" "${DOCKSIM_SHADER_DIR}/rtconst.slang"
        COMMENT "Compiling render.slang:${entry} to SPIR-V")
    list(APPEND DOCKSIM_SHADER_OUTPUTS "${spv}" "${json}")
endforeach()
add_custom_target(docksim_shaders DEPENDS ${DOCKSIM_SHADER_OUTPUTS})
