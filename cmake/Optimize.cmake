add_library(docksim_optimize INTERFACE)
add_library(docksim::optimize ALIAS docksim_optimize)

option(DOCKSIM_FAST_MATH "Fast floating-point math in Release builds of the project code" OFF)
option(DOCKSIM_NATIVE_ARCH "Compile the project code for the CPU of the build machine" OFF)

# -ffinite-math-only stays off: the code uses NaN for missing data and infinity
# as a no-hit distance, and that flag lets the compiler remove those checks.
target_compile_options(docksim_optimize INTERFACE
    $<$<AND:$<BOOL:${DOCKSIM_FAST_MATH}>,$<CONFIG:Release>,$<CXX_COMPILER_ID:GNU,Clang>>:
        -ffast-math -fno-finite-math-only>
    # The binary then runs only on CPUs with the instruction set of the build machine.
    $<$<AND:$<BOOL:${DOCKSIM_NATIVE_ARCH}>,$<CXX_COMPILER_ID:GNU,Clang>>:-march=native>)
