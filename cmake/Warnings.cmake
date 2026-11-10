add_library(docksim_warnings INTERFACE)
add_library(docksim::warnings ALIAS docksim_warnings)

target_compile_options(docksim_warnings INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang>:
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
        -Wnon-virtual-dtor -Wold-style-cast -Woverloaded-virtual -Wnull-dereference
        -Wimplicit-fallthrough -Wformat=2
        -Wno-missing-field-initializers>
    $<$<CXX_COMPILER_ID:GNU>:-Wduplicated-cond -Wlogical-op -Wuseless-cast>
    # #embed is standard C++26; Clang 21 still reports it under -Wpedantic.
    $<$<CXX_COMPILER_ID:Clang>:-Wno-c23-extensions>
    $<$<BOOL:${DOCKSIM_WARNINGS_AS_ERRORS}>:-Werror>)
