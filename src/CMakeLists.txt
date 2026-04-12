# ds_core — platform-free static library. Zero SDL2 dependency.
# Anything under src/frontend/ is NOT in this target; see the subdir below.
add_library(ds_core STATIC
    nds.cpp
    scheduler/scheduler.cpp
    cpu/arm9/arm9.cpp
    cpu/arm7/arm7.cpp
)
target_include_directories(ds_core
    PUBLIC  ${CMAKE_SOURCE_DIR}/include
    PRIVATE ${CMAKE_SOURCE_DIR}/src
)
target_enable_warnings(ds_core)
target_apple_silicon(ds_core)
target_enable_sanitizers(ds_core)

# ds_frontend — SDL2-owning static library. This is the ONLY target that may
# include SDL headers.
# TODO P0.10: Re-enable once src/frontend/CMakeLists.txt exists. Commented out
# in the Tasks 1-9 batch so `ds_core` can build standalone for verification.
# add_subdirectory(frontend)
