# ds_frontend — SDL2-aware static library. Platform abstractions live here.
# This is the ONLY target allowed to include SDL headers.

find_package(SDL2 REQUIRED)

add_library(ds_frontend STATIC
    frontend.cpp
)
target_include_directories(ds_frontend
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(ds_frontend
    PUBLIC  ds_core
    PRIVATE SDL2::SDL2
)
target_enable_warnings(ds_frontend)
target_apple_silicon(ds_frontend)
target_enable_sanitizers(ds_frontend)
