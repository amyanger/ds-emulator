cmake_minimum_required(VERSION 3.20)
project(ds_emulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile_commands.json for clangd / other tooling.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(DS_TARGET_APPLE_SILICON "Enable Apple Silicon (arm64 + NEON) optimizations" OFF)
option(DS_ENABLE_SANITIZERS    "Enable ASan + UBSan on debug builds"                OFF)
option(DS_BUILD_TESTS          "Build unit tests"                                    ON)
option(ENABLE_XRAY             "Enable in-emulator debug overlay"                    ON)

include(cmake/CompilerWarnings.cmake)
include(cmake/AppleSilicon.cmake)
include(cmake/Sanitizers.cmake)

# SDL2
find_package(SDL2 REQUIRED)

# Subdirectories
add_subdirectory(src)

if(DS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Top-level binary target wires everything together.
# TODO P0.11: Re-enable once src/main.cpp exists. Commented out in the
# Tasks 1-9 batch so CMake can configure without src/main.cpp present.
# add_executable(ds_emulator src/main.cpp)
# target_link_libraries(ds_emulator PRIVATE ds_frontend ds_core)
# target_include_directories(ds_emulator PRIVATE include src)
# target_enable_warnings(ds_emulator)
# target_apple_silicon(ds_emulator)
# target_enable_sanitizers(ds_emulator)
