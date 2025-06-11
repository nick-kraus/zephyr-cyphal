cmake_minimum_required(VERSION 3.20.0)

# Always generate compilation database.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

# Set project name, used for firmware output filename.
set(PROJECT_NAME test_zyphal)

find_package(Zephyr REQUIRED HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../zephyr")
project(app LANGUAGES C)

target_sources(app PRIVATE
    "src/can_fff.c"
    "src/test_transmit.c"
)

target_include_directories(app PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/../src"
)
