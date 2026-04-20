cmake_minimum_required(VERSION 3.18)
project(VehTelemetry VERSION 1.0.0 LANGUAGES CXX)

# ── Standard ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Compiler flags ────────────────────────────────────────────────────────
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|armv8)")
    message(STATUS "Enabling ARM NEON SIMD")
    add_compile_options(-march=armv8-a+simd -mtune=cortex-a72)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
    message(STATUS "Enabling ARMv7 NEON")
    add_compile_options(-mfpu=neon -mfloat-abi=hard)
endif()

add_compile_options(
    -Wall -Wextra -Wpedantic
    -O2
    -fno-omit-frame-pointer      # Keep frame pointers for profiling
)

# ── Dependencies ──────────────────────────────────────────────────────────

# Apache Arrow + Parquet
find_package(Arrow REQUIRED)
find_package(Parquet REQUIRED)

# Threads (pthreads)
find_package(Threads REQUIRED)

# yaml-cpp for config (optional — soft dependency)
find_package(yaml-cpp QUIET)

# ── Include directories ───────────────────────────────────────────────────
include_directories(${CMAKE_SOURCE_DIR}/include)

# ── Sources ───────────────────────────────────────────────────────────────
set(SOURCES
    src/lidar_driver.cpp
    src/imu_driver.cpp
    src/can_driver.cpp
    src/fusion_engine.cpp
    src/parquet_exporter.cpp
    src/metrics.cpp
    src/main.cpp
)

# ── Executable ────────────────────────────────────────────────────────────
add_executable(veh_telemetry ${SOURCES})

target_include_directories(veh_telemetry PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${Arrow_INCLUDE_DIRS}
    ${Parquet_INCLUDE_DIRS}
)

target_link_libraries(veh_telemetry PRIVATE
    Arrow::arrow_shared
    Parquet::parquet_shared
    Threads::Threads
)

if(yaml-cpp_FOUND)
    target_link_libraries(veh_telemetry PRIVATE yaml-cpp)
    target_compile_definitions(veh_telemetry PRIVATE HAVE_YAML_CPP)
    message(STATUS "yaml-cpp found — config file parsing enabled")
else()
    message(STATUS "yaml-cpp not found — config file parsing disabled")
endif()

# ── Install ───────────────────────────────────────────────────────────────
install(TARGETS veh_telemetry RUNTIME DESTINATION bin)

# ── Summary ───────────────────────────────────────────────────────────────
message(STATUS "VehTelemetry ${PROJECT_VERSION}")
message(STATUS "  Processor  : ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "  Build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Arrow ver  : ${Arrow_VERSION}")
message(STATUS "  Parquet ver: ${Parquet_VERSION}")
