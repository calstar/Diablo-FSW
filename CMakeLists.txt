cmake_minimum_required(VERSION 3.16)
project(SensorSystem)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/comms/include
    ${CMAKE_SOURCE_DIR}/utl
    ${CMAKE_SOURCE_DIR}/external/shared/message_factory
)

# Find required packages
find_package(Threads REQUIRED)

# Add executable for local fake sensor generator
add_executable(fake_sensor_generator
    scripts/fake_sensor_generator.cpp
)

# Add executable for remote fake sensor generator
add_executable(fake_sensor_generator_remote
    scripts/fake_sensor_generator_remote.cpp
)

# Link libraries
target_link_libraries(fake_sensor_generator
    Threads::Threads
)

target_link_libraries(fake_sensor_generator_remote
    Threads::Threads
)

# Compiler flags
target_compile_options(fake_sensor_generator PRIVATE
    -Wall
    -Wextra
    -O2
)

target_compile_options(fake_sensor_generator_remote PRIVATE
    -Wall
    -Wextra
    -O2
)

# Set output directory
set_target_properties(fake_sensor_generator PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/scripts
)

set_target_properties(fake_sensor_generator_remote PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/scripts
)
