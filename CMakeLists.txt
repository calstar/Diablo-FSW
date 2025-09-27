cmake_minimum_required(VERSION 3.20)
project(LiquidEngineFlightSoftware VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# Find optional packages
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(LIBCANARD QUIET libcanard)
endif()

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/FSW/include
    ${CMAKE_SOURCE_DIR}/comms/include
    ${CMAKE_SOURCE_DIR}/external/shared
    ${CMAKE_SOURCE_DIR}/utl
)

# External dependencies
include_directories(${EIGEN3_INCLUDE_DIRS})

# Source files for flight software
set(FSW_SOURCES
    FSW/src/main.cpp
    # Control system sources
    FSW/control/src/EngineControl.cpp
    FSW/control/src/ValveController.cpp
    FSW/control/src/GainScheduling.cpp
    FSW/control/src/OptimalController.cpp
    # Communication system sources
    FSW/comms/src/PacketProtocol.cpp
    FSW/comms/src/CommunicationProtocol.cpp
    # Calibration system sources
    FSW/calibration/src/SensorCalibration.cpp
    FSW/calibration/src/EncoderCalibration.cpp
    # Navigation system sources
    FSW/nav/src/SensorFusion.cpp
    FSW/nav/src/EKFNavigation.cpp
    # State management sources
    FSW/state/src/StateMachine.cpp
    # Legacy communication sources
    comms/src/MessageFactory.cpp
)

# Source files for utilities
set(UTL_SOURCES
    utl/db.cpp
    utl/dbConfig.cpp
    utl/Elodin.cpp
    utl/TCPSocket.cpp
)

# Create flight software executable
add_executable(engine_controller ${FSW_SOURCES} ${UTL_SOURCES})

# Link libraries
target_link_libraries(engine_controller
    Eigen3::Eigen
    Threads::Threads
    pthread
    rt
)

# Link optional libraries
if(LIBCANARD_FOUND)
    target_link_libraries(engine_controller ${LIBCANARD_LIBRARIES})
    target_compile_options(engine_controller PRIVATE ${LIBCANARD_CFLAGS_OTHER})
endif()

# Compiler-specific options
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(engine_controller PRIVATE -Wno-unused-parameter)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(engine_controller PRIVATE -Wno-unused-parameter)
endif()

# Install targets
install(TARGETS engine_controller
    RUNTIME DESTINATION bin
)

# Install configuration files
install(FILES config/config_engine.toml
    DESTINATION /etc/engine_controller
)

# Install systemd service file
install(FILES scripts/engine_controller.service
    DESTINATION /etc/systemd/system
)

# Create calibration directory
install(DIRECTORY DESTINATION /var/lib/engine_controller/calibrations)

# Create log directory
install(DIRECTORY DESTINATION /var/log/engine_controller)

# Testing
enable_testing()

# Add unit tests if they exist
if(EXISTS ${CMAKE_SOURCE_DIR}/tests)
    add_subdirectory(tests)
endif()

# Documentation
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_SOURCE_DIR}/docs/Doxyfile.in
                   ${CMAKE_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
endif()

# Packaging
set(CPACK_PACKAGE_NAME "liquid-engine-flight-software")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Liquid Engine Flight Software")
set(CPACK_PACKAGE_VENDOR "Your Organization")
set(CPACK_PACKAGE_CONTACT "your.email@organization.com")

include(CPack)