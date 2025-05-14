cmake_minimum_required(VERSION 3.14)
project(serial_interface)

find_package(ament_cmake REQUIRED)
find_package(utils REQUIRED)

# Build the core library
add_library(serial_interface_core
        core/SerialInterface.cpp
)

target_include_directories(serial_interface_core PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(serial_interface_core
        utils
)

# Install headers and library
install(
        TARGETS serial_interface_core
        EXPORT export_serial_interface
)

install(
        DIRECTORY include/
        DESTINATION include
)

ament_export_targets(export_serial_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(utils)

ament_package()