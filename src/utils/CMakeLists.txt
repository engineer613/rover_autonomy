cmake_minimum_required(VERSION 3.14)
project(utils)

find_package(ament_cmake REQUIRED)

# Header-only library
add_library(utils INTERFACE)
target_include_directories(utils INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS utils
        EXPORT export_utils
)

ament_export_targets(export_utils HAS_LIBRARY_TARGET)
ament_package()
