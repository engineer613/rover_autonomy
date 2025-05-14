cmake_minimum_required(VERSION 3.14)
project(um982_driver)

find_package(ament_cmake REQUIRED)
find_package(utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_interface REQUIRED)
find_package(unicore_msgs REQUIRED)


# Core driver library
add_library(um982_driver_core
        core/UnicoreDriver.cpp
        core/UnicoreParsers.cpp
)

target_include_directories(um982_driver_core PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(um982_driver_core
        rclcpp
        std_msgs
        unicore_msgs
        utils
        serial_interface
)

#rosidl_target_interfaces(unicore_msgs ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS
        um982_driver_core
        EXPORT export_um982_driver
)

install(DIRECTORY include/
        DESTINATION include
)

ament_export_targets(export_um982_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(utils serial_interface)
ament_package()
