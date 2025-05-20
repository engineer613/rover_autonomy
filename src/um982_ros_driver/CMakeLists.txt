cmake_minimum_required(VERSION 3.14)
project(um982_ros_driver)

find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(serial_interface REQUIRED)
find_package(unicore_msgs REQUIRED)


# Core driver library
add_library(um982_ros_driver SHARED
        src/um982_ros_driver.cpp
        src/unicore_parsers.cpp
)


rclcpp_components_register_nodes(um982_ros_driver
        "um982_ros_driver::UM982ROSDriver"
)

target_include_directories(um982_ros_driver PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(um982_ros_driver
        rclcpp
        rclcpp_lifecycle
        rclcpp_components
        std_msgs
        unicore_msgs
        serial_interface
)


# Install targets
install(TARGETS um982_ros_driver
        EXPORT export_um982_ros_driver
        DESTINATION lib
)

install(DIRECTORY include/
        DESTINATION include
)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_um982_ros_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(serial_interface)
ament_package()
