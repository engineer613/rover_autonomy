cmake_minimum_required(VERSION 3.14)
project(serial_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# List out msg files
set(msg_files
        "msg/SerialStamped.msg"
)

# Generate Messages
rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES builtin_interfaces
)

# Build the library
add_library(serial_interface_component SHARED
        src/serial_interface.cpp
)

target_include_directories(serial_interface_component PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(serial_interface_component
        rclcpp
        rclcpp_components
        std_msgs
        builtin_interfaces
)

# This is so that serial_interface can use the msgs/interfaces defined in the serial_interface pkg
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(serial_interface_component "${cpp_typesupport_target}")
#rosidl_target_interfaces(serial_interface_component ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Make it a Composable Node
rclcpp_components_register_nodes(serial_interface_component "serial_interface::SerialInterface")

ament_export_dependencies(rosidl_default_runtime)

# ======== Test Node ========
set(TEST_NODE "test_node")
add_executable(${TEST_NODE} src/test_node.cpp)
target_link_libraries(${TEST_NODE}
        serial_interface_component
)
ament_target_dependencies(${TEST_NODE}
        rclcpp
        std_msgs
        builtin_interfaces
)
# ===========================

# Install
# Library needs to be in lib/
install(TARGETS
        serial_interface_component
        EXPORT export_serial_interface
        DESTINATION lib/
)

# Nodes need to be in lib/<pkg_name>
install(TARGETS
        ${TEST_NODE}
        DESTINATION lib/
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

# Export Library Interface For Downstream Users
ament_export_targets(export_serial_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(utils)

ament_package()