from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1. Launch the driver + serial interface as a container
        ComposableNodeContainer(
            name='rover_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='serial_interface',
                    plugin='serial_interface::SerialInterfaceNode',
                    name='um982_serial_interface',
                ),
                ComposableNode(
                    package='um982_ros_driver',
                    plugin='um982_ros_driver::UM982ROSDriver',
                    name='um982_ros_driver',
                ),
            ],
            output='screen'
        ),
    ])