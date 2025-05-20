from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='serial_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='serial_interface',
                    plugin='serial_interface::SerialInterface',
                    name='serial_composable_test',
                    parameters=[{'port': '/dev/ttyUSB0'}]
                ),
            ],
            output='screen',
        )
    ])
