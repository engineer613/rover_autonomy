from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='um982_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='serial_interface',
                    plugin='serial_interface::SerialInterface',
                    name='um982_serial',
                    parameters=[{
                        'port': '/dev/gps_data'
                    }]
                ),
                ComposableNode(
                    package='um982_ros_driver',
                    plugin='um982_ros_driver::UM982ROSDriver',
                    name='um982_driver'
                    # You can add parameters here if needed
                ),
            ],
            output='screen',
        )
    ])
