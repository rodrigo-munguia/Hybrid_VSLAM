from launch import LaunchDescription
from launch_ros.actions import Node
import sys

def generate_launch_description():
    return LaunchDescription([
        #Node(
        #    package='bebop_driver',
        #    node_namespace='bebop_driver1',
        #    node_executable='bebop_driver_node',
        #    node_name='bebop_driver_node',
        #    output="screen",
        #),
        Node(
            package='keyboard',
            namespace='',
            executable='keyboard_node',
            name='keyboard_node',
            output="screen",             
        )
    ])