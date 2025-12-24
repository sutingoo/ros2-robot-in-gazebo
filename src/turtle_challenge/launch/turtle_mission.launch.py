import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='simulador_tortuga'
        ),
        Node(
            package='turtle_challenge',
            executable='turtle_bot',
            name='comandante_supremo',
            output='screen'
        )
    ])
