import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    move_node = Node(
        package="xarm_move",
        executable="xarm_move_client",
        #namespace="xarm",
        output="screen",
    )
    ld.add_action(move_node)

    return ld
