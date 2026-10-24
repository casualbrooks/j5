#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_voice = LaunchConfiguration("start_voice")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_voice",
                default_value="false",
                description="Start placeholder j5_voice hello node",
            ),
            # Placeholder nodes
            # Node(package='j5_perception', executable='vision_node', name='vision'),
            Node(
                package="j5_voice",
                executable="hello",
                name="voice_hello",
                condition=IfCondition(start_voice),
            ),
        ]
    )
