#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Placeholder nodes
        # Node(package='j5_perception', executable='vision_node', name='vision'),
        Node(package='j5_voice', executable='voice_hello', name='voice_hello'),
    ])
