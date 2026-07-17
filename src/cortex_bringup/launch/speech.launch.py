"""Speech I/O only: STT (perception) + TTS (action).

A subset of cortex.launch.py for exercising the audio path without the cognition
layer. The two nodes live in different packages now — stt_node is perception
(audio -> symbols), tts_node is action (the robot speaking).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = os.path.join(
        get_package_share_directory('cortex_bringup'), 'config', 'cortex_params.yaml')

    def node(pkg, exe):
        return Node(package=pkg, executable=exe, name=exe, output='screen',
                    parameters=[params])

    return LaunchDescription([
        node('cortex_perception', 'stt_node'),
        node('cortex_action', 'tts_node'),
    ])
