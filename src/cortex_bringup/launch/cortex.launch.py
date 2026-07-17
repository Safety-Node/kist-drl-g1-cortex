"""Top-level bringup: speech + reasoning nodes with a shared params file."""

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
        # --- speech ---
        node('cortex_speech', 'stt_node'),
        node('cortex_speech', 'tts_node'),
        # --- reasoning ---
        node('cortex_reasoning', 'orchestrator_node'),
        node('cortex_reasoning', 'vlm_node'),
    ])
