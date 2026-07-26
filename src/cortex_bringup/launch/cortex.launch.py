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
        # --- perception ---
        node('cortex_perception', 'stt_node'),
        node('cortex_perception', 'vlm_node'),
        # --- cognition ---
        node('cortex_cognition', 'orchestrator_node'),
        # --- action ---
        node('cortex_action', 'tts_node'),
        # --- gui egress ---
        node('cortex_gui', 'gui_bridge_node'),
    ])
