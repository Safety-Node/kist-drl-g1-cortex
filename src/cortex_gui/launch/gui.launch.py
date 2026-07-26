"""GUI egress only: gui_bridge_node.

Standalone bring-up for the display bridge — useful for testing the renderer
against live TaskStatus without launching the whole cortex graph. Uses the same
shared params file as cortex.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = os.path.join(
        get_package_share_directory('cortex_bringup'), 'config', 'cortex_params.yaml')

    return LaunchDescription([
        Node(package='cortex_gui', executable='gui_bridge_node',
             name='gui_bridge_node', output='screen', parameters=[params]),
    ])
