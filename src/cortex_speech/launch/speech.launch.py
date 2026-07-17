"""Launch the speech I/O nodes (STT + TTS)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='cortex_speech',
            executable='stt_node',
            name='stt_node',
            output='screen',
        ),
        Node(
            package='cortex_speech',
            executable='tts_node',
            name='tts_node',
            output='screen',
        ),
    ])
