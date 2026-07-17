"""Streaming STT node.

Subscribes to mic PCM on the shared bridge domain and emits transcripts as soon
as an utterance is endpointed by VAD, so downstream reasoning can react without
waiting for a fixed tick.

Scaffold only — the actual STT backend (e.g. faster-whisper) is not wired yet.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# from g1_onboard_msgs.msg import AudioPCM   # mic PCM (shared interface)


class SttNode(Node):
    def __init__(self) -> None:
        super().__init__('stt_node')

        # --- parameters -----------------------------------------------------
        self.declare_parameter('audio_topic', '/bridge/sensors/audio/pcm')
        self.declare_parameter('transcript_topic', '/cortex/stt/transcript')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('vad_silence_ms', 500)   # endpoint after N ms silence

        audio_topic = self.get_parameter('audio_topic').value
        transcript_topic = self.get_parameter('transcript_topic').value

        # --- io -------------------------------------------------------------
        self.pub = self.create_publisher(String, transcript_topic, 10)

        # TODO(REQ-XX) [TASK-XX]: subscribe to AudioPCM instead of the placeholder.
        # self.sub = self.create_subscription(AudioPCM, audio_topic, self._on_pcm, 10)

        # TODO(REQ-XX) [TASK-XX]: load streaming STT backend + webrtcvad here.
        #   Run inference off the spin thread (worker), like router's LLM path.
        # TODO(REQ-XX): denoise/highpass front-end for the ~30% WER problem —
        #   decide location: likely the onboard C++ mic_node (before the LAN hop),
        #   not here. Move this note there if so.

        self.get_logger().info(f'stt_node up (audio={audio_topic} -> {transcript_topic})')

    def _on_pcm(self, msg) -> None:  # noqa: ANN001 - scaffold
        # TODO(REQ-XX) [TASK-XX]: feed PCM to VAD; on endpoint, run STT and
        # publish a String transcript. Emit partial hypotheses for low latency.
        raise NotImplementedError


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SttNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
