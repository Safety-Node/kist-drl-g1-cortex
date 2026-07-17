"""Streaming TTS node with barge-in.

Subscribes to text-to-speak requests and streams synthesized audio to the
onboard speaker. Playback is cancelable: an explicit stop on the barge-in topic
interrupts speech immediately — the topic-based analogue of the orchestrator's
connector cancel.

NOTE: nothing publishes the barge-in topic yet. To cut an announcement when the
user interrupts, the orchestrator's SpeakConnector.cancel would publish here.

Scaffold only — no TTS backend wired yet.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# from g1_onboard_msgs.msg import AudioPCM, SpeakerState


class TtsNode(Node):
    def __init__(self) -> None:
        super().__init__('tts_node')

        self.declare_parameter('say_topic', '/cortex/tts/say')
        self.declare_parameter('barge_in_topic', '/cortex/tts/stop')
        self.declare_parameter('speaker_topic', '/bridge/actuators/speaker/pcm')

        say_topic = self.get_parameter('say_topic').value
        barge_in_topic = self.get_parameter('barge_in_topic').value

        self._speaking = False

        self.create_subscription(String, say_topic, self._on_say, 10)
        self.create_subscription(Bool, barge_in_topic, self._on_barge_in, 10)

        # TODO(REQ-XX) [TASK-XX]: publisher of AudioPCM to the speaker topic.
        # TODO(REQ-XX) [TASK-XX]: load streaming TTS backend.

        self.get_logger().info(f'tts_node up (say={say_topic}, barge_in={barge_in_topic})')

    def _on_say(self, msg: String) -> None:
        # TODO(REQ-XX) [TASK-XX]: stream synthesis chunk-by-chunk to the speaker,
        # checking self._speaking between chunks so barge-in can cut in.
        self._speaking = True
        self.get_logger().info(f'(stub) say: {msg.data!r}')

    def _on_barge_in(self, msg: Bool) -> None:
        if msg.data and self._speaking:
            # TODO(REQ-XX) [TASK-XX]: stop the synthesis stream + flush speaker.
            self._speaking = False
            self.get_logger().info('(stub) barge-in: playback interrupted')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
