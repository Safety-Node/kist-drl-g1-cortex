"""VLM scene critic (option 1: VLM as monitor, not orchestrator).

Tracks the subtask orchestrator_node is currently executing (received on the
active-subtask topic) and periodically judges whether that subtask's
`success_check` is met in the latest camera frame. Publishes a Verdict;
orchestrator advances the plan on pass and preempts on fail.

Runs off the control critical path — it monitors, it does not plan. Its cadence
(period_sec) is independent of the VLA/Gearsonic real-time loop.

Wiring is complete; the VLM backend itself is a stub (_evaluate).
"""

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cortex_msgs.msg import Subtask, Verdict

# from sensor_msgs.msg import Image        # camera on the bridge domain


class VlmNode(Node):
    def __init__(self) -> None:
        super().__init__('vlm_node')

        self.declare_parameter('image_topic', '/bridge/sensors/camera/color')
        self.declare_parameter('active_subtask_topic', '/cortex/active_subtask')
        self.declare_parameter('verdict_topic', '/cortex/critic/verdict')
        self.declare_parameter('period_sec', 1.0)   # critic cadence, not control rate

        image_topic = self.get_parameter('image_topic').value
        active_subtask_topic = self.get_parameter('active_subtask_topic').value
        verdict_topic = self.get_parameter('verdict_topic').value
        period = float(self.get_parameter('period_sec').value)

        self._lock = threading.Lock()
        self._current: Subtask | None = None   # subtask under evaluation
        self._latest_frame = None              # most recent camera frame

        grp = ReentrantCallbackGroup()
        self.pub = self.create_publisher(Verdict, verdict_topic, 10)
        self.create_subscription(
            Subtask, active_subtask_topic, self._on_active_subtask, 10, callback_group=grp)

        # TODO(REQ-XX) [TASK-XX]: subscribe to the camera Image and cache frames:
        #   self.create_subscription(Image, image_topic, self._on_frame, 10, ...)
        # TODO(REQ-XX) [TASK-XX]: load the (simple) VLM backend used by _evaluate.

        self.create_timer(period, self._tick, callback_group=grp)
        self.get_logger().info(
            f'vlm_node up (active={active_subtask_topic} -> verdict={verdict_topic}, {period}s)')

    def _on_active_subtask(self, msg: Subtask) -> None:
        with self._lock:
            self._current = msg
        self.get_logger().info(
            f'now judging subtask {msg.id!r} (check={msg.success_check!r})')

    def _tick(self) -> None:
        with self._lock:
            cur = self._current
            frame = self._latest_frame
        if cur is None:
            return  # nothing in flight

        passed, confidence, reason = self._evaluate(cur.success_check, frame)

        v = Verdict()
        v.header.stamp = self.get_clock().now().to_msg()
        v.subtask_id = cur.id
        v.passed = passed
        v.confidence = float(confidence)
        v.reason = reason
        self.pub.publish(v)

    def _evaluate(self, success_check: str, frame):
        """Return (passed, confidence, reason). Pure w.r.t. inputs.

        STUB: no VLM backend yet. Returns "not yet" so the loop is exercisable
        without ever falsely passing a subtask.
        """
        # TODO(REQ-XX) [TASK-XX]: run the VLM against `frame` + `success_check`.
        return False, 0.0, 'vlm backend not wired'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VlmNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
