"""VLM scene critic + VLA prompt grounder (VLM as monitor, not orchestrator).

Two jobs, both keyed to the sub-task orchestrator_node is executing (received
on the active-subtask topic):

  1. CRITIC — periodically judge whether the sub-task's `success_check` is met
     in the latest camera frame and publish a Verdict. When the sub-task
     carries a progress_gate, judging is HELD until the VLA policy's
     task_progress (from kist-vla-inference) reaches the gate — a mid-motion
     scene must not be judged as failure. The gate defers judgment; it never
     passes anything by itself.

  2. GROUNDER — when the sub-task carries a goal_text, ground that abstract
     goal against the live scene into the imperative the VLA policy was
     trained on ("Open the right door of the refrigerator. Hook the yellow
     tip attached to your right hand under the door handle and pull") and
     publish it as a VlaPrompt. Once per sub-task, in a worker thread —
     grounding latency must not stall the critic tick.

Runs off the control critical path — it monitors and grounds, it does not
plan. Its cadence (period_sec) is independent of the VLA/Gearsonic loop.

Wiring is complete; the VLM backend itself is a stub (_evaluate / _ground).
"""

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32

from cortex_msgs.msg import PreconditionReport, Subtask, Verdict, VlaPrompt

# from sensor_msgs.msg import Image        # camera on the bridge domain


class VlmNode(Node):
    def __init__(self) -> None:
        super().__init__('vlm_node')

        self.declare_parameter('image_topic', '/bridge/sensors/camera/color')
        self.declare_parameter('active_subtask_topic', '/cortex/active_subtask')
        self.declare_parameter('verdict_topic', '/cortex/critic/verdict')
        self.declare_parameter('vla_prompt_topic', '/cortex/vla/prompt')
        self.declare_parameter('precondition_report_topic', '/cortex/precondition/report')
        # Published by kist-vla-inference (task_progress head of the policy —
        # currently dropped in its runner; wiring tracked in the ICD).
        self.declare_parameter('progress_topic', '/cortex/vla/task_progress')
        self.declare_parameter('period_sec', 1.0)   # critic cadence, not control rate

        g = self.get_parameter
        image_topic = g('image_topic').value
        period = float(g('period_sec').value)

        self._lock = threading.Lock()
        self._current: Subtask | None = None   # subtask under evaluation
        self._latest_frame = None              # most recent camera frame
        self._progress: float | None = None    # latest VLA task_progress (this subtask)

        grp = ReentrantCallbackGroup()
        self.pub = self.create_publisher(Verdict, g('verdict_topic').value, 10)
        self.prompt_pub = self.create_publisher(
            VlaPrompt, g('vla_prompt_topic').value, 10)
        self.precondition_pub = self.create_publisher(
            PreconditionReport, g('precondition_report_topic').value, 10)
        self.create_subscription(
            Subtask, g('active_subtask_topic').value, self._on_active_subtask, 10,
            callback_group=grp)
        self.create_subscription(
            Float32, g('progress_topic').value, self._on_progress, 10,
            callback_group=grp)

        # TODO(REQ-XX) [TASK-XX]: subscribe to the camera Image and cache frames:
        #   self.create_subscription(Image, image_topic, self._on_frame, 10, ...)
        # TODO(REQ-XX) [TASK-XX]: load the VLM backend used by _evaluate/_ground.

        self.create_timer(period, self._tick, callback_group=grp)
        self.get_logger().info(
            f"vlm_node up (verdict={g('verdict_topic').value}, "
            f"prompt={g('vla_prompt_topic').value}, {period}s)")

    # --- inputs -----------------------------------------------------------
    def _on_active_subtask(self, msg: Subtask) -> None:
        with self._lock:
            self._current = msg
            # task_progress belongs to one VLA execution; a value from the
            # previous sub-task's motion must not open the new gate.
            self._progress = None
        self.get_logger().info(
            f'now judging subtask {msg.id!r} (check={msg.success_check!r}, '
            f'gate={msg.progress_gate}, goal={msg.goal_text!r})')
        if msg.goal_text:
            # The orchestrator publishes one Subtask per sub-task ENTRY, so one
            # message = one grounding (re-running a scenario re-grounds — the
            # scene may have changed). Off the callback thread: grounding is a
            # VLM inference and must not block subtask/progress callbacks.
            threading.Thread(
                target=self._ground_and_publish, args=(msg,), daemon=True).start()
        elif msg.precondition_check:
            # No grounding round-trip to ride (nav etc.) — dedicated check.
            # The orchestrator is deferring on_start for this report, bounded
            # by its own fail-open deadline, so a slow/dead backend here can
            # delay but never strand the motion.
            threading.Thread(
                target=self._precheck_and_publish, args=(msg,), daemon=True).start()

    def _on_progress(self, msg: Float32) -> None:
        with self._lock:
            self._progress = float(msg.data)

    # --- critic -----------------------------------------------------------
    def _tick(self) -> None:
        with self._lock:
            cur = self._current
            frame = self._latest_frame
            progress = self._progress
        if cur is None:
            return  # nothing in flight

        # Progress gate: defer judgment while the arm policy says the motion
        # is still under way. No Verdict at all is published — silence here
        # means "not yet", and the orchestrator's timeout still bounds it.
        if cur.progress_gate > 0.0 and (progress is None
                                        or progress < cur.progress_gate):
            return

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

    # --- grounder ---------------------------------------------------------
    def _ground_and_publish(self, sub: Subtask) -> None:
        # One VLM call answers both questions (same frame, no extra latency):
        # the grounded imperative AND — when the sub-task carries one — whether
        # the precondition holds. FAIL-OPEN on the precondition: any backend
        # error or unparseable answer reports met=True, because this gate is an
        # optimization (skip a doomed 20s timeout), not a safety interlock.
        text = self._ground(sub.goal_text, self._latest_frame)
        met, why = True, ''
        if sub.precondition_check:
            try:
                met, why = self._check_precondition(
                    sub.precondition_check, self._latest_frame)
            except Exception as exc:  # noqa: BLE001 — fail-open by design
                self.get_logger().warning(
                    f'precondition check errored (fail-open): {exc}')
                met, why = True, ''
        with self._lock:
            still_active = self._current is not None and self._current.id == sub.id
        if not still_active:
            return  # superseded while grounding — a stale prompt must not fire the arm
        p = VlaPrompt()
        p.header.stamp = self.get_clock().now().to_msg()
        p.subtask_id = sub.id
        p.text = text
        p.precondition_met = met
        p.precondition_why = why
        self.prompt_pub.publish(p)
        self.get_logger().info(
            f'grounded {sub.id!r}: {text!r} (precondition_met={met})')

    def _ground(self, goal_text: str, frame) -> str:
        """Ground an abstract goal against the scene into a VLA imperative.

        STUB: echoes the goal. A real backend describes the scene (handle
        side, marker positions, obstacles) and rewrites the goal into the
        scene-specific imperative phrasing the VLA policy was trained on.
        """
        # TODO(REQ-XX) [TASK-XX]: run the VLM against `frame` + `goal_text`.
        return goal_text

    def _precheck_and_publish(self, sub: Subtask) -> None:
        """Dedicated precondition check for sub-tasks without a grounded goal."""
        try:
            met, why = self._check_precondition(
                sub.precondition_check, self._latest_frame)
        except Exception as exc:  # noqa: BLE001 — fail-open by design
            self.get_logger().warning(
                f'precondition check errored (fail-open): {exc}')
            met, why = True, ''
        with self._lock:
            still_active = self._current is not None and self._current.id == sub.id
        if not still_active:
            return  # superseded — a stale report must not gate a new sub-task
        r = PreconditionReport()
        r.header.stamp = self.get_clock().now().to_msg()
        r.subtask_id = sub.id
        r.met = met
        r.why = why
        self.precondition_pub.publish(r)
        self.get_logger().info(
            f'precondition {sub.id!r}: met={met}'
            + (f' ({why})' if not met else ''))

    def _check_precondition(self, check: str, frame) -> tuple:
        """Return (met, why). Judged in the same grounding call once the real
        backend lands — ask for prompt + {"precondition_met", "why"} in one
        structured response.

        STUB: returns met=True (fail-open) so shadow logs stay quiet until a
        real backend produces actual judgments. The stub must never block a
        motion — the deliberate opposite of _evaluate's always-False.
        """
        # TODO(REQ-XX) [TASK-XX]: fold into the _ground VLM call.
        return True, 'vlm backend not wired (fail-open)'

    def _on_frame(self, msg) -> None:
        with self._lock:
            self._latest_frame = msg


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
