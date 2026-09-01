"""VLM node — on-demand scene judge, VLA prompt grounder, precondition judge.

vlm_node is IDLE by default. It acts only when asked, in two moments:

  1. SUB-TASK ENTRY (active-subtask topic). If the Subtask carries a
     goal_text, ground it against the live frame into a VlaPrompt; if it
     carries a precondition_check, judge that too (same VLM call when
     grounded, dedicated call + PreconditionReport otherwise). Then idle.

  2. JUDGMENT REQUEST (VerdictRequest). The orchestrator sends exactly one
     when every motion of the in-flight sub-task has reported done
     (CommandStatus) — judge the scene against success_check ONCE and answer
     with ONE Verdict. No periodic critic loop, no progress gate: the
     completion signal owns "when", the VLM owns "whether".

Why on-demand (vs the old 1 Hz critic loop + progress_gate):
- VLM cost drops from ~1 call/s to ~1 call/sub-task.
- The VLA -> VLM task_progress side channel disappears — completion signals
  route through the orchestrator like every other coordination path.
- "Judge near completion" stops being a threshold heuristic and becomes the
  literal contract: judge when the module says it finished.

Runs off the control critical path. Wiring is complete; the VLM backend
itself is a stub (_evaluate / _ground / _check_precondition).
"""

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cortex_msgs.msg import (
    PreconditionReport, Subtask, Verdict, VerdictRequest, VlaPrompt,
)

# from sensor_msgs.msg import Image        # camera on the bridge domain


class VlmNode(Node):
    def __init__(self) -> None:
        super().__init__('vlm_node')

        self.declare_parameter('image_topic', '/bridge/sensors/camera/color')
        self.declare_parameter('active_subtask_topic', '/cortex/active_subtask')
        self.declare_parameter('verdict_topic', '/cortex/critic/verdict')
        self.declare_parameter('verdict_request_topic', '/cortex/critic/request')
        self.declare_parameter('vla_prompt_topic', '/cortex/vla/prompt')
        self.declare_parameter('precondition_report_topic', '/cortex/precondition/report')

        g = self.get_parameter
        image_topic = g('image_topic').value

        self._lock = threading.Lock()
        self._current: Subtask | None = None   # for stale checks on entry work
        self._latest_frame = None              # most recent camera frame

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
            VerdictRequest, g('verdict_request_topic').value,
            self._on_verdict_request, 10, callback_group=grp)

        # TODO(REQ-XX) [TASK-XX]: subscribe to the camera Image and cache frames:
        #   self.create_subscription(Image, image_topic, self._on_frame, 10, ...)
        # TODO(REQ-XX) [TASK-XX]: load the VLM backend used by the stubs below.

        self.get_logger().info(
            f"vlm_node up (on-demand: request={g('verdict_request_topic').value} "
            f"-> verdict={g('verdict_topic').value})")

    # --- sub-task entry: grounding + precondition -------------------------
    def _on_active_subtask(self, msg: Subtask) -> None:
        with self._lock:
            self._current = msg
        self.get_logger().info(
            f'sub-task {msg.id!r} entered (goal={msg.goal_text!r}, '
            f'precondition={msg.precondition_check!r}) — awaiting requests')
        if msg.goal_text:
            # One Subtask message = one grounding (re-running a scenario
            # re-grounds — the scene may have changed). Off the callback
            # thread: VLM inference must not block other callbacks.
            threading.Thread(
                target=self._ground_and_publish, args=(msg,), daemon=True).start()
        elif msg.precondition_check:
            # No grounding round-trip to ride (nav etc.) — dedicated check.
            # The orchestrator defers on_start bounded by its own fail-open
            # deadline, so a slow/dead backend delays but never strands it.
            threading.Thread(
                target=self._precheck_and_publish, args=(msg,), daemon=True).start()

    # --- on-demand critic -------------------------------------------------
    def _on_verdict_request(self, msg: VerdictRequest) -> None:
        # One request -> one verdict. Judged in a worker thread; the
        # orchestrator matches the answer by subtask_id, so a stale verdict
        # (sub-task advanced meanwhile) is dropped on its side.
        threading.Thread(
            target=self._judge_and_publish, args=(msg,), daemon=True).start()

    def _judge_and_publish(self, req: VerdictRequest) -> None:
        passed, confidence, reason = self._evaluate(
            req.success_check, self._latest_frame)
        v = Verdict()
        v.header.stamp = self.get_clock().now().to_msg()
        v.subtask_id = req.subtask_id
        v.passed = passed
        v.confidence = float(confidence)
        v.reason = reason
        self.pub.publish(v)
        self.get_logger().info(
            f'verdict {req.subtask_id!r}: passed={passed} ({reason})')

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
        # optimization (skip a doomed timeout), not a safety interlock.
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

    # --- precondition (non-grounded) --------------------------------------
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
        """Return (met, why).

        STUB: returns met=True (fail-open) so shadow logs stay quiet until a
        real backend produces actual judgments. The stub must never block a
        motion — the deliberate opposite of _evaluate's always-False.
        """
        # TODO(REQ-XX) [TASK-XX]: run the VLM against `frame` + `check`.
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
