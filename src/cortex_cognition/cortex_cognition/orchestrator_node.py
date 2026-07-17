"""orchestrator_node — hook-driven scenario orchestrator (TaskSrv form, rclpy).

Port of the OM1 TaskSrvProvider pattern into an event-driven rclpy node. There
is NO LLM and NO separate router: a scenario declares its own trigger keywords,
and an STT transcript is matched against them to activate a scenario.

A scenario is a list of sub-tasks; each sub-task is a small lifecycle machine:

    on_create → on_start → (poll `success` each tick) → on_success | on_fail

Each hook is an ordered list of {label: payload} steps. The label selects a
**Connector** (speak / navigation / vla) — capability-separated channels, each
ultimately funnelling into Gearsonic's single-writer Handler on the onboard
side. Success is a polymorphic **Criterion**; the demo uses a VLM criterion that
reads the latest Verdict published by vlm_node (kept off this tick loop because
VLM inference is heavy — running it here would re-introduce the blocking-tick
problem OM1 had).

Scenarios are JSON5 data under config/scenarios/*.json5 (same schema as the old
workstation scenarios: name / triggers / sub_tasks / on_* / success).

Preemption is a two-phase handshake, not an instant switch: a new trigger asks
every connector to stop, buffers the new scenario in _pending, and enters
CANCELING. Only once the module confirms it stopped (_stopped) — or we hit
cancel_timeout and escalate to the onboard E-STOP — does the buffered scenario
start. This prevents the old and new motions overlapping on the robot.

STATE (per agreed direction):
  - trigger matching, hook lifecycle, preemption (CANCELING + pending buffer),
    VLM criterion, and the speak connector are wired.
  - navigation / vla connectors are stubs: their onboard destination topic/type
    is blocked on the Gearsonic Handler interface spec.
  - sensor criteria (uwb / joint) are stubs pending the onboard sensor caches.
  - the stop-confirmation channel (Handler CommandStatus, correlated by
    command_id) is wired in _stopped(), but gated by the assume_stopped param:
    no Handler publishes CommandStatus yet, so the dev default assumes stops
    succeed immediately. Flip assume_stopped=False to exercise the real path.
"""

import glob
import os
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import json5
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String

from cortex_msgs.msg import CommandStatus, Subtask, TaskStatus, Verdict


# ===========================================================================
# Scenario model + loader
# ===========================================================================
@dataclass
class SubTaskDef:
    name: str
    on_create: list = field(default_factory=list)   # [{label: payload}, ...]
    on_start: list = field(default_factory=list)
    on_success: list = field(default_factory=list)
    success: dict = field(default_factory=dict)      # criterion spec
    timeout_s: float = 30.0


@dataclass
class Scenario:
    name: str
    triggers: list           # keyword substrings matched against transcripts
    sub_tasks: list          # [SubTaskDef]


def load_scenarios(scenario_dir: str) -> list:
    """Load every *.json5 under scenario_dir into Scenario objects."""
    scenarios = []
    for path in sorted(glob.glob(os.path.join(scenario_dir, '*.json5'))):
        with open(path, 'r', encoding='utf-8') as f:
            raw = json5.load(f)
        subs = [
            SubTaskDef(
                name=s['name'],
                on_create=s.get('on_create', []),
                on_start=s.get('on_start', []),
                on_success=s.get('on_success', []),
                success=s.get('success', {}),
                timeout_s=float(s.get('success', {}).get('timeout_s', 30.0)),
            )
            for s in raw.get('sub_tasks', [])
        ]
        scenarios.append(Scenario(raw['name'], raw.get('triggers', []), subs))
    return scenarios


# ===========================================================================
# Criteria — polymorphic success rules
# ===========================================================================
class Criterion(ABC):
    @abstractmethod
    def evaluate(self, node: 'OrchestratorNode', subtask_id: str) -> bool: ...


class VlmCriterion(Criterion):
    """Pass when vlm_node's latest Verdict for this subtask says passed."""

    def evaluate(self, node, subtask_id) -> bool:
        v = node.latest_verdict
        return v is not None and v.subtask_id == subtask_id and v.passed


class _StubCriterion(Criterion):
    """Sensor criteria not wired yet — never falsely passes."""

    def __init__(self, kind: str):
        self.kind = kind

    def evaluate(self, node, subtask_id) -> bool:
        # TODO(REQ-XX) [TASK-XX]: read onboard uwb/joint caches on the bridge domain.
        return False


def build_criterion(spec: dict) -> Criterion:
    kind = spec.get('type', 'vlm')
    if kind == 'vlm':
        return VlmCriterion()
    # TODO(REQ-XX) [TASK-XX]: uwb_pose / joint_state / composite criteria.
    return _StubCriterion(kind)


# ===========================================================================
# Connectors — capability-separated dispatch channels (label -> connector)
# ===========================================================================
class Connector(ABC):
    # Motion connectors go through the Handler and report stop via CommandStatus;
    # speak (local TTS) does not participate in the stop-confirmation protocol.
    is_motion: bool = False

    @abstractmethod
    def dispatch(self, node: 'OrchestratorNode', payload, command_id: int) -> None:
        """Send the command, stamped with command_id so the Handler can echo it
        back in CommandStatus."""

    def cancel(self, node: 'OrchestratorNode', command_id: int) -> None:
        """Best-effort stop of command_id. Override where meaningful."""


class SpeakConnector(Connector):
    """`speak` label -> local TTS via tts_node. Not a Handler command."""

    def dispatch(self, node, payload, command_id) -> None:
        node.say_pub.publish(String(data=str(payload)))   # command_id unused


class NavigationConnector(Connector):
    """`navigation` label -> LocoCommand / named goal -> Gearsonic Handler."""

    is_motion = True

    def dispatch(self, node, payload, command_id) -> None:
        # TODO(REQ-XX) [TASK-XX]: publish LocoCommand / nav goal carrying
        # command_id, once the Gearsonic Handler input interface is specified.
        node.get_logger().info(f'(stub) navigation dispatch cmd#{command_id}: {payload!r}')

    def cancel(self, node, command_id) -> None:
        # TODO(REQ-XX) [TASK-XX]: send stop/hold for command_id to the Handler.
        node.get_logger().info(f'(stub) navigation cancel cmd#{command_id}')


class VlaConnector(Connector):
    """`vla` label -> arm/hand joint inference -> Gearsonic Handler."""

    is_motion = True

    def dispatch(self, node, payload, command_id) -> None:
        # TODO(REQ-XX) [TASK-XX]: publish VLA task cmd carrying command_id, once
        # the Gearsonic Handler input interface is specified.
        node.get_logger().info(f'(stub) vla dispatch cmd#{command_id}: {payload!r}')

    def cancel(self, node, command_id) -> None:
        node.get_logger().info(f'(stub) vla cancel cmd#{command_id}')


# ===========================================================================
# Node
# ===========================================================================
class OrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__('orchestrator_node')

        default_dir = os.path.join(
            get_package_share_directory('cortex_cognition'), 'scenarios')
        self.declare_parameter('scenario_dir', default_dir)
        self.declare_parameter('transcript_topic', '/cortex/stt/transcript')
        self.declare_parameter('status_topic', '/cortex/task_status')
        self.declare_parameter('verdict_topic', '/cortex/critic/verdict')
        self.declare_parameter('active_subtask_topic', '/cortex/active_subtask')
        self.declare_parameter('say_topic', '/cortex/tts/say')
        self.declare_parameter('handler_status_topic', '/bridge/handler/status')
        self.declare_parameter('tick_rate_hz', 10.0)
        # How long to wait for a stop-confirmation before escalating (E-STOP).
        self.declare_parameter('cancel_timeout_sec', 2.0)
        # DEV DEFAULT: no Handler publishes CommandStatus yet, so assume a cancel
        # stops immediately. Set FALSE to exercise the real CommandStatus path
        # (and once the Handler is live, remove this knob).
        self.declare_parameter('assume_stopped', True)

        g = self.get_parameter
        scenario_dir = g('scenario_dir').value
        tick_hz = float(g('tick_rate_hz').value)
        self._cancel_timeout = float(g('cancel_timeout_sec').value)
        self._assume_stopped = bool(g('assume_stopped').value)

        # --- connectors (label -> connector) ------------------------------
        self.connectors = {
            'speak': SpeakConnector(),
            'navigation': NavigationConnector(),
            'vla': VlaConnector(),
        }

        # --- scenarios ----------------------------------------------------
        self.scenarios = load_scenarios(scenario_dir)
        self.get_logger().info(
            f'loaded {len(self.scenarios)} scenario(s) from {scenario_dir}')

        # --- run state (mutated only inside the mutually-exclusive callback
        #     group below, so never by two threads at once — see grp) --------
        self.latest_verdict = None       # cached Verdict from vlm_node
        self._active = None              # active Scenario
        self._index = 0                  # current sub-task index
        self._criterion = None           # current sub-task's Criterion
        self._started = False            # on_start fired for current sub-task?
        self._t0 = 0.0                   # current sub-task start time
        # --- preemption / CANCELING state ---
        self._canceling = False          # waiting for the module to confirm stop
        self._cancel_t0 = 0.0            # when the current cancel began
        self._pending = None             # Scenario to start once stop is confirmed
        # --- command correlation (stop-confirmation via CommandStatus) ---
        self._handler_status = None      # latest CommandStatus from the Handler
        self._command_seq = 0            # monotonic command id source
        self._active_command_id = 0      # last motion command dispatched
        self._canceling_command_id = 0   # the command we're waiting to see stopped

        # --- io -----------------------------------------------------------
        # One mutually-exclusive group for transcript / verdict / tick so state
        # (_active, _index, latest_verdict, ...) has a single writer at a time.
        # Ticks are cheap (read cache + publish), so serializing costs nothing.
        grp = MutuallyExclusiveCallbackGroup()
        self.status_pub = self.create_publisher(TaskStatus, g('status_topic').value, 10)
        self.active_pub = self.create_publisher(Subtask, g('active_subtask_topic').value, 10)
        self.say_pub = self.create_publisher(String, g('say_topic').value, 10)

        self.create_subscription(
            String, g('transcript_topic').value, self._on_transcript, 10, callback_group=grp)
        self.create_subscription(
            Verdict, g('verdict_topic').value, self._on_verdict, 10, callback_group=grp)

        # Handler status: latched, reliable stream (see CommandStatus.msg) so a
        # dropped transition / late subscriber is self-healing.
        status_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            CommandStatus, g('handler_status_topic').value, self._on_handler_status,
            status_qos, callback_group=grp)

        self.create_timer(1.0 / tick_hz, self._tick, callback_group=grp)
        self.get_logger().info(f'orchestrator_node up (tick={tick_hz}Hz)')

    # --- inputs -----------------------------------------------------------
    def _on_transcript(self, msg: String) -> None:
        text = msg.data
        for sc in self.scenarios:
            if any(kw in text for kw in sc.triggers):
                self._request(sc)
                return
        # No trigger matched — ignore (not every utterance is a command).

    def _on_verdict(self, msg: Verdict) -> None:
        self.latest_verdict = msg

    def _on_handler_status(self, msg: CommandStatus) -> None:
        self._handler_status = msg          # cache; _stopped() reads it

    def _next_command_id(self) -> int:
        self._command_seq += 1
        return self._command_seq

    # --- scenario lifecycle -----------------------------------------------
    def _request(self, sc: Scenario) -> None:
        """A trigger matched. Start now if idle; otherwise preempt: cancel the
        running scenario and BUFFER this one until the stop is confirmed."""
        if self._active is None and not self._canceling:
            self._begin(sc)
            return
        # Buffer the newest request (latest wins) and start canceling if not yet.
        self._pending = sc
        if not self._canceling:
            self._publish_status(TaskStatus.STATE_PREEMPTED, detail='preempted by new trigger')
            self._begin_cancel('new trigger')

    def _begin(self, sc: Scenario) -> None:
        self.get_logger().info(f'begin scenario {sc.name!r} ({len(sc.sub_tasks)} sub-tasks)')
        self._active = sc
        self._index = 0
        self._enter_subtask()

    # --- CANCELING: stop the module, wait for confirmation, then continue ---
    def _begin_cancel(self, reason: str) -> None:
        """Ask every connector to stop and enter CANCELING. We do NOT start the
        next command until _stopped() confirms (or we escalate on timeout)."""
        self._canceling_command_id = self._active_command_id
        self._cancel_connectors(self._canceling_command_id)
        self._canceling = True
        self._cancel_t0 = self._now()
        self._reset_exec()               # stop driving the old scenario's lifecycle
        self.get_logger().info(
            f'canceling cmd#{self._canceling_command_id} ({reason}) '
            '— waiting for stop confirmation')

    def _service_cancel(self) -> None:
        """Called each tick while CANCELING."""
        if self._stopped():
            self._finish_cancel()
        elif self._now() - self._cancel_t0 >= self._cancel_timeout:
            self._escalate()

    def _finish_cancel(self) -> None:
        self._canceling = False
        pending, self._pending = self._pending, None
        if pending is not None:
            self._begin(pending)         # buffered command runs now that we stopped
        # else: nothing queued -> stay idle

    def _escalate(self) -> None:
        self.get_logger().error(
            'stop NOT confirmed within cancel_timeout — escalating (E-STOP)')
        # TODO(REQ-XX) [TASK-XX]: trigger the onboard hard-RT E-STOP / hold.
        # Safety must NOT depend on cortex — this only *requests* the onboard
        # safety_monitor to stop; that node is the real guarantee.
        self._finish_cancel()            # proceed once safed

    def _stopped(self) -> bool:
        """Has the module actually stopped the command we canceled?

        Reads the latest CommandStatus from the Handler and correlates by
        command_id. This is the real confirmation channel; it only works once
        the Handler actually publishes CommandStatus. Until then, run with
        assume_stopped=True (the dev default).
        """
        if self._assume_stopped:
            return True                  # DEV: no Handler yet — assume it stopped
        s = self._handler_status
        if s is None:
            return False                 # no status yet -> can't confirm -> wait
        if s.state == CommandStatus.IDLE:
            return True                  # Handler has no active command -> stopped
        if s.state in (CommandStatus.CANCELED, CommandStatus.ABORTED):
            return s.command_id == self._canceling_command_id
        return False                     # ACCEPTED / EXECUTING / CANCELING -> not yet

    def _enter_subtask(self) -> None:
        st = self._current()
        if st is None:
            return
        self.latest_verdict = None
        self._criterion = build_criterion(st.success)
        self._started = False
        self._t0 = self._now()
        self._dispatch(st.on_create)                 # fire-and-forget announce
        # Tell vlm_node what to judge (id + the vlm check text).
        sub = Subtask()
        sub.id = st.name
        sub.success_check = str(st.success.get('check', st.success.get('type', '')))
        sub.timeout_sec = st.timeout_s
        self.active_pub.publish(sub)
        self._publish_status(TaskStatus.STATE_RUNNING, current_subtask=st.name)

    def _tick(self) -> None:
        # While canceling, drive only the stop-confirmation; the lifecycle is
        # frozen until the module confirms it stopped.
        if self._canceling:
            self._service_cancel()
            return
        st = self._current()
        if st is None:
            return
        if not self._started:
            # New motion command -> new id the Handler will echo in CommandStatus.
            self._active_command_id = self._next_command_id()
            self._dispatch(st.on_start)              # fire motion once
            self._started = True
            return
        if self._criterion.evaluate(self, st.name):
            self._dispatch(st.on_success)
            self._advance()
        elif self._now() - self._t0 >= st.timeout_s:
            self._fail(f'{st.name} timeout')

    def _advance(self) -> None:
        self._index += 1
        if self._current() is not None:
            self._enter_subtask()
        else:
            self._publish_status(TaskStatus.STATE_SUCCEEDED)
            self._reset_exec()

    def _fail(self, reason: str) -> None:
        # Report failure, then stop the module and wait for confirmation
        # (go idle only once stopped — no _pending queued).
        self._publish_status(TaskStatus.STATE_FAILED, detail=reason)
        self.connectors['speak'].dispatch(self, f'실패: {reason}')
        self._begin_cancel(reason)

    # --- helpers ----------------------------------------------------------
    def _dispatch(self, hooks: list) -> None:
        """Run a hook list: each {label: payload} routes to its connector,
        stamped with the current motion command id."""
        for step in hooks:
            for label, payload in step.items():
                conn = self.connectors.get(label)
                if conn is None:
                    self.get_logger().warning(f'no connector for label {label!r}')
                    continue
                conn.dispatch(self, payload, self._active_command_id)

    def _cancel_connectors(self, command_id: int) -> None:
        for conn in self.connectors.values():
            conn.cancel(self, command_id)

    def _current(self):
        if self._active and 0 <= self._index < len(self._active.sub_tasks):
            return self._active.sub_tasks[self._index]
        return None

    def _reset_exec(self) -> None:
        """Clear sub-task execution state. Does NOT touch _canceling / _pending."""
        self._active = None
        self._index = 0
        self._criterion = None
        self._started = False

    def _now(self) -> float:
        return time.monotonic()

    def _publish_status(self, state, current_subtask='', detail='') -> None:
        msg = TaskStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_name = self._active.name if self._active else ''
        msg.current_subtask = current_subtask
        msg.subtask_index = self._index
        msg.subtask_count = len(self._active.sub_tasks) if self._active else 0
        msg.state = state
        msg.detail = detail
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrchestratorNode()
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
