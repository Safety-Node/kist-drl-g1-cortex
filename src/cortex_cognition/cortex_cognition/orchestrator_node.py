"""orchestrator_node — hook-driven scenario orchestrator (TaskSrv form, rclpy).

No LLM, no router: a scenario declares its own trigger keywords and an STT
transcript is matched against them. A scenario is a list of sub-tasks, each a
small lifecycle machine:

    on_create → on_start → (poll `success` each tick) → on_success | on_fail

A hook step is {label: payload}; the label picks a Connector (speak / navigation
/ vla). `success` is a polymorphic Criterion (vlm reads vlm_node's Verdict off
the tick loop — heavy inference must not block it). Scenarios are JSON5 under
config/scenarios/.

Every tracked action is registered in {command_id -> InflightCmd}; modules echo
the command_id in a CommandStatus stream and the registry drops a command on a
terminal state, so the loop reasons over the set regardless of how many actions a
sub-task fires. Preemption is a handshake: a new trigger cancels every in-flight
command, buffers the new scenario, and only starts it once the registry empties
(or cancel_timeout → escalate). _check_liveness faults a command whose status
goes stale.

Wired: everything above + speak (tts_node echoes CommandStatus). Stub: nav/vla
connectors (no CommandStatus yet) and uwb/joint criteria — blocked on the
Gearsonic Handler / onboard sensors. Hence the dev defaults assume_stopped=True
and monitor_liveness=False; flip both on with the real sources.
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
from std_msgs.msg import Bool, String

from cortex_msgs.msg import (CommandStatus, SpeakCommand, Subtask, TaskStatus,
                             Verdict)

# CommandStatus states that mean "this command has stopped" — the registry drops
# a command when it reaches one, and a cancel is done when the registry is empty.
_TERMINAL = (CommandStatus.IDLE, CommandStatus.CANCELED,
             CommandStatus.SUCCEEDED, CommandStatus.ABORTED)


@dataclass
class InflightCmd:
    """One tracked command the orchestrator has dispatched and is monitoring."""
    connector: str          # which connector owns it (for cancel / escalate)
    state: int              # latest CommandStatus.state
    last_seen: float        # monotonic time of the last status update (staleness)


# ===========================================================================
# Scenario model + loader
# ===========================================================================
@dataclass
class SubTaskDef:
    name: str
    on_create: list = field(default_factory=list)   # [{label: payload}, ...]
    on_start: list = field(default_factory=list)
    on_success: list = field(default_factory=list)
    on_fail: list = field(default_factory=list)     # fired on timeout; owns the message
    success: dict = field(default_factory=dict)     # raw spec (kept for the vlm check text)
    criterion: 'Criterion' = None                   # built at LOAD -> fail fast
    timeout_s: float = 30.0


@dataclass
class Scenario:
    name: str
    triggers: list           # keyword substrings matched against transcripts
    sub_tasks: list          # [SubTaskDef]. Empty == a pure "stop": preempt, then idle.


def _load_subtask(s: dict) -> SubTaskDef:
    spec = s.get('success', {})
    return SubTaskDef(
        name=s['name'],
        on_create=s.get('on_create', []),
        on_start=s.get('on_start', []),
        on_success=s.get('on_success', []),
        on_fail=s.get('on_fail', []),
        success=spec,
        criterion=build_criterion(spec),            # raises ScenarioConfigError here
        timeout_s=float(spec.get('timeout_s', 30.0)),
    )


def load_scenarios(scenario_dir: str) -> list:
    """Load every *.json5 under scenario_dir into Scenario objects.

    Criteria are built here, so a bad scenario stops the node at startup with the
    offending file named — rather than loading fine and then never passing.
    """
    scenarios = []
    for path in sorted(glob.glob(os.path.join(scenario_dir, '*.json5'))):
        with open(path, 'r', encoding='utf-8') as f:
            raw = json5.load(f)
        try:
            subs = [_load_subtask(s) for s in raw.get('sub_tasks', [])]
            scenarios.append(Scenario(raw['name'], raw.get('triggers', []), subs))
        except (ScenarioConfigError, KeyError, TypeError, ValueError) as exc:
            raise ScenarioConfigError(f'{path}: {exc}') from exc
    return scenarios


# ===========================================================================
# Criteria — polymorphic success rules
# ===========================================================================
class ScenarioConfigError(ValueError):
    """Bad scenario. Raised at LOAD, not run — a typo must fail fast, not degrade
    into a sub-task that never passes and 'fails' on timeout 15 s later."""


class Criterion(ABC):
    @abstractmethod
    def evaluate(self, node: 'OrchestratorNode', subtask_id: str) -> bool: ...


class VlmCriterion(Criterion):
    """Pass when vlm_node's latest Verdict for this sub-task says passed."""

    def evaluate(self, node, subtask_id) -> bool:
        v = node.latest_verdict
        return v is not None and v.subtask_id == subtask_id and v.passed


@dataclass
class DelayCriterion(Criterion):
    """Pass ``seconds`` after on_start (not entry — see _tick). Placeholder that
    asserts nothing about the world; replace with uwb_pose / vlm once wired."""

    seconds: float = 0.0

    def evaluate(self, node, subtask_id) -> bool:
        return node.elapsed() >= self.seconds


@dataclass
class VoiceKeywordCriterion(Criterion):
    """Pass on an utterance heard SINCE THIS SUB-TASK STARTED. For context replies
    ("응" / "오리엔탈로"); an independent command belongs in `triggers`, not here."""

    keywords: list = field(default_factory=list)

    def evaluate(self, node, subtask_id) -> bool:
        return any(kw in t for t in node.transcripts for kw in self.keywords)


@dataclass
class CompositeCriterion(Criterion):
    """All-of (AND). Worth it only when children catch different failure modes and
    a false positive is costlier — P(all) is a product, so AND lowers the pass
    rate and adds false negatives. (grasp: joint AND vlm; arrival: uwb alone.)"""

    children: list = field(default_factory=list)

    def evaluate(self, node, subtask_id) -> bool:
        return all(c.evaluate(node, subtask_id) for c in self.children)


class AlwaysCriterion(Criterion):
    """Pass immediately. For smoke-testing the dispatch path."""

    def evaluate(self, node, subtask_id) -> bool:
        return True


def _req(spec: dict, key: str, tag: str):
    if key not in spec:
        raise ScenarioConfigError(f'criterion {tag!r} requires {key!r}')
    return spec[key]


_CRITERION_BUILDERS = {
    'vlm': lambda s: VlmCriterion(),
    'always': lambda s: AlwaysCriterion(),
    'delay': lambda s: DelayCriterion(seconds=float(_req(s, 'seconds', 'delay'))),
    'voice_keyword': lambda s: VoiceKeywordCriterion(
        keywords=list(_req(s, 'keywords', 'voice_keyword'))),
    'composite': lambda s: CompositeCriterion(
        children=[build_criterion(c) for c in _req(s, 'children', 'composite')]),
}

# Known in the workstation but not ported — named so they fail at load with a
# reason instead of looking like a typo (or worse, silently never passing).
_NOT_PORTED = {
    'uwb_pose': 'needs an onboard pose subscription (not wired)',
    'joint_state': 'needs an onboard joint_states subscription (not wired)',
    'voice_choice': 'needs the scenario blackboard (not ported)',
}


def build_criterion(spec: dict) -> Criterion:
    tag = spec.get('type')   # required — a missing type used to default to a never-passing vlm
    if tag is None:
        raise ScenarioConfigError(
            f'success.type is required; known: {sorted(_CRITERION_BUILDERS)}')
    builder = _CRITERION_BUILDERS.get(tag)
    if builder is None:
        if tag in _NOT_PORTED:
            raise ScenarioConfigError(
                f'criterion {tag!r} is not available yet: {_NOT_PORTED[tag]}')
        raise ScenarioConfigError(
            f'unknown criterion type {tag!r}; known: {sorted(_CRITERION_BUILDERS)}')
    return builder(spec)


# ===========================================================================
# Connectors — capability-separated dispatch channels (label -> connector)
# ===========================================================================
class Connector(ABC):
    # Every tracked connector is stamped with a command_id, registered, and
    # monitored the same way. dispatch/cancel/escalate are abstract because both
    # cancel and escalate are safety-relevant — no silent no-op inheritance.
    is_tracked: bool = True

    @abstractmethod
    def dispatch(self, node: 'OrchestratorNode', payload, command_id: int) -> None: ...

    @abstractmethod
    def cancel(self, node: 'OrchestratorNode', command_id: int) -> None: ...

    @abstractmethod
    def escalate(self, node: 'OrchestratorNode', command_id: int) -> None:
        """Last resort when cancel is not confirmed in time (motion → E-STOP,
        speech → speaker flush)."""


class SpeakConnector(Connector):
    """`speak` -> tts_node. tts echoes command_id in CommandStatus, so tracked."""

    def dispatch(self, node, payload, command_id) -> None:
        node.say_pub.publish(SpeakCommand(command_id=command_id, text=str(payload)))

    def cancel(self, node, command_id) -> None:
        # tts has a single in-flight synthesis, so barge-in cuts "the current one"
        # (no id needed). Only cancels PC-side synthesis; already-published audio
        # is not recalled. Correlation still holds: tts reports CANCELED with the id.
        node.stop_pub.publish(Bool(data=True))

    def escalate(self, node, command_id) -> None:
        # TODO(REQ-XX): flush the NX speaker queue — speaker module not wired yet.
        node.get_logger().warning(f'(stub) speak escalate cmd#{command_id} — flush TBD')


class NavigationConnector(Connector):
    """`navigation` -> LocoCommand / named goal -> Gearsonic Handler (stub)."""

    def dispatch(self, node, payload, command_id) -> None:
        node.get_logger().info(f'(stub) navigation dispatch cmd#{command_id}: {payload!r}')

    def cancel(self, node, command_id) -> None:
        node.get_logger().info(f'(stub) navigation cancel cmd#{command_id}')

    def escalate(self, node, command_id) -> None:
        node.get_logger().error(f'(stub) navigation escalate cmd#{command_id} — E-STOP TBD')


class VlaConnector(Connector):
    """`vla` -> arm/hand joint inference -> Gearsonic Handler (stub)."""

    def dispatch(self, node, payload, command_id) -> None:
        node.get_logger().info(f'(stub) vla dispatch cmd#{command_id}: {payload!r}')

    def cancel(self, node, command_id) -> None:
        node.get_logger().info(f'(stub) vla cancel cmd#{command_id}')

    def escalate(self, node, command_id) -> None:
        node.get_logger().error(f'(stub) vla escalate cmd#{command_id} — E-STOP TBD')


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
        self.declare_parameter('stop_topic', '/cortex/tts/stop')   # tts barge-in
        # CommandStatus sources feeding the registry: the (future) Handler for
        # nav/vla, and tts_node for speak. Both publish CommandStatus.
        self.declare_parameter('handler_status_topic', '/bridge/handler/status')
        self.declare_parameter('tts_status_topic', '/cortex/tts/status')
        self.declare_parameter('tick_rate_hz', 10.0)
        # How long to wait for a stop-confirmation before escalating.
        self.declare_parameter('cancel_timeout_sec', 2.0)
        # DEV DEFAULT: no Handler publishes CommandStatus yet, so assume a cancel
        # stops immediately. Set FALSE to exercise the real registry path.
        self.declare_parameter('assume_stopped', True)
        # Liveness: fault a tracked command whose status has not updated within
        # stale_sec. OFF by default — with stub connectors there is no source, so
        # it would false-fault. Turn on with the real sources.
        self.declare_parameter('monitor_liveness', False)
        self.declare_parameter('stale_sec', 0.5)

        g = self.get_parameter
        scenario_dir = g('scenario_dir').value
        tick_hz = float(g('tick_rate_hz').value)
        self._cancel_timeout = float(g('cancel_timeout_sec').value)
        self._assume_stopped = bool(g('assume_stopped').value)
        self._monitor_liveness = bool(g('monitor_liveness').value)
        self._stale_sec = float(g('stale_sec').value)

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
        # --- action registry (command_id -> InflightCmd) ------------------
        # Every tracked command the orchestrator has dispatched and not yet seen
        # reach a terminal state. A cancel is done when this is empty. Handles
        # any composition (one motion, nav+vla, speak, ...) uniformly.
        self._inflight: dict = {}
        self._command_seq = 0            # monotonic command id source
        # Utterances heard since the current sub-task started (voice_keyword).
        self._transcripts: list = []

        # --- io -----------------------------------------------------------
        # One mutually-exclusive group for transcript / verdict / tick so state
        # (_active, _index, latest_verdict, ...) has a single writer at a time.
        # Ticks are cheap (read cache + publish), so serializing costs nothing.
        grp = MutuallyExclusiveCallbackGroup()
        self.status_pub = self.create_publisher(TaskStatus, g('status_topic').value, 10)
        self.active_pub = self.create_publisher(Subtask, g('active_subtask_topic').value, 10)
        self.say_pub = self.create_publisher(SpeakCommand, g('say_topic').value, 10)
        # SpeakConnector.cancel publishes here so a preempt/fail cuts in-flight TTS.
        self.stop_pub = self.create_publisher(Bool, g('stop_topic').value, 10)

        self.create_subscription(
            String, g('transcript_topic').value, self._on_transcript, 10, callback_group=grp)
        self.create_subscription(
            Verdict, g('verdict_topic').value, self._on_verdict, 10, callback_group=grp)

        # CommandStatus sources: latched, reliable stream (see CommandStatus.msg)
        # so a dropped transition / late subscriber is self-healing. Handler
        # (nav/vla) and tts_node (speak) both feed the same registry handler.
        status_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        for topic in (g('handler_status_topic').value, g('tts_status_topic').value):
            self.create_subscription(
                CommandStatus, topic, self._on_command_status, status_qos, callback_group=grp)

        self.create_timer(1.0 / tick_hz, self._tick, callback_group=grp)
        self.get_logger().info(f'orchestrator_node up (tick={tick_hz}Hz)')

    # --- inputs -----------------------------------------------------------
    def _on_transcript(self, msg: String) -> None:
        text = msg.data
        # Buffer BEFORE the trigger check: a voice_keyword criterion reads this,
        # and an early return on a trigger match must not swallow the utterance.
        self._transcripts.append(text)
        for sc in self.scenarios:
            if any(kw in text for kw in sc.triggers):
                self._request(sc)
                return
        # No trigger matched — ignore (not every utterance is a command).

    def _on_verdict(self, msg: Verdict) -> None:
        self.latest_verdict = msg

    def _on_command_status(self, msg: CommandStatus) -> None:
        # Update the registry; drop on terminal. Unknown id (stale echo, or the
        # untracked id=0) is ignored.
        c = self._inflight.get(msg.command_id)
        if c is None:
            return
        c.state = msg.state
        c.last_seen = self._now()
        if msg.state in _TERMINAL:
            del self._inflight[msg.command_id]

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
        if not sc.sub_tasks:
            # A scenario with no sub-tasks is a pure stop: _request already
            # preempted (cancelling the motion), and there is nothing to run.
            # Settle straight back to idle instead of leaving _active dangling.
            self._publish_status(TaskStatus.STATE_SUCCEEDED, detail='stop')
            self._reset_exec()
            return
        self._enter_subtask()

    # --- CANCELING: cancel every in-flight command, wait for all, then continue --
    def _begin_cancel(self, reason: str) -> None:
        n = len(self._inflight)
        for cid, c in list(self._inflight.items()):
            self.connectors[c.connector].cancel(self, cid)
        self._canceling = True
        self._cancel_t0 = self._now()
        self._reset_exec()
        self.get_logger().info(f'canceling {n} command(s) ({reason})')

    def _service_cancel(self) -> None:
        if self._cancel_done():
            self._finish_cancel()
        elif self._now() - self._cancel_t0 >= self._cancel_timeout:
            self._escalate()

    def _cancel_done(self) -> bool:
        # DEV: assume_stopped short-circuits while nav/vla have no status source.
        return self._assume_stopped or not self._inflight

    def _finish_cancel(self) -> None:
        self._canceling = False
        pending, self._pending = self._pending, None
        if pending is not None:
            self._begin(pending)         # buffered scenario runs now that we stopped
        # else: stay idle

    def _escalate(self) -> None:
        # Cancel not confirmed in time: escalate each remaining command
        # (motion → E-STOP, speech → flush) and stop tracking — safety is onboard's.
        self.get_logger().error(
            f'stop not confirmed in {self._cancel_timeout}s — escalating {len(self._inflight)}')
        for cid, c in list(self._inflight.items()):
            self.connectors[c.connector].escalate(self, cid)
        self._inflight.clear()
        self._finish_cancel()

    def _check_liveness(self) -> None:
        # Watchdog for a hung module: a non-terminal command whose status stops
        # updating. The subtask/cancel timeouts assume the module is alive-but-slow.
        if not self._monitor_liveness:
            return
        now = self._now()
        for cid, c in list(self._inflight.items()):
            if c.state not in _TERMINAL and now - c.last_seen > self._stale_sec:
                self.get_logger().error(
                    f'command {cid} ({c.connector}) status stale — fault')
                self.connectors[c.connector].escalate(self, cid)
                del self._inflight[cid]

    def _enter_subtask(self) -> None:
        st = self._current()
        if st is None:
            return
        self.latest_verdict = None
        self._transcripts.clear()        # voice_keyword sees only THIS sub-task's speech
        self._criterion = st.criterion
        self._started = False            # _t0 is set on on_start, not here — see _tick
        self._dispatch(st.on_create)                 # announce
        # tell vlm_node what to judge
        sub = Subtask()
        sub.id = st.name
        sub.success_check = str(st.success.get('check', st.success.get('type', '')))
        sub.timeout_sec = st.timeout_s
        self.active_pub.publish(sub)
        self._publish_status(TaskStatus.STATE_RUNNING, current_subtask=st.name)

    def _tick(self) -> None:
        self._check_liveness()           # a command can hang in any state
        if self._canceling:              # lifecycle frozen until all stop
            self._service_cancel()
            return
        st = self._current()
        if st is None:
            return
        if not self._started:
            # _t0 here (on_start), not on entry, so timeout/delay mean "since motion
            # began". Safe this late: the timeout below only runs once _started.
            self._t0 = self._now()
            self._dispatch(st.on_start)
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
        # Order matters: capture on_fail and publish FAILED while _active still
        # stands (both gone after _begin_cancel → _reset_exec); cancel BEFORE
        # announcing so barge-in doesn't cut the on_fail message.
        st = self._current()
        on_fail = st.on_fail if st else []
        self._publish_status(TaskStatus.STATE_FAILED, detail=reason)
        self._begin_cancel(reason)
        self._dispatch(on_fail)

    # --- helpers ----------------------------------------------------------
    def _dispatch(self, hooks: list) -> None:
        # Each hook step routes to its connector; every tracked command gets its
        # OWN id and registry entry, so nav+vla in one sub-task are tracked apart.
        for step in hooks:
            for label, payload in step.items():
                conn = self.connectors.get(label)
                if conn is None:
                    self.get_logger().warning(f'no connector for label {label!r}')
                    continue
                if conn.is_tracked:
                    cid = self._next_command_id()
                    self._inflight[cid] = InflightCmd(label, CommandStatus.ACCEPTED, self._now())
                    conn.dispatch(self, payload, cid)
                else:
                    conn.dispatch(self, payload, 0)

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

    # --- read-only state for Criterion.evaluate ---------------------------
    def elapsed(self) -> float:
        """Seconds since the current sub-task's on_start dispatch."""
        return self._now() - self._t0

    @property
    def transcripts(self) -> list:
        """Utterances heard since the current sub-task started."""
        return self._transcripts

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
