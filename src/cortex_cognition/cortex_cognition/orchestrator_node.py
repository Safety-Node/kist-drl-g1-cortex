"""orchestrator_node — hook-driven scenario orchestrator (TaskSrv form, rclpy).

No LLM, no router: a scenario declares its own trigger keywords and an STT
transcript is matched against them. A scenario is a list of sub-tasks, each a
small lifecycle machine:

    on_create → on_start → (poll `success` each tick) → on_success | on_fail

A hook step is {label: payload}; the label picks a Connector (speak / navigation
/ vla). `success` is a polymorphic Criterion (vlm reads vlm_node's Verdict off
the tick loop — heavy inference must not block it). Scenarios are JSON5 under
config/scenarios/.

OPEN-LOOP: connectors fire commands (ActionCmd for speak, stubs for nav/vla) and
never wait for confirmation. Preemption is immediate — a new trigger cancels the
current commands (fire-and-forget) and starts the new scenario right away, with no
CANCELING wait or status monitoring. The old and new motions may briefly overlap;
that trade-off is accepted for simplicity.
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
from std_msgs.msg import Bool, String

from cortex_msgs.msg import ActionCmd, Subtask, TaskStatus, Verdict


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
    # Open-loop: dispatch fires a command, cancel fires a stop. Neither waits for
    # confirmation. Both abstract — cancel is safety-relevant, no silent no-op.
    @abstractmethod
    def dispatch(self, node: 'OrchestratorNode', payload) -> None: ...

    @abstractmethod
    def cancel(self, node: 'OrchestratorNode') -> None: ...


class SpeakConnector(Connector):
    """`speak` -> tts_node (ActionCmd)."""

    def dispatch(self, node, payload) -> None:
        node.say_pub.publish(ActionCmd(text=str(payload)))

    def cancel(self, node) -> None:
        # Barge-in (fire-and-forget). Only cuts PC-side synthesis; audio already
        # published to the speaker is not recalled.
        node.stop_pub.publish(Bool(data=True))


class NavigationConnector(Connector):
    """`navigation` -> LocoCommand / named goal -> Gearsonic Handler (stub)."""

    def dispatch(self, node, payload) -> None:
        node.get_logger().info(f'(stub) navigation dispatch: {payload!r}')

    def cancel(self, node) -> None:
        node.get_logger().info('(stub) navigation cancel')


class VlaConnector(Connector):
    """`vla` -> arm/hand joint inference -> Gearsonic Handler (stub)."""

    def dispatch(self, node, payload) -> None:
        node.get_logger().info(f'(stub) vla dispatch: {payload!r}')

    def cancel(self, node) -> None:
        node.get_logger().info('(stub) vla cancel')


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
        self.declare_parameter('tick_rate_hz', 10.0)

        g = self.get_parameter
        scenario_dir = g('scenario_dir').value
        tick_hz = float(g('tick_rate_hz').value)

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
        # Utterances heard since the current sub-task started (voice_keyword).
        self._transcripts: list = []

        # --- io -----------------------------------------------------------
        # One mutually-exclusive group for transcript / verdict / tick so state
        # (_active, _index, latest_verdict, ...) has a single writer at a time.
        grp = MutuallyExclusiveCallbackGroup()
        self.status_pub = self.create_publisher(TaskStatus, g('status_topic').value, 10)
        self.active_pub = self.create_publisher(Subtask, g('active_subtask_topic').value, 10)
        self.say_pub = self.create_publisher(ActionCmd, g('say_topic').value, 10)
        self.stop_pub = self.create_publisher(Bool, g('stop_topic').value, 10)   # tts barge-in

        self.create_subscription(
            String, g('transcript_topic').value, self._on_transcript, 10, callback_group=grp)
        self.create_subscription(
            Verdict, g('verdict_topic').value, self._on_verdict, 10, callback_group=grp)

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

    # --- scenario lifecycle -----------------------------------------------
    def _request(self, sc: Scenario) -> None:
        """A trigger matched. Preempt the running scenario (fire-and-forget cancel)
        and start the new one immediately — open-loop, no wait."""
        if self._active is not None:
            self._publish_status(TaskStatus.STATE_PREEMPTED, detail='preempted by new trigger')
            self._stop_current()
        self._begin(sc)

    def _begin(self, sc: Scenario) -> None:
        self.get_logger().info(f'begin scenario {sc.name!r} ({len(sc.sub_tasks)} sub-tasks)')
        self._active = sc
        self._index = 0
        if not sc.sub_tasks:
            # A pure "stop" scenario: _request already preempted, nothing to run.
            self._publish_status(TaskStatus.STATE_SUCCEEDED, detail='stop')
            self._reset_exec()
            return
        self._enter_subtask()

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
        # Capture on_fail and publish FAILED while _active still stands (gone after
        # _stop_current → _reset_exec); cancel BEFORE announcing so barge-in
        # doesn't cut the on_fail message.
        st = self._current()
        on_fail = st.on_fail if st else []
        self._publish_status(TaskStatus.STATE_FAILED, detail=reason)
        self._stop_current()
        self._dispatch(on_fail)

    def _stop_current(self) -> None:
        """Fire-and-forget cancel of the current commands + clear exec state.
        The caller publishes the status (PREEMPTED / FAILED) first."""
        for conn in self.connectors.values():
            conn.cancel(self)
        self._reset_exec()

    # --- helpers ----------------------------------------------------------
    def _dispatch(self, hooks: list) -> None:
        # Each hook step routes to its connector (fire-and-forget, open-loop).
        for step in hooks:
            for label, payload in step.items():
                conn = self.connectors.get(label)
                if conn is None:
                    self.get_logger().warning(f'no connector for label {label!r}')
                    continue
                conn.dispatch(self, payload)

    def _current(self):
        if self._active and 0 <= self._index < len(self._active.sub_tasks):
            return self._active.sub_tasks[self._index]
        return None

    def _reset_exec(self) -> None:
        """Clear sub-task execution state (back to idle)."""
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
