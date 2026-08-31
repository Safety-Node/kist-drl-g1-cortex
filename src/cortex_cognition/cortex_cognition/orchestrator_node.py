"""orchestrator_node — hook-driven scenario orchestrator (TaskSrv form, rclpy).

Two planner modes, switched by the `planner_mode` parameter BEFORE launch
(cortex_params.yaml) — the execution engine below the plan is identical:

    static  scenarios are JSON5 files under config/scenarios/; each declares
            its own trigger keywords and a transcript is matched against them.
    llm     every final transcript is sent to llm_node (PlanRequest); the
            returned Plan carries a scenario in the SAME schema (as JSON), so
            it is validated by the same loader and run by the same engine.

A scenario is a list of sub-tasks, each a small lifecycle machine:

    on_create → on_start → (poll `success` each tick) → on_success | on_fail

A hook step is {label: payload}; the label picks a Connector (speak / navigation
/ vla). A vla payload of {grounded: true, goal: "..."} is not sent verbatim:
the goal goes to vlm_node (via the active Subtask), which grounds it against
the live scene and returns a VlaPrompt; only then is the vla connector fired.
`success` is a polymorphic Criterion (vlm reads vlm_node's Verdict off the
tick loop — heavy inference must not block it).

OPEN-LOOP: connectors fire commands (ActionCmd for speak, stubs for nav/vla) and
never wait for confirmation. Preemption is immediate — a new trigger cancels the
current commands (fire-and-forget) and starts the new scenario right away, with no
CANCELING wait or status monitoring. The old and new motions may briefly overlap;
that trade-off is accepted for simplicity.
"""

import glob
import os
import time
import uuid
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import json5
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from cortex_msgs.msg import (
    ActionCmd, Plan, PlanRequest, Subtask, TaskStatus, Verdict, VlaPrompt,
)


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
    goal_text: str = ''                             # abstract goal of a grounded vla step
    progress_gate: float = 0.0                      # vlm judges only past this task_progress


def _grounded_goal(s: dict) -> str:
    """Extract THE grounded-vla goal of a sub-task ('' if none).

    A grounded vla payload is {grounded: true, goal: "..."}. At most one per
    sub-task: one Subtask carries one goal_text and one VlaPrompt answers it —
    two grounded steps would race on the same reply. Fail at load, not mid-run.
    """
    goals = []
    for hook in ('on_create', 'on_start', 'on_success', 'on_fail'):
        for step in s.get(hook, []):
            if not isinstance(step, dict):
                # A hook step must be {label: payload}. Guarded here (load
                # path) because an LLM plan can hallucinate shapes a hand-
                # written file never would — and this must reject, not crash.
                raise ScenarioConfigError(
                    f"{s.get('name')}.{hook}: step must be {{label: payload}}, "
                    f'got {step!r}')
            payload = step.get('vla')
            if isinstance(payload, dict):
                if not payload.get('grounded') or not isinstance(
                        payload.get('goal'), str) or not payload['goal']:
                    raise ScenarioConfigError(
                        f"{s.get('name')}: dict vla payload must be "
                        f"{{grounded: true, goal: \"...\"}}, got {payload!r}")
                if hook != 'on_start':
                    raise ScenarioConfigError(
                        f"{s.get('name')}: grounded vla belongs in on_start "
                        f'(found in {hook}); other hooks fire without a prompt')
                goals.append(payload['goal'])
    if len(goals) > 1:
        raise ScenarioConfigError(
            f"{s.get('name')}: at most one grounded vla step per sub-task")
    return goals[0] if goals else ''


@dataclass
class Scenario:
    name: str
    triggers: list           # keyword substrings matched against transcripts
    sub_tasks: list          # [SubTaskDef]. Empty == a pure "stop": preempt, then idle.


def _load_subtask(s: dict) -> SubTaskDef:
    spec = s.get('success', {})
    gate = float(spec.get('progress_gate', 0.0))
    if not 0.0 <= gate < 1.0:
        raise ScenarioConfigError(
            f"{s.get('name')}: progress_gate must be in [0, 1), got {gate}")
    return SubTaskDef(
        name=s['name'],
        on_create=s.get('on_create', []),
        on_start=s.get('on_start', []),
        on_success=s.get('on_success', []),
        on_fail=s.get('on_fail', []),
        success=spec,
        criterion=build_criterion(spec),            # raises ScenarioConfigError here
        timeout_s=float(spec.get('timeout_s', 30.0)),
        goal_text=_grounded_goal(s),
        progress_gate=gate,
    )


def scenario_from_raw(raw: dict) -> Scenario:
    """One raw scenario dict -> Scenario. Shared by BOTH planner modes: files
    (load_scenarios) and LLM plans (_on_plan) go through the same validation,
    so an LLM hallucination fails exactly like a typo in a .json5 would."""
    subs = [_load_subtask(s) for s in raw.get('sub_tasks', [])]
    return Scenario(raw['name'], raw.get('triggers', []), subs)


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
            scenarios.append(scenario_from_raw(raw))
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
        # --- planner mode: static (JSON5 files) | llm (llm_node plans) -----
        # Set in cortex_params.yaml before launch; not a runtime toggle.
        self.declare_parameter('planner_mode', 'static')
        self.declare_parameter('llm_request_topic', '/cortex/llm/request')
        self.declare_parameter('llm_plan_topic', '/cortex/llm/plan')
        self.declare_parameter('llm_timeout_s', 20.0)  # utterance -> plan budget
        self.declare_parameter('vla_prompt_topic', '/cortex/vla/prompt')

        g = self.get_parameter
        scenario_dir = g('scenario_dir').value
        tick_hz = float(g('tick_rate_hz').value)
        self._mode = g('planner_mode').value
        if self._mode not in ('static', 'llm'):
            raise ValueError(f"planner_mode must be 'static' or 'llm', got {self._mode!r}")
        self._llm_timeout_s = float(g('llm_timeout_s').value)

        # --- connectors (label -> connector) ------------------------------
        self.connectors = {
            'speak': SpeakConnector(),
            'navigation': NavigationConnector(),
            'vla': VlaConnector(),
        }

        # --- scenarios ----------------------------------------------------
        # Files load in BOTH modes: llm mode does not use their triggers, but a
        # load failure means broken schema assumptions and should stop startup
        # regardless of which planner is active.
        self.scenarios = load_scenarios(scenario_dir)
        self.get_logger().info(
            f'loaded {len(self.scenarios)} scenario(s) from {scenario_dir} '
            f'(planner_mode={self._mode})')

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
        # llm mode: the one plan request in flight (None = not waiting).
        # request_id gates stale plans; deadline bounds a dead llm_node.
        self._pending_request_id = None
        self._pending_deadline = 0.0
        # Grounded-vla handshake for the CURRENT sub-task: prompt text arrives
        # from vlm_node (VlaPrompt) and is dispatched after on_start.
        self._awaiting_prompt = False
        self._prompt_text = None

        # --- io -----------------------------------------------------------
        # One mutually-exclusive group for transcript / verdict / tick so state
        # (_active, _index, latest_verdict, ...) has a single writer at a time.
        grp = MutuallyExclusiveCallbackGroup()
        self.status_pub = self.create_publisher(TaskStatus, g('status_topic').value, 10)
        self.active_pub = self.create_publisher(Subtask, g('active_subtask_topic').value, 10)
        self.say_pub = self.create_publisher(ActionCmd, g('say_topic').value, 10)
        self.stop_pub = self.create_publisher(Bool, g('stop_topic').value, 10)   # tts barge-in

        self.plan_req_pub = self.create_publisher(
            PlanRequest, g('llm_request_topic').value, 10)

        self.create_subscription(
            String, g('transcript_topic').value, self._on_transcript, 10, callback_group=grp)
        self.create_subscription(
            Verdict, g('verdict_topic').value, self._on_verdict, 10, callback_group=grp)
        self.create_subscription(
            Plan, g('llm_plan_topic').value, self._on_plan, 10, callback_group=grp)
        self.create_subscription(
            VlaPrompt, g('vla_prompt_topic').value, self._on_vla_prompt, 10,
            callback_group=grp)

        self.create_timer(1.0 / tick_hz, self._tick, callback_group=grp)
        self.get_logger().info(f'orchestrator_node up (tick={tick_hz}Hz)')

    # --- inputs -----------------------------------------------------------
    def _on_transcript(self, msg: String) -> None:
        text = msg.data
        # Buffer BEFORE the trigger/plan path: a voice_keyword criterion reads
        # this, and an early return must not swallow the utterance.
        self._transcripts.append(text)
        if self._mode == 'llm':
            self._request_plan(text)
            return
        for sc in self.scenarios:
            if any(kw in text for kw in sc.triggers):
                self._request(sc)
                return
        # No trigger matched — ignore (not every utterance is a command).

    def _on_verdict(self, msg: Verdict) -> None:
        self.latest_verdict = msg

    def _on_vla_prompt(self, msg: VlaPrompt) -> None:
        st = self._current()
        if st is None or st.name != msg.subtask_id or not self._awaiting_prompt:
            return  # stale prompt (sub-task advanced/preempted) — must not fire the arm
        # Cache; _tick dispatches it AFTER on_start so hook order holds even
        # when grounding finishes before the first tick.
        self._prompt_text = msg.text

    # --- llm planner ------------------------------------------------------
    def _request_plan(self, text: str) -> None:
        """Send the utterance to llm_node. Latest-wins: a newer utterance
        replaces the pending request; the old plan is dropped by request_id.
        The RUNNING scenario is preempted only when a valid plan lands —
        planning failures must not kill a demo in progress."""
        req = PlanRequest()
        req.header.stamp = self.get_clock().now().to_msg()
        req.request_id = uuid.uuid4().hex
        req.text = text
        self._pending_request_id = req.request_id
        self._pending_deadline = self._now() + self._llm_timeout_s
        self.plan_req_pub.publish(req)
        self.get_logger().info(f'plan requested {req.request_id}: {text!r}')

    def _on_plan(self, msg: Plan) -> None:
        if msg.request_id != self._pending_request_id:
            return  # superseded request — ignore
        self._pending_request_id = None
        if not msg.ok:
            # 'not_a_command' is the LLM's escape hatch for bystander speech /
            # voice_keyword replies — silent by design. Real failures get a
            # spoken cue, but only when idle (never talk over a running task).
            if msg.error != 'not_a_command':
                self.get_logger().warning(f'plan failed: {msg.error}')
                if self._active is None:
                    self.say_pub.publish(ActionCmd(text='명령을 계획하지 못했습니다.'))
            return
        try:
            raw = json5.loads(msg.scenario_json)
            sc = scenario_from_raw(raw)   # same validator as the file path
        except (ScenarioConfigError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f'plan rejected: {exc}')
            if self._active is None:
                self.say_pub.publish(ActionCmd(text='계획을 이해하지 못했습니다.'))
            return
        self._request(sc)   # preempt-and-begin, same as a static trigger

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
        self._awaiting_prompt = bool(st.goal_text)
        self._prompt_text = None
        self._dispatch(st.on_create)                 # announce
        # Tell vlm_node what to judge — and, via goal_text, what to ground.
        # Publishing st.goal_text is what kicks off the VlaPrompt round-trip.
        sub = Subtask()
        sub.id = st.name
        sub.success_check = str(st.success.get('check', st.success.get('type', '')))
        sub.timeout_sec = st.timeout_s
        sub.goal_text = st.goal_text
        sub.progress_gate = st.progress_gate
        self.active_pub.publish(sub)
        self._publish_status(TaskStatus.STATE_RUNNING, current_subtask=st.name)

    def _tick(self) -> None:
        # llm mode: bound the utterance -> plan wait even while idle (this is
        # the only per-tick work that exists outside a running scenario).
        if (self._pending_request_id is not None
                and self._now() >= self._pending_deadline):
            self.get_logger().warning(
                f'plan request {self._pending_request_id} timed out '
                f'({self._llm_timeout_s}s) — is llm_node up?')
            self._pending_request_id = None

        st = self._current()
        if st is None:
            return
        if not self._started:
            # _t0 here (on_start), not on entry, so timeout/delay mean "since motion
            # began". Safe this late: the timeout below only runs once _started.
            # NOTE for grounded vla: the sub-task timeout therefore covers
            # grounding latency + motion — budget timeout_s accordingly.
            self._t0 = self._now()
            self._dispatch(st.on_start)
            self._started = True
            return
        if self._awaiting_prompt and self._prompt_text is not None:
            # Grounded prompt landed: fire the deferred vla step. After
            # on_start (hook order) and before the criterion check (a verdict
            # cannot pass a motion that never started).
            self._awaiting_prompt = False
            self.connectors['vla'].dispatch(self, self._prompt_text)
            self._prompt_text = None
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
                if label == 'vla' and isinstance(payload, dict):
                    # Grounded step — deferred until vlm_node's VlaPrompt
                    # arrives (dispatched in _tick), never sent verbatim.
                    continue
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
        self._awaiting_prompt = False
        self._prompt_text = None

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
