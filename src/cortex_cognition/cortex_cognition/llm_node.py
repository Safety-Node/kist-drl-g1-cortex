"""llm_node — utterance -> scenario planner (LLM-backed).

    PlanRequest (/cortex/llm/request) -> queue -> backend worker
                                                      |
    Plan        (/cortex/llm/plan)    <---------------+

Turns one final STT transcript into a scenario in the SAME JSON schema the
file scenarios use (config/scenarios/*.json5, minus `triggers`). The
orchestrator validates the plan with its file-loading code path, so a
hallucinated hook label or criterion type fails there fast — this node does
not own the schema, it only asks the LLM to target it.

Latest-wins: a new request supersedes a queued one (same policy as the VLA
runner's chunk queue). The orchestrator correlates by request_id, so a stale
plan that still slips out is simply ignored on arrival.

Backends: `openai` (TODO — no credentials in CI) and `dummy` (keyword-matched
canned plans; lets the whole llm-mode loop run end-to-end with no network,
mirroring stt_node's dummy backend).
"""

import json
import queue
import threading
from enum import Enum

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from cortex_msgs.msg import Plan, PlanRequest


class LLMBackend(str, Enum):
    OPENAI = 'openai'
    DUMMY = 'dummy'


# ---------------------------------------------------------------------------
# System prompt — the contract the LLM must target.
#
# Kept next to the code (not a config file) deliberately: the schema below must
# track the orchestrator's loader and the CI scenario-sanity checker, and a
# drifting external prompt file is exactly the kind of silent skew this stack
# avoids. If the loader grows a criterion/label, update this in the same PR.
# ---------------------------------------------------------------------------
SYSTEM_PROMPT = """\
You are the task planner of a Unitree G1 home-assistant robot. Given one user
command (Korean), output ONE scenario as pure JSON (no markdown fence, no
commentary) in exactly this schema:

{
  "name": "<snake_case_scenario_name>",
  "sub_tasks": [
    {
      "name": "<snake_case_step_name>",
      "precondition": "<English scene condition that must hold BEFORE this
                       step's motion starts, e.g. 'the refrigerator door is
                       open' or, for navigation, 'the path ahead is clear'.
                       OPTIONAL, legal on any step. Use it when the step
                       depends on a previous step's outcome or on a scene
                       premise worth checking before moving.>",
      "on_create":  [ {"speak": "<Korean announcement>"} ],
      "on_start":   [ <one or more action steps, see labels> ],
      "success":    { <criterion, see below> },
      "on_success": [ {"speak": "<Korean, optional>"} ],
      "on_fail":    [ {"speak": "<Korean failure message, REQUIRED unless
                       success.type is 'always'>"} ]
    }
  ]
}

Action step labels (each step is {"<label>": <payload>}; a list expresses
combos — e.g. speak + vla is two steps in on_start):
  speak       payload: Korean text to say (TTS).
  navigation  payload: "goal:<named_goal>" — known goals: refrigerator,
              counter, table, home.
  vla         payload: {"grounded": true, "goal": "<abstract English goal>"}
              for manipulation. The goal is abstract ("open the refrigerator
              door") — a VLM grounds it against the live camera scene before
              the arm policy runs, so do NOT write scene details you cannot
              see. A plain string payload is also legal and is sent verbatim
              (only for fixed, scene-independent motions).

success criterion types:
  {"type": "vlm", "check": "<English scene condition>", "timeout_s": <sec>,
   "progress_gate": 0.9}
              Scene judgment by VLM. For vla sub-tasks ALWAYS include
              progress_gate (default 0.9): judging fires only once the arm
              policy reports task_progress >= gate, so a mid-motion scene is
              not judged as failure.
  {"type": "delay", "seconds": <sec>, "timeout_s": <sec>}
              Blind wait. Placeholder only — asserts nothing about the world.
  {"type": "voice_keyword", "keywords": ["<Korean>", ...], "timeout_s": <sec>}
              Pass when the person says one of these AFTER this step started.
  {"type": "composite", "timeout_s": <sec>, "children": [<criteria>]}
              AND of children. Use only when children catch different failure
              modes (e.g. gripper closed AND object visible).
  {"type": "always"}
              Immediate pass. Only for pure announcements.

Rules:
- Speak Korean to the user; write vla goals and vlm checks in English.
- Navigation before manipulation when the target is elsewhere.
- Every sub-task that can fail MUST own a Korean on_fail message.
- 3-6 sub_tasks; each one motion primitive. Do not invent unknown goals/labels.
"""


# Canned plans for the dummy backend, keyword-matched against the utterance.
# Enough to exercise: llm-mode trigger, grounded vla dispatch, progress gate,
# nav+speak combo — the full new-message surface, with zero network.
_DUMMY_PLANS = {
    '오이': {
        'name': 'dummy_cucumber',
        'sub_tasks': [
            {
                'name': 'approach_fridge',
                'precondition': 'the path ahead of the robot is clear',
                'on_create': [{'speak': '냉장고 앞으로 갑니다.'}],
                'on_start': [{'navigation': 'goal:refrigerator'}],
                'success': {'type': 'vlm',
                            'check': 'robot is facing the refrigerator door',
                            'timeout_s': 25},
                'on_fail': [{'speak': '냉장고로 이동하지 못했습니다.'}],
            },
            {
                'name': 'open_door',
                'on_create': [{'speak': '냉장고 문을 엽니다.'}],
                'on_start': [{'vla': {'grounded': True,
                                      'goal': 'open the refrigerator door'}}],
                'success': {'type': 'vlm', 'check': 'refrigerator door is open',
                            'timeout_s': 20, 'progress_gate': 0.9},
                'on_fail': [{'speak': '냉장고 문을 열지 못했습니다.'}],
            },
            {
                'name': 'grasp_cucumber',
                'precondition': 'the refrigerator door is open and a cucumber '
                                'is visible',
                'on_create': [{'speak': '오이를 집겠습니다.'}],
                'on_start': [{'vla': {'grounded': True,
                                      'goal': 'pick up the cucumber'}}],
                'success': {'type': 'vlm',
                            'check': 'a cucumber is held in the gripper',
                            'timeout_s': 20, 'progress_gate': 0.9},
                'on_success': [{'speak': '오이를 집었습니다.'}],
                'on_fail': [{'speak': '오이를 집지 못했습니다.'}],
            },
        ],
    },
    '인사': {
        'name': 'dummy_greet',
        'sub_tasks': [
            {
                'name': 'greet',
                'on_create': [{'speak': '안녕하세요, 지원입니다.'}],
                'success': {'type': 'always'},
            },
        ],
    },
}


class LlmNode(Node):
    def __init__(self) -> None:
        super().__init__('llm_node')

        self.declare_parameter('request_topic', '/cortex/llm/request')
        self.declare_parameter('plan_topic', '/cortex/llm/plan')
        self.declare_parameter('backend', LLMBackend.DUMMY.value)
        self.declare_parameter('model', 'gpt-4o')          # openai backend only
        self.declare_parameter('request_timeout_s', 20.0)  # backend call budget

        g = self.get_parameter
        self._backend = LLMBackend(g('backend').value)
        self._model = g('model').value
        self._timeout_s = float(g('request_timeout_s').value)

        # Latest-wins queue: planning for a superseded utterance wastes seconds
        # of LLM latency, so a new request evicts an unstarted old one.
        self._queue: queue.Queue = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)

        grp = ReentrantCallbackGroup()
        self._plan_pub = self.create_publisher(Plan, g('plan_topic').value, 10)
        self.create_subscription(
            PlanRequest, g('request_topic').value, self._on_request, 10,
            callback_group=grp)

        self._worker.start()
        self.get_logger().info(
            f'llm_node up (backend={self._backend.value}, '
            f"{g('request_topic').value} -> {g('plan_topic').value})")

    def destroy_node(self) -> None:
        self._stop.set()
        self._worker.join(timeout=2.0)
        super().destroy_node()

    # --- input ------------------------------------------------------------
    def _on_request(self, msg: PlanRequest) -> None:
        self.get_logger().info(f'plan request {msg.request_id}: {msg.text!r}')
        # Evict an unstarted older request; an in-flight one finishes and its
        # stale plan is dropped by the orchestrator's request_id check.
        try:
            self._queue.get_nowait()
        except queue.Empty:
            pass
        self._queue.put(msg)

    # --- worker -----------------------------------------------------------
    def _worker_loop(self) -> None:
        while not self._stop.is_set():
            try:
                req = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue

            plan = Plan()
            plan.header.stamp = self.get_clock().now().to_msg()
            plan.request_id = req.request_id
            try:
                scenario = self._generate(req.text)
                # Sanity here is only "is it JSON" — semantic validation
                # (labels, criteria) is the orchestrator loader's job.
                plan.scenario_json = json.dumps(scenario, ensure_ascii=False)
                plan.ok = True
                plan.error = ''
            except Exception as exc:  # noqa: BLE001 — any backend failure -> ok=False
                plan.ok = False
                plan.error = str(exc)
                plan.scenario_json = ''
                self.get_logger().error(f'plan {req.request_id} failed: {exc}')
            self._plan_pub.publish(plan)

    # --- backends ---------------------------------------------------------
    def _generate(self, text: str) -> dict:
        if self._backend == LLMBackend.DUMMY:
            return self._generate_dummy(text)
        if self._backend == LLMBackend.OPENAI:
            return self._generate_openai(text)
        raise ValueError(f'unknown backend {self._backend!r}')

    def _generate_dummy(self, text: str) -> dict:
        for keyword, plan in _DUMMY_PLANS.items():
            if keyword in text:
                return plan
        raise ValueError(
            f'dummy backend has no canned plan for {text!r} '
            f'(known keywords: {sorted(_DUMMY_PLANS)})')

    def _generate_openai(self, text: str) -> dict:
        # TODO(REQ-XX) [TASK-XX]: call the chat API with SYSTEM_PROMPT + text,
        # temperature 0, response_format json_object, self._timeout_s budget;
        # json.loads the content and return it. Keep the import inside this
        # method so the dummy backend needs no openai package.
        raise NotImplementedError(
            'openai backend not wired yet — use backend:=dummy')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LlmNode()
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
