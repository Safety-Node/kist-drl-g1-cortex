# kist-drl-g1-cortex

**KIST DRL — Unitree G1 Cognitive / Reasoning Stack (PC)**

Event-driven ROS 2 rclpy workspace for the PC running the KIST G1 collaborative demo (2026).
Speech I/O, scenario orchestration, VLM success judgment.

Replaces the OM1 fork (`kist-drl-g1-workstation`): no tick-blocking LLM cortex, and one
DDS graph shared with the NX instead of a hand-rolled relay.

> Target HW: PC (Ubuntu 22.04, ROS 2 Humble, Python 3.10)
> Companion repo: `kist-drl-g1-onboard` (NX side — sensors / safety / motors)

> 🚧 **Scaffold.** Backends (STT / TTS / VLM) and the motion connectors are placeholders.
> Markers `TODO(REQ-XX) [TASK-XX]` link back to the Notion DBs.

---

## Packages

| # | Package | Build | Notes |
|---|---|---|---|
| 1 | `g1_onboard_msgs` | ament_cmake | Shared interfaces submodule (SSOT) — **do not fork** |
| 2 | `cortex_msgs` | ament_cmake | Cortex-internal: `Subtask` / `TaskStatus` / `Verdict`. `CommandStatus` is a prototype destined for SSOT (see `docs/proposals/`) |
| 3 | `cortex_speech` | ament_python | `stt_node` (VAD → transcript), `tts_node` (text → speaker, barge-in) |
| 4 | `cortex_reasoning` | ament_python | `orchestrator_node` (JSON5 scenarios, hook lifecycle, preemption), `vlm_node` (scene critic) |
| 5 | `cortex_bringup` | ament_python | Top-level launch + params |

---

## Flow

```
발화 ─STT─▶ orchestrator: triggers[] 매칭 ─▶ 시나리오
                │  sub-task lifecycle: on_create → on_start → success poll → on_success
                │  dispatch by label ─┬─ speak      → tts_node
                │                     ├─ navigation → Nav Planner  ─┐
                │                     └─ vla        → VLA inference ─┴─▶ Gearsonic Handler → G1
                ◀── Verdict ── vlm_node (씬 판정)
                ◀── CommandStatus ── Handler (정지 확인)
```

- **No LLM router**: a scenario declares its own `triggers[]`; a transcript is keyword-matched.
- **Success = VLM**: `vlm_node` judges the scene and publishes a `Verdict`; the orchestrator's
  `VlmCriterion` reads the cache (VLM inference never runs on the tick loop).
- **Preemption is a handshake**: a new trigger cancels, buffers the new scenario, and waits for
  the module to confirm it stopped (`CommandStatus`) before starting — old and new motions never overlap.

Scenarios are JSON5 under `src/cortex_reasoning/config/scenarios/` — see
`refrigerator_pickup.json5` for the hook/criteria schema.

---

## Install

PC requirements:
- Ubuntu 22.04 (must match NX onboard distro)
- ROS 2 Humble (must match NX onboard distro)
- Python 3.10 (Humble system Python — rclpy ABI is tied to the system Python minor version)

```bash
# System deps
sudo apt-get update && sudo apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions python3-rosdep

# Shared interfaces submodule
git submodule update --init --recursive

# Repo-local activation (ROS + CycloneDDS + bridge domain)
source env.sh

# Deps
rosdep install --from-paths src --ignore-src -r -y
pip install -r requirements.txt          # non-ROS deps (json5, STT/TTS/LLM clients)

colcon build --symlink-install
```

---

## Run

```bash
./scripts/run_cortex.sh                       # env.sh + full node graph
ros2 launch cortex_bringup cortex.launch.py   # same, if env.sh already sourced
ros2 launch cortex_speech speech.launch.py    # STT + TTS only
```

Parameters (topics, tick rate, cancel timeout) live in
`src/cortex_bringup/config/cortex_params.yaml`.

> `assume_stopped: true` is the dev default — no Handler publishes `CommandStatus` yet, so
> cancels are assumed to succeed. Set `false` once the Handler is live.

---

## DDS / domain

`env.sh` sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=1` (the **bridge**
domain shared with onboard) and points `DDS_PEER_IP` at the NX. `config/cyclonedds.xml`
mirrors the onboard bridge-domain config (unicast peers, `AllowMulticast=false`).

⚠️ Onboard isolates `/onboard/*` from `/bridge/*` via **separate DDS domains**, previously
bridged by `comm_bridge`. With `comm_bridge` deleted, onboard producers must publish directly
on the bridge domain. Keep both `cyclonedds.xml` files and `ROS_DOMAIN_ID` in sync.

---

## Where the spec lives

| Layer | Notion |
|---|---|
| Requirements | [SYS-REQ DB](https://www.notion.so/d7d7c9b9943b4018a4bce2afb904d706) |
| Interface contracts | [ICD DB](https://www.notion.so/b319b5cec8f2429389fb5fac8c042503) |
| Work items | [Tasks DB](https://www.notion.so/cd779d7a54b343b6a9e5449f4620a44c) |
| Verification | [Tests DB](https://www.notion.so/a67e62ef1cfc4f85be29a340107846b6) |

Each `TODO(REQ-XX) [TASK-XX]` in code links to the matching Notion page.

---

## Blocked on external specs

| Waiting on | Blocks |
|---|---|
| Gearsonic Handler interface | `navigation` / `vla` connector dispatch + cancel (stubs) |
| Handler `CommandStatus` publishing | real stop-confirmation (`_stopped`, `assume_stopped=false`) |
| Onboard `/bridge/*` audio topic names | `stt_node` / `tts_node` wiring |
| VLM backend | `vlm_node._evaluate` |

---

## Contributing

PRs are squash-merged to `main`. Conventions enforced in CI:

- Branch name: `TASK-{number}` (Notion-linked work) or `chore/{description}` (non-task housekeeping)
- PR title: `[TASK-{number}] <type>(<scope>)?: <subject>` or `[chore] <type>(<scope>)?: <subject>`
  (Conventional Commits, lowercase casing)

---

## License

Apache-2.0
