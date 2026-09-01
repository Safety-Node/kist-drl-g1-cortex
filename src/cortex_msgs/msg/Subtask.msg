# What orchestrator_node hands to vlm_node: which sub-task is in flight and what
# "success" looks like for it. The full sub-task (hooks, connectors) lives in the
# JSON5 scenario + orchestrator; this message is only the projection the critic
# needs in order to judge the scene — and, when goal_text is set, to ground the
# VLA prompt for it (VlaPrompt).

string   id              # sub-task id; vlm_node tags its Verdict with this so the
                         # orchestrator can match a verdict to the active sub-task
string   success_check   # what the VLM should look for, e.g. "cucumber in gripper"
float32  timeout_sec     # 0 = no timeout (informational; orchestrator owns the clock)

string   goal_text       # abstract VLA goal for the VLM to ground ("" = none —
                         # vlm_node then only judges, it does not emit a VlaPrompt)
float32  progress_gate   # judge only once VLA task_progress >= this (0 = ungated).
                         # Keeps the critic from failing a manipulation that is
                         # still mid-motion; the VLM fires near completion.

string   precondition_check  # scene condition that must hold BEFORE the grounded
                             # vla step fires ("" = no check). Judged by the VLM
                             # in the SAME call that grounds goal_text — no extra
                             # inference. Only meaningful with goal_text set
                             # (enforced by the scenario loader).
