# What orchestrator_node hands to vlm_node: which sub-task is in flight and what
# "success" looks like for it. The full sub-task (hooks, connectors) lives in the
# JSON5 scenario + orchestrator; this message is only the projection the critic
# needs in order to judge the scene.

string   id              # sub-task id; vlm_node tags its Verdict with this so the
                         # orchestrator can match a verdict to the active sub-task
string   success_check   # what the VLM should look for, e.g. "cucumber in gripper"
float32  timeout_sec     # 0 = no timeout (informational; orchestrator owns the clock)
