# orchestrator_node -> vlm_node: judge the scene NOW, once.
#
# Sent when every motion of the in-flight sub-task has reported done
# (CommandStatus) — or immediately after on_start when the sub-task has no
# motion steps. vlm_node is otherwise idle as a critic: one request, one
# Verdict, then idle again. There is no periodic judging loop and no
# progress gate — the completion signal owns "when", the VLM owns "whether".

std_msgs/Header header

string subtask_id     # vlm_node echoes this on the Verdict
string success_check  # what to look for, e.g. "refrigerator door is open"
