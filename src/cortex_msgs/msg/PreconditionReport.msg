# vlm_node -> orchestrator_node: precondition verdict for a sub-task WITHOUT a
# grounded vla step (nav etc.). Grounded sub-tasks get their verdict on the
# VlaPrompt instead — one VLM call answers grounding + precondition there.
#
# The orchestrator defers the ENTIRE on_start until this arrives, bounded by
# precondition_timeout_s: a missing report FAILS OPEN (motion proceeds with a
# warning) because this gate is an optimization, not a safety interlock — a
# dead vlm_node must not strand navigation that needs nothing from the VLM.

std_msgs/Header header

string subtask_id    # the Subtask.id this report answers
bool   met           # does the scene satisfy Subtask.precondition_check?
string why           # human-readable reason when unmet ("" otherwise)
