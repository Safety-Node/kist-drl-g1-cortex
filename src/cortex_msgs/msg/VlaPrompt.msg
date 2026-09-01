# vlm_node -> orchestrator_node: grounded VLA prompt for the active sub-task.
#
# The scenario/LLM plan carries only an ABSTRACT goal (Subtask.goal_text, e.g.
# "open the refrigerator door"). The VLM sees the current frame and grounds it
# into the scene-specific imperative the VLA policy was trained on, e.g.
# "Open the right door of the refrigerator. Hook the yellow tip attached to
# your right hand under the door handle and pull."
#
# The orchestrator holds the vla dispatch for the sub-task until this arrives
# — and, when the sub-task declares a precondition, the ENTIRE on_start (this
# message doubles as the precondition verdict). The sub-task timeout keeps a
# missing prompt from hanging the plan in either case.

std_msgs/Header header

string subtask_id    # the Subtask.id this prompt belongs to
string text          # scene-grounded imperative handed to the VLA as-is

# Precondition report, piggybacked on the grounding call (same frame, same
# inference). FAIL-OPEN: when the check is empty, unparseable, or the backend
# errors, precondition_met is true — this gate is an optimization layer, not a
# safety interlock (that is the E-STOP path), so infrastructure failure must
# not block motion. The orchestrator decides what to do with an unmet report:
# shadow mode (default) logs and fires anyway; enforce mode fails the sub-task.
bool   precondition_met
string precondition_why    # human-readable reason when unmet ("" otherwise)
