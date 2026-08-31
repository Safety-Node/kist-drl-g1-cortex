# vlm_node -> orchestrator_node: grounded VLA prompt for the active sub-task.
#
# The scenario/LLM plan carries only an ABSTRACT goal (Subtask.goal_text, e.g.
# "open the refrigerator door"). The VLM sees the current frame and grounds it
# into the scene-specific imperative the VLA policy was trained on, e.g.
# "Open the right door of the refrigerator. Hook the yellow tip attached to
# your right hand under the door handle and pull."
#
# The orchestrator holds the vla dispatch for the sub-task until this arrives
# (the sub-task timeout keeps a missing prompt from hanging the plan).

std_msgs/Header header

string subtask_id    # the Subtask.id this prompt belongs to
string text          # scene-grounded imperative handed to the VLA as-is
