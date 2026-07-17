# Pass/fail judgment emitted by vlm_node for the in-flight subtask.
# planner_node advances the plan on `passed`, or cancels the active goal on fail.

std_msgs/Header header

string          subtask_id      # the Subtask.id this verdict refers to
bool            passed          # was Subtask.success_check satisfied?
float32         confidence      # 0.0 .. 1.0
string          reason          # human-readable note, esp. on failure
