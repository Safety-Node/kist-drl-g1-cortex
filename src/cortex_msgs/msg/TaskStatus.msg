# Orchestration status published by planner_node as a plan executes.

std_msgs/Header header

string          task_name       # top-level task this plan fulfills
string          current_subtask # id of the subtask in flight ("" if idle)
uint32          subtask_index   # 0-based position in the plan
uint32          subtask_count   # total subtasks in the plan

uint8 STATE_IDLE      = 0
uint8 STATE_RUNNING   = 1
uint8 STATE_SUCCEEDED = 2
uint8 STATE_FAILED    = 3
uint8 STATE_PREEMPTED = 4       # canceled (new utterance / critic fail)
uint8           state

string          detail          # human-readable note / failure reason
