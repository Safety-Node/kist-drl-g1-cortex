# nav / vla module -> orchestrator_node: the dispatched motion finished (or
# died). This is the completion signal that triggers on-demand scene judgment:
# the orchestrator collects one report per motion label it dispatched for the
# in-flight sub-task, and only when ALL report ok does it ask vlm_node to
# judge (VerdictRequest). ok=false skips judgment and fails the sub-task
# immediately with `detail` as the reason.
#
# ⚠️ External publishers (gearsonic navigation / kist-vla-inference) are NOT
# wired yet — this message defines the contract they will publish against.
# Until then, motion sub-tasks end by timeout (the safe direction).
#
# NOTE: candidate for the g1_onboard_msgs SSOT once the CommandStatus proposal
# lands (see the Notion page, 전달 사항 / Cancel Sync).

std_msgs/Header header

string source    # which connector's motion this reports: "navigation" | "vla"
bool   ok        # true = motion completed; false = module failed / aborted
string detail    # human-readable note, esp. on failure
