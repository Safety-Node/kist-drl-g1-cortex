# CommandStatus.msg — DRAFT proposal for g1_onboard_msgs (SSOT).
#
# NOT built from here. This is a design draft: land it via a PR to
# kist-drl-g1-msgs and add it to that package's rosidl_generate_interfaces()
# list in CMakeLists.txt.
#
# A buildable PROTOTYPE copy currently lives at cortex_msgs/msg/CommandStatus.msg
# so cortex can exercise the wiring now; keep the two in sync until this lands in
# SSOT, then delete the cortex_msgs copy and import from g1_onboard_msgs.
#
# ---------------------------------------------------------------------------
# Direction : Handler (onboard / Gearsonic)  ->  cortex
# Purpose   : let cortex CONFIRM that a stop (cancel) actually took effect, and
#             more generally observe command execution, over plain pub/sub
#             (no ROS Action server required on the onboard side).
#
# Publish as a LEVEL-triggered status STREAM, not a one-shot event:
#   - republish the current state periodically (e.g. 10-20 Hz) AND on change;
#   - QoS RELIABLE + TRANSIENT_LOCAL (latched) so a dropped transition and a
#     late subscriber are self-healing — cortex reads the latest value and
#     never has to catch an edge in flight.
#
# Correlation: `command_id` echoes the id cortex stamped on the command this
# status refers to. Under rapid preemption several commands exist in quick
# succession, so cortex MUST match by id to know which command a status is
# about. => every command message cortex sends the Handler needs a matching
# `uint32 command_id` field (add in the same PR).
# ---------------------------------------------------------------------------

std_msgs/Header header

uint32 command_id           # id of the command this status refers to (0 = none / idle)

# --- execution state of command_id ---------------------------------------
uint8 IDLE      = 0         # Handler idle / holding, no active command
uint8 ACCEPTED  = 1         # command received, not yet executing
uint8 EXECUTING = 2         # actively executing
uint8 CANCELING = 3         # stop requested, motion winding down
uint8 CANCELED  = 4         # stopped because of a cancel — motion halted cleanly
uint8 SUCCEEDED = 5         # command finished on its own
uint8 ABORTED   = 6         # stopped by fault / safety (E-STOP), not a clean cancel
uint8 state

string detail               # human-readable note (fault / abort reason, etc.)
