# CommandStatus — Handler (onboard / Gearsonic) -> cortex.
#
# PROTOTYPE COPY. The canonical proposal is docs/proposals/CommandStatus.msg and
# this type belongs in g1_onboard_msgs (SSOT). It lives here only so cortex can
# build + exercise the stop-confirmation wiring now; once it lands in SSOT via
# PR, import it from g1_onboard_msgs and delete this file. Keep the two in sync.
#
# Publish as a LEVEL-triggered stream (periodic + on-change), QoS RELIABLE +
# TRANSIENT_LOCAL, so a dropped transition / late subscriber is self-healing.

std_msgs/Header header

uint32 command_id           # id of the command this status refers to (0 = none / idle)

uint8 IDLE      = 0         # Handler idle / holding, no active command
uint8 ACCEPTED  = 1         # command received, not yet executing
uint8 EXECUTING = 2         # actively executing
uint8 CANCELING = 3         # stop requested, motion winding down
uint8 CANCELED  = 4         # stopped because of a cancel — motion halted cleanly
uint8 SUCCEEDED = 5         # command finished on its own
uint8 ABORTED   = 6         # stopped by fault / safety (E-STOP), not a clean cancel
uint8 state

string detail               # human-readable note (fault / abort reason, etc.)
