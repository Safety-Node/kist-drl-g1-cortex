# CommandStatus — Handler (onboard / Gearsonic) -> cortex.
#
# PROTOTYPE. This type is a proposal to the module that owns the Handler, and
# belongs in g1_onboard_msgs (SSOT) once agreed. It lives here only so cortex can
# build + exercise the stop-confirmation wiring now; when it lands in SSOT, import
# it from g1_onboard_msgs and delete this file.
#
# The proposal — protocol, sequence, and what we need from the Handler — is written
# up for the other module developers on the Notion "Unitree-G1" page, under
# 전달 사항 / 07/17 / 2. Cancel Sync.
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
