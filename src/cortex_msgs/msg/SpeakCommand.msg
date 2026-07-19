# orchestrator_node -> tts_node. Text to speak, plus the command_id the
# orchestrator stamped so tts_node can echo it back in CommandStatus. That lets
# the orchestrator's action registry track speech like any other tracked action
# (nav / vla): same CommandStatus contract, same cancel/monitor path.

uint32 command_id       # 0 = untracked (fire-and-forget, no status expected)
string text
