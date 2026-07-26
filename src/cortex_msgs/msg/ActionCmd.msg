# orchestrator_node -> an action module (tts, and later nav / vla). Open-loop:
# the orchestrator fires the command and does not wait for confirmation, so there
# is no command_id / status correlation. `text` is the payload — text to speak, a
# nav goal name, a VLA prompt, etc.
string text
