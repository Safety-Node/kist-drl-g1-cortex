# llm_node -> orchestrator_node: the generated scenario for one PlanRequest.
#
# The payload is a JSON string in the SAME schema as the file scenarios
# (config/scenarios/*.json5, minus `triggers` — an LLM plan is born from an
# utterance, not a keyword). One schema, one loader, one validator: the
# orchestrator parses this with the exact code path used for files, so a
# hallucinated hook label or criterion fails fast instead of half-running.
#
# Routing lives inside the plan: each sub-task's on_* hooks pick connectors
# by label (speak / navigation / vla), and a step list expresses combos
# (tts+vla = two steps). The message itself carries no routing.

std_msgs/Header header

string request_id     # echoes PlanRequest.request_id
bool   ok             # false = generation failed; see error
string error          # human-readable reason when !ok ("" otherwise)
string scenario_json  # scenario-schema JSON ("" when !ok)
