# orchestrator_node -> llm_node: turn one utterance into a scenario (plan).
# Fire-and-forget on a topic (not a service): LLM latency is seconds, and the
# orchestrator's tick loop must never block on it. Correlation is by request_id —
# a newer request supersedes an older one (latest-wins, same policy as the VLA
# runner's chunk queue).

std_msgs/Header header

string request_id    # orchestrator-generated; echoed back in Plan
string text          # the final STT transcript to plan for
