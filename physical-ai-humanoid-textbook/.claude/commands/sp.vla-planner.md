# VLA Task Planner Generator

Generate Vision-Language-Action planners that convert LLM outputs to ROS action sequences.

## Capabilities

- Create Python nodes integrating OpenAI/Anthropic APIs
- Parse natural language to robot action primitives
- Generate ROS 2 action/service clients
- Implement task state machines
- Add error handling and recovery

## Usage

User specifies:
- Available robot actions (navigate, grasp, look_at, etc.)
- Task input method (voice, text, GUI)
- LLM provider (OpenAI GPT-4, Claude)

## Output

Complete Python node with:
1. LLM API integration
2. Prompt templates for task decomposition
3. Action primitive definitions
4. State machine executor
5. ROS 2 publishers/subscribers

## Example

**Input**: "Voice command handler for 'pick and place' tasks"

**Output**: `vla_planner_node.py` with:
- Whisper ASR integration
- GPT-4 task decomposition
- Action sequence publisher
- Example commands

## Quality

- ✅ Production-grade error handling
- ✅ Timeout management
- ✅ Logging and debugging
- ✅ Example prompts included
