# Assistive Robot Behavior Tree (BT) Project

This project implements a modular, hierarchical Behavior Tree (BT) architecture in Python for an intelligent assistive robot supporting elderly people at home. It now includes LLM (Large Language Model) integration to interpret human commands and trigger the correct robot behaviors, as well as dynamic user preferences for personalized action ordering.

---

## Project Structure & Explanation

```
BT/
  core/        # Core BT node classes and subtree support
  nodes/       # Example Condition and Action node functions
  trees/       # Main and modular BT tree definitions
  engine/      # BT execution engine
  llm/         # LLM interface and intent mapping
  main.py      # Demo and interactive entry point
```

### 1. `core/`
- **bt_node.py**: Defines the fundamental BT node types: `Node`, `Sequence`, `Selector`, `Condition`, `Action`, and `PreferenceSequence`. `PreferenceSequence` dynamically orders its children based on user preferences stored in the blackboard. All nodes can be tagged for semantic matching.
- **subtree.py**: Allows embedding reusable subtrees for modular, hierarchical BT design.

### 2. `nodes/`
- **conditions.py**: Contains functions that check the robot's world state (e.g., `emergency_detected`, `scheduled_task_time`). Used by `Condition` nodes.
- **actions.py**: Contains functions that perform actions (e.g., `assess_emergency`, `notify_caregiver`, `clean_glasses`, `clean_table`, `clean_floor`). Cleaning actions are tagged for preference-based ordering.

### 3. `trees/`
- **main_tree.py**: Assembles the main behavior tree using the core nodes and example conditions/actions. Includes a cleaning task subtree using `PreferenceSequence` to demonstrate user preference-aware behavior.

### 4. `engine/`
- **bt_engine.py**: Implements the BT execution engine. It ticks the tree, manages the blackboard (shared memory), and prints status for each tick.

### 5. `llm/`
- **llm_interface.py**: Connects to an LLM (e.g., OpenAI GPT) to interpret human natural language commands and classify them into robot intents (like 'emergency', 'user_request', etc.).
- **intent_mapper.py**: Maps the LLM's intent output to the BT blackboard, triggering the correct behavior in the tree.

### 6. `main.py`
- Entry point for the project. Supports two modes:
  - **Demo Mode**: Runs predefined scenarios (emergency, scheduled task, user request, routine check, and cleaning with user preferences) to show the BT in action.
  - **Interactive LLM Mode**: Accepts human commands, uses the LLM to interpret them, and updates the BT blackboard to trigger the appropriate behavior. Also allows direct setting of user preferences (e.g., cleaning order).

---

## Dynamic User Preferences & Preference-Aware Behavior

- **User Preferences**: The robot can store and use user preferences (e.g., preferred order for cleaning tasks) in the blackboard under a `preferences` key.
- **PreferenceSequence Node**: A special BT node that dynamically orders its children based on the user's current preferences. Each child action is tagged (e.g., 'glasses', 'table', 'floor') and executed in the preferred order.
- **Learning/Updating Preferences**: Preferences can be updated at runtime, either by direct command (e.g., `set cleaning order to table, glasses, floor`) or by learning from user feedback (extendable via LLM or feedback module).
- **Demo**: The cleaning scenario demonstrates how the robot adapts its behavior when the user changes their cleaning order preference.

---

## How the System Works

1. **Behavior Tree (BT) Core**: The robot's logic is structured as a tree of nodes. Each node can be a Sequence (all children must succeed), Selector (first child to succeed), Condition (checks a fact), Action (performs a task), or PreferenceSequence (orders children by user preference).
2. **Blackboard**: Shared memory (a Python dict) that holds the robot's current state, triggers, and user preferences.
3. **LLM Integration**: In interactive mode, the user types a natural language command. The LLM interprets it as an intent, which is mapped to the blackboard, activating the relevant part of the BT.
4. **Preference-Aware Execution**: When a PreferenceSequence node is ticked, it checks the user's preferences and executes actions in the preferred order.
5. **Execution Engine**: Ticks the tree, causing it to check conditions and perform actions based on the current blackboard state and preferences.

---

## How to Run

1. **Install dependencies** (for LLM mode):
   ```bash
   pip install openai
   ```
2. **Set your OpenAI API key** (for LLM mode):
   ```bash
   set OPENAI_API_KEY=sk-...your-key...
   ```
3. **Run the project**:
   ```bash
   python main.py
   ```
4. **Choose a mode**:
   - Enter `1` for demo scenarios (no LLM required)
   - Enter `2` for interactive LLM mode (requires OpenAI API key)

---

## Extending the Project
- **Add new robot behaviors**: Create new condition/action functions in `nodes/` and add them to the tree in `trees/`.
- **Support more intents or preferences**: Update the LLM prompt, intent mapping logic, and add new preference keys in `llm/` and `core/`.
- **Swap LLM provider**: Replace the code in `llm/llm_interface.py` to use a local or different cloud model.
- **Implement learning**: Add a feedback module to learn preferences from user actions or feedback automatically.

---

**For research, prototyping, and educational use.** 