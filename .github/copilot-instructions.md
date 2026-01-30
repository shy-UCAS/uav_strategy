# Copilot Instructions for SPADE-BDI UAV Strategy

This repository implements a Multi-Agent System (MAS) for UAV swarm situational awareness and strategy using **SPADE** (Smart Python Agent Development Environment) and **BDI** (Belief-Desire-Intention) architecture via `spade_bdi`.

## Big Picture Architecture

- **Hybrid Agents**: Agents are implemented using the `BDIAgent` class, which bridges Python execution and AgentSpeak logic.
  - **High-Level Logic**: defined in `.asl` files (AgentSpeak) handling goals (`!goal`), beliefs (`+belief`), and plans.
  - **Low-Level Execution**: defined in Python functions registered as actions (e.g., `.act_move`).
- **Communication**: 
  - **Direct**: XMPP-based via SPADE (JID/Password).
  - **Shared State**: Redis is used for high-frequency shared state updates (e.g., UAV blue force global positions) to avoid message overhead.
- **Path Planning**: Modularized in `planning_modules/` (e.g., `quick_path_planners`, `math_curves_generators`).

## Project Structure

- `spade_bdi/`: Core library for BDI agents.
- `examples/uavs_strategy/`: Main application logic.
  - `*.asl`: AgentSpeak logic files defining agent behaviors.
  - `planning_modules/`: Algorithms for trajectory generation and path finding.
  - `behaviors_modules/`: Python implementations of complex agent behaviors.
  - `run_example.py`: Entry point for orchestrating the multi-agent simulation.

## Developer Workflows

### Running Simulations
The primary way to run the system is executing specific example scripts:
```bash
python examples/uavs_strategy/run_example.py
```
*Note: Ensure an XMPP server (like Prosody or Openfire) is running or you have valid credentials if the example requires them.*

### Dependency Management
- **System Paths**: The project manually modifies `sys.path` in `run_example.py` to include `spade-master` and `spade_bdi-master` from the project root. Ensure these directories are present or the code is updated to use installed packages.
- Dependencies are listed in `requirements.txt`.

## Coding Conventions

### Defining Agents
1. **AgentSpeak (`.asl`)**: Define the cognitive logic.
   ```agentspeak
   !start.                 // Initial goal
   +!start <- .my_action.  // Plan handling the goal
   ```
2. **Python (`.py`)**: Implement the `BDIAgent` and custom actions.
   ```python
   class MyAgent(BDIAgent):
       def add_custom_actions(self, actions):
           @actions.add(".my_action", 0) # 0 is arity
           def _my_action(agent, term, intention):
               # Implementation
               yield # Important: Actions are generators
   ```

### Shared Data Guidelines
- For heavy/frequent data (like real-time coordinates), write to Redis rather than sending direct ACL messages between agents.
- Use `planning_modules` for pure calculation logic (stateless) and keep Agent classes focused on state management and communication.

### Common Pitfalls
- **Path Issues**: Be aware of relative imports and `sys.path` hacks in example scripts.
- **Action Blocking**: Agent actions should yield to allow the BDI reasoning cycle to continue.
- **ASL Syntax**: Remember ASL syntax requirements (periods `.` at end of lines, capitalized variables).
