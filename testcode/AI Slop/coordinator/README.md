# Chess Robot Coordinator - Reorganized Structure

## Project Structure

```
coordinator/
├── __init__.py              # Package initialization and exports
├── constants.py             # Board dimensions, physics constants, piece positions
├── data_types.py            # Data classes (Position, Piece, Commands, ExecutionPlan, etc.)
├── path_planner.py          # Base PathPlanner abstract class
├── sequential_planner.py    # Simple parallel movement planner (no collision avoidance)
├── ai_planner.py            # Advanced AI planner with collision avoidance and routing
├── utils.py                 # Helper functions
└── main.py                  # SimulatorEngine and UI classes

run_coordinator.py           # Main entry point to run the application
```

## Running the Application

```bash
python run_coordinator.py
```

## Adding New Path Planners

1. Create a new file in the `coordinator/` folder (e.g., `my_planner.py`)
2. Import the base class: `from .path_planner import PathPlanner`
3. Implement the `plan_movements()` and `get_name()` methods
4. Add your planner to `coordinator/__init__.py`
5. Import and add to the available planners in `coordinator/main.py`

### Example:

```python
# coordinator/my_planner.py
from .path_planner import PathPlanner
from .data_types import Piece, Position, ExecutionPlan

class MyCustomPlanner(PathPlanner):
    def plan_movements(self, pieces, target_positions):
        # Your planning logic here
        plan = ExecutionPlan()
        return plan
    
    def get_name(self):
        return "My Custom Planner"
```

## Key Features

### AI_Planner
- Iterative optimization with 20 random attempts
- Target assignment optimization for identical pieces (pawns, rooks, etc.)
- Collision avoidance with multiple strategies:
  - Direct path with collision checking
  - Route-around with intermediate waypoints
  - Moving blocking pieces out of the way
  - Moving blocking pieces toward their own targets first
- Randomized exploration strategies for better solutions

### Command-Based Execution
- Pieces execute sequences of commands (ROTATE, MOVE_STRAIGHT, WAIT)
- Multiple command sequences can be queued per piece
- Parallel execution with precise timing control
- Smooth interpolation during animation

## Controls

- **Randomize Button**: Randomly place all pieces on the board
- **Plan Moves Button**: Calculate movement paths to starting positions
- **Execute Button**: Animate the planned movements
- **SPACE**: Cycle between available path planners
- **P**: Toggle path visualization
- **R**: Reset board to starting positions
