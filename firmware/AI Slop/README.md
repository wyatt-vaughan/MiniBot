# Chess Robot Coordinator

A comprehensive master controller system for a chess-playing robot that manages piece communication, movement planning, and game rule enforcement with a visual simulator.

## Features

### Core Functionality
- **Visual Simulator**: Real-time display of the chess board and piece positions
- **Piece Representation**: 30mm OD pieces displayed with orientation indicators
- **Board Layout**: 55mm square chess board with 100mm margins on sides
- **Movement Types**: Rotation, straight-line movement, and arc motion support

### Movement Planning
- **Modular Path Planning**: Easily swappable path planning algorithms via abstract base class
- **Two Built-in Planners**:
  - **Sequential Planner**: Simple sequential movement of pieces
  - **Optimized Planner**: Collision-aware planner with detour capabilities
- **Collision Detection**: Real-time detection of piece collisions
- **Path Visualization**: Dotted line display of planned movement paths with target markers

### Interactive UI
Three main control buttons:
1. **Randomize**: Randomly repositions all pieces on the board
2. **Plan Moves**: Calculates optimal movement paths to return pieces to starting chess positions
3. **Execute**: Animates the planned movements in real-time simulator mode

### Testing & Benchmarking
- **Automated Testing Mode**: Loop randomization → planning → execution → collision checking
- **Performance Metrics**: Tracks move time, execution time, collision rates, and positioning accuracy
- **Algorithm Comparison**: Run multiple planners against the same test cases to compare performance

## Installation

### Prerequisites
- Python 3.7+
- pip (Python package manager)

### Setup

```bash
# Navigate to the project directory
cd "C:\Users\DDeGo\Documents\Drew\Projects\MiniBot\firmware\AI Slop"

# Install dependencies
python setup.py
# OR manually:
pip install -r requirements.txt
```

## Usage

### Interactive Simulator Mode

```bash
python coordinator.py
```

This launches the PyGame-based UI with the following controls:

**Mouse Controls:**
- Click "Randomize" button: Randomly position all pieces on the board
- Click "Plan Moves" button: Calculate movement paths to starting positions
- Click "Execute" button: Animate the planned movements

**Keyboard Controls:**
- `SPACE`: Switch between available path planners
- `P`: Toggle path visualization on/off
- `R`: Reset board to initial state (clears positions and paths)

### Benchmark Mode

Run automated testing to score path planning algorithms:

```bash
# Run 10 iterations (default)
python coordinator.py benchmark

# Run custom number of iterations
python coordinator.py benchmark 50
```

This mode:
1. Randomizes piece positions
2. Plans movements to starting positions
3. Executes movements in simulator
4. Checks for collisions
5. Measures positioning accuracy
6. Calculates total move time
7. Repeats for specified iterations
8. Outputs detailed statistics for each planner

**Benchmark Output Metrics:**
- Average move time (seconds)
- Total move time across all iterations
- Average execution time
- Collision statistics
- Positional accuracy error (mm)

## Architecture

### Core Components

#### Position Class
Represents 2D location and orientation of pieces
- `x`, `y`: Coordinates in millimeters
- `orientation`: Rotation angle in degrees

#### Piece Class
Represents a chess piece on the board
- `id`: Piece identifier (a-h for pawns, A-H for back row)
- `position`: Current Position object
- `start_position`: Original starting position

#### PathPlanner (Abstract Base Class)
Interface for implementing different movement planning algorithms

```python
class PathPlanner(ABC):
    def plan_movements(self, pieces: Dict[str, Piece], 
                      target_positions: Dict[str, Position]) -> Dict[str, MovePath]:
        """Plan movements for all pieces"""
        pass
    
    def get_name(self) -> str:
        """Get planner name"""
        pass
```

#### SimulatorEngine
Manages the simulation state and piece movement execution
- Initializes board with pieces
- Randomizes positions
- Executes movement paths
- Updates piece positions based on elapsed time
- Detects collisions

#### ChessRobotUI
Pygame-based user interface
- Renders chess board with pieces
- Displays movement paths
- Handles user input
- Shows real-time status and metrics

#### PathPlannerBenchmark
Automated testing framework for comparing planners
- Runs multiple iterations of randomization → planning → execution
- Collects performance metrics
- Generates comparative statistics

## Extending the System

### Adding a New Path Planner

Create a new class inheriting from `PathPlanner`:

```python
class MyCustomPlanner(PathPlanner):
    def plan_movements(self, pieces: Dict[str, Piece],
                      target_positions: Dict[str, Position]) -> Dict[str, MovePath]:
        # Your planning logic here
        paths = {}
        for piece_id, target_pos in target_positions.items():
            if piece_id not in pieces:
                continue
            
            piece = pieces[piece_id]
            waypoints = [piece.position.copy()]
            # ... add logic to create waypoints ...
            waypoints.append(target_pos.copy())
            
            paths[piece_id] = MovePath(
                piece_id=piece_id,
                waypoints=waypoints,
                duration=estimated_duration  # in seconds
            )
        
        return paths
    
    def get_name(self) -> str:
        return "My Custom Planner"
```

Register your planner in the UI:

```python
# In ChessRobotUI.__init__():
self.available_planners = [
    SequentialPathPlanner(),
    OptimizedPathPlanner(),
    MyCustomPlanner(),  # Add your planner here
]
```

Then switch between planners using SPACE key in the UI.

### Customizing Board Parameters

Edit the constants at the top of `coordinator.py`:

```python
BOARD_SQUARE_SIZE = 55        # Chess board square size in mm
BOARD_EXTRA_SIDE = 100        # Extra margin on left/right in mm
PIECE_RADIUS = 15             # Piece radius in mm (30mm OD)
BOARD_DISPLAY_SQUARE_SIZE = 70  # Display scaling
```

### Movement Type Implementation

The `MovePath` class uses waypoints for movement. Each waypoint is a `Position` object containing:
- `x`, `y`: Location in mm
- `orientation`: Rotation angle in degrees

The simulator linearly interpolates between waypoints, making it suitable for:
- **Rotation**: Create waypoint with same x,y but different orientation
- **Straight Movement**: Create waypoint with new x,y coordinates
- **Arc Motion**: Create multiple waypoints tracing the desired arc

## Data Flow

```
User Input (Buttons/Keys)
    ↓
[Plan Movements] → PathPlanner.plan_movements()
    ↓
Generate MovePath objects
    ↓
[Execute] → SimulatorEngine.start_execution()
    ↓
SimulatorEngine.update() [60 FPS]
    ↓
Interpolate piece positions between waypoints
    ↓
Detect collisions
    ↓
Render UI
```

## Performance Considerations

### Path Planning
- **Sequential Planner**: O(n) complexity, suitable for simple environments
- **Optimized Planner**: O(n²) complexity due to collision checking, more robust

### Simulator
- 60 FPS update rate for smooth animation
- Collision detection runs every frame (~16ms)
- Path interpolation is linear and efficient

### Benchmarking
- Each iteration includes actual simulation of movement
- Move time estimation based on distance and rotation requirements
- Measurements include full round-trip execution, not just planning

## Future Enhancement Ideas

1. **Advanced Path Planning**
   - RRT* (Rapidly-exploring Random Trees)
   - Rapidly-exploring Random Graphs (RRG)
   - Potential field methods
   - Genetic algorithm optimizers

2. **Movement Refinement**
   - Curved path generation for smooth arcs
   - Velocity profiles for realistic acceleration
   - Time-optimal path planning

3. **Hardware Integration**
   - ESP-NOW communication protocol
   - Real piece position feedback
   - Actual movement execution

4. **Game Logic**
   - Chess rule enforcement
   - Move validation
   - Endgame support

5. **UI Enhancements**
   - 3D visualization option
   - Save/load board states
   - Replay recorded movements
   - Interactive move planning

## Troubleshooting

### PyGame Window Not Opening
- Ensure pygame is installed: `pip install pygame`
- Check that your display drivers are up to date

### High Collision Rates in Benchmark
- Increase `COLLISION_DISTANCE` constant for more margin
- Switch to OptimizedPlanner which includes collision avoidance
- Increase board size or piece spacing

### Slow Execution
- Reduce benchmark iterations
- Check system resources (CPU/RAM usage)
- Disable path visualization (press P key) for faster UI refresh

## License

This project is part of the MiniBot chess robot system.

## Author

Drew DeGonge
