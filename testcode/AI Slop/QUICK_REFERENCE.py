"""
CHESS ROBOT COORDINATOR - QUICK REFERENCE CARD

Save this for quick lookup while developing or using the system.
"""

# ============================================================================
# QUICK COMMANDS
# ============================================================================

# Run interactive simulator with UI
# $ python coordinator.py

# Run benchmarks (test algorithms)
# $ python coordinator.py benchmark 20

# Run working code examples
# $ python examples.py

# Run interactive quick-start guide
# $ python START_HERE.py

# Verify system is working
# $ python quick_validate.py


# ============================================================================
# UI KEYBOARD SHORTCUTS
# ============================================================================

# SPACE  - Switch between path planners
# P      - Toggle path visualization (show/hide dotted lines)
# R      - Reset board (clear all paths and randomization)
# ESC    - Close the application


# ============================================================================
# MOUSE CONTROLS
# ============================================================================

# CLICK "Randomize"  - Random piece positions
# CLICK "Plan Moves" - Calculate paths to starting positions
# CLICK "Execute"    - Animate movement in simulator


# ============================================================================
# CORE CLASSES TO USE IN CODE
# ============================================================================

from coordinator import (
    Position,               # (x, y, orientation)
    Piece,                  # (id, position)
    PathPlanner,            # Abstract base class
    SequentialPlanner,      # Simple sequential algorithm
    OptimizedPlanner,       # Collision-aware algorithm
    SimulatorEngine,        # Main simulator
    ChessRobotUI,           # PyGame UI
    PathPlannerBenchmark,   # Testing framework
)

# Create a position (in millimeters)
pos = Position(x=100, y=200, orientation=45.0)

# Check distance between positions
distance = pos1.distance_to(pos2)  # Returns distance in mm

# Create a chess piece
piece = Piece(id='a', position=pos)

# Create simulator
sim = SimulatorEngine()
sim.initialize_board()  # 16 chess pieces at starting positions

# Randomize positions
sim.randomize_positions()

# Check for collisions
collisions = sim.check_collisions()  # Returns list of colliding pairs

# Plan movements
planner = SequentialPathPlanner()
paths = planner.plan_movements(sim.state.pieces, target_positions)

# Execute movements
sim.start_execution(paths)

# Update simulation (call in a loop)
while sim.state.executing:
    sim.update(dt=0.016)  # ~60 FPS


# ============================================================================
# CREATING A CUSTOM PATH PLANNER
# ============================================================================

from coordinator import PathPlanner, MovePath

class MyPlanner(PathPlanner):
    def plan_movements(self, pieces, target_positions):
        paths = {}
        for piece_id, target_pos in target_positions.items():
            if piece_id not in pieces:
                continue
            
            piece = pieces[piece_id]
            waypoints = [piece.position.copy()]
            
            # Calculate angle to target
            import math
            dx = target_pos.x - piece.position.x
            dy = target_pos.y - piece.position.y
            angle = math.degrees(math.atan2(dy, dx))
            
            # Create waypoints (rotation -> move -> done)
            waypoints.append(Position(piece.position.x, piece.position.y, angle))
            waypoints.append(target_pos.copy())
            
            # Estimate duration
            distance = piece.position.distance_to(target_pos)
            duration = distance / 100.0 + 2.0  # movement + rotation time
            
            paths[piece_id] = MovePath(piece_id, waypoints, duration)
        
        return paths
    
    def get_name(self):
        return "My Custom Planner"


# ============================================================================
# CONFIGURATION TUNING
# ============================================================================

# Edit config.py to change:

BOARD_SQUARE_SIZE = 55          # Chess square size (mm)
BOARD_EXTRA_SIDE = 100          # Side margins (mm)
PIECE_RADIUS = 15               # Piece radius (30mm OD)
COLLISION_DISTANCE = 35         # Collision threshold (mm)

ROTATION_SPEED = 180            # Degrees per second
TRANSLATION_SPEED = 100         # MM per second

WINDOW_WIDTH = 1200             # Display width (pixels)
WINDOW_HEIGHT = 900             # Display height (pixels)

SIMULATOR_FPS = 60              # Update rate (frames per second)


# ============================================================================
# PIECE IDS
# ============================================================================

# All 16 chess pieces:
# Pawns:    'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'
# Back row: 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'

# Starting positions in PIECE_START_POSITIONS dict:
from coordinator import PIECE_START_POSITIONS
# {piece_id: (column, row)}
# 'a': (0, 0), 'b': (1, 0), ... 'h': (7, 0)  # Pawns at row 0
# 'A': (0, 7), 'B': (1, 7), ... 'H': (7, 7)  # Back row at row 7


# ============================================================================
# BOARD COORDINATES
# ============================================================================

# Board is 8x8 with chess notation:
# Columns: a b c d e f g h (0-7)
# Rows: 1 2 3 4 5 6 7 8 (0-7 internally)

# Convert to world coordinates (mm):
from coordinator import BOARD_SQUARE_SIZE, BOARD_EXTRA_SIDE

column = 3  # 'c'
row = 4     # Piece on row 4

world_x = BOARD_EXTRA_SIDE + column * BOARD_SQUARE_SIZE
world_y = row * BOARD_SQUARE_SIZE

# Piece position at column 'c', row 4:
pos = Position(x=world_x, y=world_y, orientation=0.0)


# ============================================================================
# RUNNING BENCHMARKS PROGRAMMATICALLY
# ============================================================================

from coordinator import PathPlannerBenchmark, SequentialPathPlanner

benchmark = PathPlannerBenchmark(iterations=10)
planner = SequentialPathPlanner()

results = benchmark.benchmark_planner(planner)

# Results dictionary contains:
# {
#     'planner': str,                    # Planner name
#     'iterations': int,                 # Number of iterations
#     'avg_move_time': float,            # Average seconds
#     'avg_execution_time': float,       # Average seconds
#     'total_move_time': float,          # Total seconds
#     'collisions': int,                 # Total collision count
#     'collision_rate': float,           # Collisions per iteration
#     'avg_accuracy_error': float,       # Average mm error
# }

print(f"Move time: {results['avg_move_time']:.2f}s")
print(f"Collisions: {results['collisions']}")
print(f"Accuracy error: {results['avg_accuracy_error']:.2f}mm")


# ============================================================================
# MOVEMENT WAYPOINTS
# ============================================================================

# Rotation only:
waypoints = [
    Position(100, 100, 0),      # Current position, facing right
    Position(100, 100, 90),     # Same position, now facing up
]

# Straight movement:
waypoints = [
    Position(0, 0, 0),          # Start
    Position(100, 0, 0),        # Move 100mm right
]

# Arc movement (approximated with waypoints):
waypoints = [
    Position(0, 0, 0),
    Position(50, 50, 45),       # Intermediate point
    Position(100, 100, 90),     # End point
]

# Combined (rotate + move):
waypoints = [
    Position(100, 100, 0),      # Start position, facing right
    Position(100, 100, 45),     # Rotate 45 degrees
    Position(200, 150, 45),     # Move while facing 45 degrees
]


# ============================================================================
# DEBUGGING TIPS
# ============================================================================

# Enable debug printing in config.py:
DEBUG_MODE = True
VERBOSE_PLANNER_OUTPUT = True
VERBOSE_SIMULATOR_OUTPUT = True

# Check piece positions:
for piece_id, piece in sim.state.pieces.items():
    print(f"{piece_id}: ({piece.position.x}, {piece.position.y}, {piece.position.orientation})")

# Check collision details:
collisions = sim.check_collisions()
if collisions:
    for piece1, piece2 in collisions:
        p1 = sim.state.pieces[piece1]
        p2 = sim.state.pieces[piece2]
        distance = p1.distance_to(p2)
        print(f"Collision: {piece1} <-> {piece2}, distance: {distance:.1f}mm")

# Check path details:
for piece_id, path in sim.state.paths.items():
    print(f"{piece_id}: {len(path.waypoints)} waypoints, {path.duration:.2f}s")


# ============================================================================
# COMMON TASKS
# ============================================================================

# Task: Randomize and plan
sim.randomize_positions()
paths = planner.plan_movements(sim.state.pieces, target_positions)
sim.state.paths = paths

# Task: Execute and measure time
import time
start = time.time()
sim.start_execution(paths)
while sim.state.executing:
    sim.update(0.016)
elapsed = time.time() - start
print(f"Execution took {elapsed:.2f}s")

# Task: Check accuracy after execution
for piece_id, piece in sim.state.pieces.items():
    if piece_id in target_positions:
        target = target_positions[piece_id]
        error = piece.position.distance_to(target)
        print(f"{piece_id}: error {error:.2f}mm")

# Task: Compare two planners
planner1 = SequentialPathPlanner()
planner2 = OptimizedPathPlanner()

paths1 = planner1.plan_movements(sim.state.pieces, targets)
paths2 = planner2.plan_movements(sim.state.pieces, targets)

time1 = sum(p.duration for p in paths1.values())
time2 = sum(p.duration for p in paths2.values())

print(f"{planner1.get_name()}: {time1:.1f}s")
print(f"{planner2.get_name()}: {time2:.1f}s")


# ============================================================================
# FILE STRUCTURE
# ============================================================================

"""
coordinator.py           - Main application (UI, simulator, planners)
custom_planners.py       - Example custom implementations
config.py                - All configuration parameters
examples.py              - Working code examples
START_HERE.py            - Interactive guide
quick_validate.py        - Fast validation
validate.py              - Comprehensive tests
requirements.txt         - Dependencies
README.md                - Full documentation
IMPLEMENTATION.md        - Implementation details
FILES_CREATED.md         - Summary of what was created
"""


# ============================================================================
# TROUBLESHOOTING
# ============================================================================

# PyGame import error:
# Solution: pip install pygame

# ModuleNotFoundError:
# Solution: Make sure you're in the correct directory

# High collision rate:
# Solution: Use OptimizedPlanner or increase COLLISION_DISTANCE

# Slow benchmarks:
# Solution: Use fewer iterations (e.g., benchmark 5 instead of 20)

# Window won't open:
# Solution: Check graphics drivers, try pygame initialization test
