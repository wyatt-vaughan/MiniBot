"""
Configuration file for Chess Robot Coordinator

Adjust these settings to customize behavior, tuning, and display options
"""

# ============================================================================
# Physical Board Parameters
# ============================================================================

# Board geometry (in millimeters)
BOARD_SQUARE_SIZE = 55          # Size of each chess board square
BOARD_EXTRA_SIDE = 100          # Extra space on left and right margins
BOARD_TOTAL_WIDTH = 8 * BOARD_SQUARE_SIZE + 2 * BOARD_EXTRA_SIDE  # 640mm
BOARD_TOTAL_HEIGHT = 8 * BOARD_SQUARE_SIZE  # 440mm

# Piece dimensions (in millimeters)
PIECE_DIAMETER = 30             # Outer diameter of pieces
PIECE_RADIUS = PIECE_DIAMETER / 2  # 15mm
COLLISION_MARGIN = 5            # Safety margin beyond piece radius

# Effective collision distance
COLLISION_DISTANCE = 2 * PIECE_RADIUS + COLLISION_MARGIN  # 35mm

# ============================================================================
# Movement Parameters
# ============================================================================

# Estimated movement speeds (tuned based on real robot)
ROTATION_SPEED = 180            # degrees per second
TRANSLATION_SPEED = 100         # mm per second (can be updated from real data)
ARC_SPEED = 75                  # mm per second along arc

# Minimum and maximum parameters
MIN_MOVE_DISTANCE = 5           # mm, ignore moves shorter than this
MAX_MOVE_DISTANCE = 1000        # mm, longest feasible move
MIN_ROTATION_ANGLE = 1          # degrees, ignore rotations smaller than this
MAX_ROTATION_ANGLE = 360        # degrees, full rotation

# ============================================================================
# Simulator Display Parameters
# ============================================================================

# Window dimensions
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 900

# Board display scaling
BOARD_DISPLAY_OFFSET_X = 50     # Left margin in pixels
BOARD_DISPLAY_OFFSET_Y = 50     # Top margin in pixels
BOARD_DISPLAY_SQUARE_SIZE = 70  # Display size of one board square in pixels

# Piece display
PIECE_DISPLAY_RADIUS = 15       # Pixels for rendering pieces
PIECE_COLOR_LIGHT = (100, 200, 255)  # Pawns (light pieces)
PIECE_COLOR_DARK = (255, 150, 100)   # Back row (dark pieces)
PIECE_OUTLINE_WIDTH = 2         # Pixels

# Board colors (classic chess board)
BOARD_LIGHT_SQUARE = (240, 217, 181)
BOARD_DARK_SQUARE = (181, 136, 99)
BOARD_OUTLINE_COLOR = (0, 0, 0)
BOARD_OUTLINE_WIDTH = 1

# Path visualization
PATH_LINE_COLOR = (150, 150, 150)
PATH_LINE_DASH_SIZE = 3
PATH_LINE_DASH_GAP = 5
PATH_TARGET_COLOR = (100, 255, 100)
PATH_TARGET_RADIUS = 8

# UI elements
UI_BUTTON_COLOR = (200, 200, 200)
UI_BUTTON_OUTLINE = (0, 0, 0)
UI_BUTTON_OUTLINE_WIDTH = 2
UI_BUTTON_WIDTH = 150
UI_BUTTON_HEIGHT = 40

# Font sizes
FONT_SIZE_SMALL = 24
FONT_SIZE_MEDIUM = 32
FONT_SIZE_LARGE = 48

# ============================================================================
# Simulation Parameters
# ============================================================================

# Simulator frame rate
SIMULATOR_FPS = 60
SIMULATOR_DT = 1.0 / SIMULATOR_FPS  # ~16.67ms per frame

# Movement interpolation
USE_LINEAR_INTERPOLATION = True  # vs spline-based
INTERPOLATION_EPSILON = 0.001   # How close to waypoint before moving to next

# Collision detection
CHECK_COLLISIONS_EVERY_FRAME = True
ENABLE_CONTINUOUS_COLLISION = True  # Check during movement, not just endpoints

# ============================================================================
# Path Planning Parameters
# ============================================================================

# Sequential Planner
SEQUENTIAL_ROTATION_TIME = 2.0  # seconds for any rotation (estimate)
SEQUENTIAL_TRANSLATION_RATIO = 1.0  # seconds per 100mm

# Optimized Planner
OPTIMIZED_DETOUR_DISTANCE = 100  # mm offset for collision avoidance
OPTIMIZED_CLUSTER_RADIUS = 200   # mm, pieces within this distance are grouped
OPTIMIZED_CHECK_PATH_RESOLUTION = 10  # mm, spacing for path collision checks

# Movement order strategy
MOVEMENT_PRIORITY = "nearest_first"  # Options: "nearest_first", "sequential", "furthest_first"

# ============================================================================
# Benchmarking Parameters
# ============================================================================

# Default benchmark settings
DEFAULT_BENCHMARK_ITERATIONS = 10
BENCHMARK_SAVE_RESULTS = True
BENCHMARK_RESULTS_FILE = "benchmark_results.csv"

# Performance thresholds (for evaluation)
ACCEPTABLE_COLLISION_RATE = 0.0  # No collisions acceptable
ACCEPTABLE_POSITIONING_ERROR = 10.0  # mm, tolerance for final position
ACCEPTABLE_MOVE_TIME = 60.0  # seconds max

# ============================================================================
# Logging and Debug Options
# ============================================================================

DEBUG_MODE = False
DEBUG_PRINT_PATHS = False
DEBUG_PRINT_COLLISIONS = True
DEBUG_DRAW_COLLISION_ZONES = False
DEBUG_DRAW_WAYPOINT_CIRCLES = False

VERBOSE_PLANNER_OUTPUT = False
VERBOSE_SIMULATOR_OUTPUT = False

# ============================================================================
# Control Parameters (Keyboard/Mouse)
# ============================================================================

# Keybindings
KEY_SWITCH_PLANNER = "space"
KEY_TOGGLE_PATHS = "p"
KEY_RESET_BOARD = "r"
KEY_PAUSE_SIMULATION = "pause"
KEY_SAVE_STATE = "s"
KEY_LOAD_STATE = "l"

# Mouse sensitivity
BUTTON_HOVER_HIGHLIGHT = True

# ============================================================================
# Output and Reporting
# ============================================================================

# Benchmark report format
REPORT_INCLUDE_STATISTICS = True
REPORT_INCLUDE_GRAPHS = False
REPORT_CSV_FORMAT = True
REPORT_JSON_FORMAT = False

# Metrics to track
TRACK_MOVE_TIME = True
TRACK_EXECUTION_TIME = True
TRACK_COLLISION_RATE = True
TRACK_POSITIONING_ACCURACY = True
TRACK_WAYPOINT_EFFICIENCY = True

# ============================================================================
# Advanced Options
# ============================================================================

# Piece position randomization
RANDOMIZATION_ALLOW_BOARD_EDGES = True  # Can place pieces at board boundary
RANDOMIZATION_ALLOW_OVERLAPS = False    # Strict no-overlap requirement
RANDOMIZATION_MAX_ATTEMPTS = 100        # Give up after this many tries
RANDOMIZATION_PLACEMENT_ATTEMPTS_PER_PIECE = 10

# Collision detection
COLLISION_DETECTION_METHOD = "circle"  # "circle" or "rectangle"
USE_SPATIAL_HASHING = False  # Optimize collision detection with spatial hash

# Path smoothing
ENABLE_PATH_SMOOTHING = False
PATH_SMOOTHING_ITERATIONS = 3
PATH_SMOOTHING_WINDOW = 3

# Dynamic recalculation
ENABLE_DYNAMIC_REPLANNING = False  # Replan if collision detected during execution
REPLAN_ON_COLLISION = True
MAX_REPLAN_ATTEMPTS = 3

# ============================================================================
# Hardware Integration (Future)
# ============================================================================

# ESP-NOW settings
ENABLE_HARDWARE_COMMUNICATION = False
HARDWARE_TIMEOUT = 5.0  # seconds
HARDWARE_BAUD_RATE = 115200
HARDWARE_PORT = "COM3"  # Update for your system

# Real piece feedback
ENABLE_REAL_POSITION_FEEDBACK = False
FEEDBACK_UPDATE_RATE = 10  # Hz
POSITION_SYNC_TOLERANCE = 5  # mm, threshold to trigger resync

# ============================================================================
# Function to load and apply configurations
# ============================================================================

def get_config():
    """Return current configuration as dictionary"""
    return {k: v for k, v in globals().items() if k.isupper()}


def validate_config():
    """Validate configuration parameters"""
    errors = []
    
    if BOARD_SQUARE_SIZE <= 0:
        errors.append("BOARD_SQUARE_SIZE must be positive")
    
    if PIECE_RADIUS <= 0:
        errors.append("PIECE_RADIUS must be positive")
    
    if SIMULATOR_FPS <= 0:
        errors.append("SIMULATOR_FPS must be positive")
    
    if WINDOW_WIDTH < 400 or WINDOW_HEIGHT < 300:
        errors.append("Window dimensions too small")
    
    if DEFAULT_BENCHMARK_ITERATIONS <= 0:
        errors.append("DEFAULT_BENCHMARK_ITERATIONS must be positive")
    
    if ROTATION_SPEED <= 0 or TRANSLATION_SPEED <= 0:
        errors.append("Movement speeds must be positive")
    
    return errors


if __name__ == "__main__":
    errors = validate_config()
    if errors:
        print("Configuration errors:")
        for error in errors:
            print(f"  - {error}")
    else:
        print("Configuration validated successfully")
        print("\nKey parameters:")
        print(f"  Board: {BOARD_TOTAL_WIDTH}mm x {BOARD_TOTAL_HEIGHT}mm")
        print(f"  Piece diameter: {PIECE_DIAMETER}mm")
        print(f"  Collision distance: {COLLISION_DISTANCE}mm")
        print(f"  Movement speed: {TRANSLATION_SPEED}mm/s")
        print(f"  Display FPS: {SIMULATOR_FPS}")
        print(f"  Window: {WINDOW_WIDTH}x{WINDOW_HEIGHT}")
