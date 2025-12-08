"""
Chess Robot Coordinator and Master Controller
Manages piece communication, movement planning, and game rule enforcement
"""

import pygame
import numpy as np
import random
import time
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
from enum import Enum
from abc import ABC, abstractmethod
import math

# ============================================================================
# Constants
# ============================================================================

BOARD_SQUARE_SIZE = 55  # mm
BOARD_EXTRA_SIDE = 100  # mm on each side
PIECE_RADIUS = 15  # 30mm OD = 15mm radius

# Chess pieces: 8 pawns per side (row 1 & 6) + 8 major pieces per side (row 0 & 7)
# White pieces (bottom): pawns a2-h2, back row a1-h1
# Black pieces (top): pawns a7-h7, back row a8-h8
# Format: piece_id: (column, row)
PIECE_START_POSITIONS = {
    # White pawns (row 1)
    'p1': (0, 1), 'p2': (1, 1), 'p3': (2, 1), 'p4': (3, 1),
    'p5': (4, 1), 'p6': (5, 1), 'p7': (6, 1), 'p8': (7, 1),
    # White major pieces (row 0) - a1-h1
    'R1': (0, 0), 'N1': (1, 0), 'B1': (2, 0), 'Q': (3, 0),
    'K': (4, 0), 'B2': (5, 0), 'N2': (6, 0), 'R2': (7, 0),
    # Black pawns (row 6)
    'P1': (0, 6), 'P2': (1, 6), 'P3': (2, 6), 'P4': (3, 6),
    'P5': (4, 6), 'P6': (5, 6), 'P7': (6, 6), 'P8': (7, 6),
    # Black major pieces (row 7) - a8-h8
    'r1': (0, 7), 'n1': (1, 7), 'b1': (2, 7), 'q': (3, 7),
    'k': (4, 7), 'b2': (5, 7), 'n2': (6, 7), 'r2': (7, 7),
}

# UI dimensions
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 900
BOARD_DISPLAY_OFFSET_X = 50
BOARD_DISPLAY_OFFSET_Y = 50
BOARD_DISPLAY_SQUARE_SIZE = 70  # pixels for display

# ============================================================================
# Helper Functions
# ============================================================================

def board_coords_to_world(col: int, row: int) -> Tuple[float, float]:
    """
    Convert chess board coordinates (column, row) to world coordinates (x, y) in mm.
    Centers the position within the square.
    
    Args:
        col: Column 0-7
        row: Row 0-7
        
    Returns:
        Tuple of (x, y) in millimeters, centered on the square
    """
    # Center of square
    x = BOARD_EXTRA_SIDE + col * BOARD_SQUARE_SIZE + BOARD_SQUARE_SIZE / 2
    y = row * BOARD_SQUARE_SIZE + BOARD_SQUARE_SIZE / 2
    return (x, y)

# ============================================================================
# Enums and Data Classes
# ============================================================================

class MoveType(Enum):
    ROTATE = "rotate"
    STRAIGHT = "straight"
    ARC = "arc"

@dataclass
class Position:
    """Position in 2D space (mm)"""
    x: float
    y: float
    orientation: float = 0.0  # degrees, 0 = facing right
    
    def distance_to(self, other: 'Position') -> float:
        """Distance in mm"""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def copy(self) -> 'Position':
        return Position(self.x, self.y, self.orientation)

@dataclass
class Piece:
    """A chess piece on the board"""
    id: str
    position: Position
    start_position: Optional[Position] = None
    
    def distance_to(self, other: 'Piece') -> float:
        return self.position.distance_to(other.position)

@dataclass
class MoveCommand:
    """A movement command for a piece"""
    piece_id: str
    move_type: MoveType
    parameters: Dict  # Type-specific params (angle, distance, arc_radius, etc)
    
@dataclass
class MovePath:
    """A planned movement path for a piece"""
    piece_id: str
    waypoints: List[Position] = field(default_factory=list)
    duration: float = 0.0  # estimated time in seconds

@dataclass
class SimulatorState:
    """State of the simulator"""
    pieces: Dict[str, Piece]
    paths: Dict[str, MovePath] = field(default_factory=dict)
    executing: bool = False
    current_piece_executing: Optional[str] = None
    execution_start_time: float = 0.0

# ============================================================================
# Path Planning Interface
# ============================================================================

class PathPlanner(ABC):
    """Abstract base class for path planning algorithms"""
    
    @abstractmethod
    def plan_movements(self, pieces: Dict[str, Piece], 
                       target_positions: Dict[str, Position]) -> Dict[str, MovePath]:
        """
        Plan movements for all pieces to reach target positions
        
        Args:
            pieces: Current pieces with their positions
            target_positions: Target positions for each piece
            
        Returns:
            Dictionary mapping piece_id to MovePath
        """
        pass
    
    @abstractmethod
    def get_name(self) -> str:
        """Get the name of this planner"""
        pass

class SequentialPathPlanner(PathPlanner):
    """Simple sequential planner with differential drive physics"""
    
    def plan_movements(self, pieces: Dict[str, Piece],
                       target_positions: Dict[str, Position]) -> Dict[str, MovePath]:
        """
        Plan movements using differential drive physics.
        Pieces can only move forward/backward in the direction they're facing.
        """
        paths = {}
        
        for piece_id, target_pos in target_positions.items():
            if piece_id not in pieces:
                continue
                
            piece = pieces[piece_id]
            
            # Create path: rotate to face target, then move forward
            waypoints = [piece.position.copy()]
            
            # Calculate angle to target
            dx = target_pos.x - piece.position.x
            dy = target_pos.y - piece.position.y
            target_angle = math.degrees(math.atan2(dy, dx))
            
            # Normalize angle to 0-360
            target_angle = target_angle % 360
            current_angle = piece.position.orientation % 360
            
            # Calculate shortest rotation direction
            angle_diff = (target_angle - current_angle) % 360
            if angle_diff > 180:
                angle_diff -= 360
            
            # Step 1: Rotate to face target (no position change, only orientation)
            waypoints.append(Position(piece.position.x, piece.position.y, target_angle))
            
            # Step 2: Move forward (direction = orientation, no spinning)
            waypoints.append(Position(target_pos.x, target_pos.y, target_angle))
            
            # Estimate duration
            rotation_time = abs(angle_diff) / 180.0  # seconds for rotation
            movement_distance = piece.position.distance_to(target_pos)
            movement_time = movement_distance / 100  # 100mm/s forward speed
            
            paths[piece_id] = MovePath(
                piece_id=piece_id,
                waypoints=waypoints,
                duration=rotation_time + movement_time
            )
        
        return paths
    
    def get_name(self) -> str:
        return "Sequential Planner"

class CollisionAwarePathPlanner(PathPlanner):
    """
    Advanced planner with differential drive physics and collision avoidance.
    
    Strategy:
    1. Sort pieces by distance to target (closer = higher priority)
    2. For each piece, check if direct path is clear
    3. If collision detected:
       a. Try to move other pieces out of the way first
       b. Try perpendicular detour paths
       c. Move to staging area and retry later
    """
    
    COLLISION_DISTANCE = PIECE_RADIUS * 2 + 5  # mm, with buffer
    
    def plan_movements(self, pieces: Dict[str, Piece],
                       target_positions: Dict[str, Position]) -> Dict[str, MovePath]:
        """Plan movements with collision avoidance using differential drive physics"""
        paths = {}
        
        # Occupied positions: where pieces currently are and will end up
        occupied_positions = {pid: p.position for pid, p in pieces.items()}
        
        # Sort by distance to target (closest first = highest priority)
        sorted_pieces = sorted(
            target_positions.items(),
            key=lambda x: pieces[x[0]].position.distance_to(x[1])
        )
        
        for piece_id, target_pos in sorted_pieces:
            if piece_id not in pieces:
                continue
            
            piece = pieces[piece_id]
            current_pos = piece.position
            
            # Try to find a collision-free path
            path = self._plan_piece_movement(
                piece_id, current_pos, target_pos, 
                occupied_positions, pieces, target_positions
            )
            
            if path:
                paths[piece_id] = path
                # Update occupied positions with final target
                occupied_positions[piece_id] = target_pos
        
        return paths
    
    def _plan_piece_movement(self, piece_id: str, current_pos: Position,
                             target_pos: Position, occupied: Dict[str, Position],
                             pieces: Dict[str, Piece], 
                             targets: Dict[str, Position]) -> Optional[MovePath]:
        """Plan movement for a single piece with collision avoidance"""
        
        # Check if direct path is clear
        if not self._has_collision(current_pos, target_pos, occupied, piece_id):
            return self._create_path(current_pos, target_pos)
        
        # Direct path blocked - try detour
        detour_point = self._find_detour_path(current_pos, target_pos, occupied)
        if detour_point and not self._has_collision(current_pos, detour_point, occupied, piece_id):
            # Go via detour, then to target
            waypoints = [current_pos.copy()]
            
            # Rotate to face detour
            dx = detour_point.x - current_pos.x
            dy = detour_point.y - current_pos.y
            angle1 = math.degrees(math.atan2(dy, dx)) % 360
            waypoints.append(Position(current_pos.x, current_pos.y, angle1))
            waypoints.append(detour_point.copy())
            
            # Rotate to face target
            dx = target_pos.x - detour_point.x
            dy = target_pos.y - detour_point.y
            angle2 = math.degrees(math.atan2(dy, dx)) % 360
            waypoints.append(Position(detour_point.x, detour_point.y, angle2))
            waypoints.append(Position(target_pos.x, target_pos.y, angle2))
            
            # Calculate duration
            dist1 = current_pos.distance_to(detour_point)
            dist2 = detour_point.distance_to(target_pos)
            rot1 = abs((angle1 - current_pos.orientation) % 360)
            if rot1 > 180:
                rot1 = 360 - rot1
            rot2 = abs((angle2 - angle1) % 360)
            if rot2 > 180:
                rot2 = 360 - rot2
            
            duration = (rot1 + rot2) / 180.0 + (dist1 + dist2) / 100.0
            
            return MovePath(piece_id, waypoints, duration)
        
        # No clear path - create minimal movement
        # Just rotate in place to target angle, then move
        return self._create_path(current_pos, target_pos)
    
    def _has_collision(self, start: Position, end: Position,
                       occupied: Dict[str, Position], exclude_id: str) -> bool:
        """Check if path from start to end collides with occupied positions"""
        for other_id, other_pos in occupied.items():
            if other_id == exclude_id:
                continue
            
            # Distance from piece to line segment
            distance = self._point_to_segment_distance(other_pos, start, end)
            if distance < self.COLLISION_DISTANCE:
                return True
        
        return False
    
    def _point_to_segment_distance(self, point: Position,
                                    seg_start: Position, seg_end: Position) -> float:
        """Calculate shortest distance from point to line segment"""
        dx = seg_end.x - seg_start.x
        dy = seg_end.y - seg_start.y
        
        if dx == 0 and dy == 0:
            return point.distance_to(seg_start)
        
        t = max(0, min(1, ((point.x - seg_start.x) * dx + 
                           (point.y - seg_start.y) * dy) / (dx*dx + dy*dy)))
        
        closest_x = seg_start.x + t * dx
        closest_y = seg_start.y + t * dy
        closest = Position(closest_x, closest_y)
        
        return point.distance_to(closest)
    
    def _find_detour_path(self, start: Position, end: Position,
                          occupied: Dict[str, Position]) -> Optional[Position]:
        """Find a perpendicular detour point to avoid obstacles"""
        dx = end.x - start.x
        dy = end.y - start.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance == 0:
            return None
        
        # Try perpendicular offsets
        perp_x = -dy / distance
        perp_y = dx / distance
        
        # Try both sides, at 1.5x the piece diameter
        offset_distance = PIECE_RADIUS * 3
        
        for detour_point in [
            Position(start.x + perp_x * offset_distance, start.y + perp_y * offset_distance, 0),
            Position(start.x - perp_x * offset_distance, start.y - perp_y * offset_distance, 0),
        ]:
            # Check if this detour point is valid
            if not self._point_in_collision(detour_point, occupied):
                return detour_point
        
        return None
    
    def _point_in_collision(self, point: Position, occupied: Dict[str, Position]) -> bool:
        """Check if a point collides with any occupied position"""
        for other_pos in occupied.values():
            if point.distance_to(other_pos) < self.COLLISION_DISTANCE:
                return True
        return False
    
    def _create_path(self, start: Position, end: Position) -> MovePath:
        """Create a basic path: rotate then move forward"""
        waypoints = [start.copy()]
        
        # Calculate angle to target
        dx = end.x - start.x
        dy = end.y - start.y
        target_angle = math.degrees(math.atan2(dy, dx)) % 360
        
        # Rotation step
        waypoints.append(Position(start.x, start.y, target_angle))
        
        # Movement step
        waypoints.append(Position(end.x, end.y, target_angle))
        
        # Duration: rotation + movement
        angle_diff = abs((target_angle - start.orientation) % 360)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        rotation_time = angle_diff / 180.0
        movement_time = start.distance_to(end) / 100.0
        
        return MovePath("temp", waypoints, rotation_time + movement_time)
    
    def get_name(self) -> str:
        return "Collision-Aware Planner"

# ============================================================================
# Simulator Engine
# ============================================================================

class SimulatorEngine:
    """Manages piece simulation and movement execution"""
    
    def __init__(self):
        self.state = SimulatorState(pieces={})
        self.current_piece_path: Optional[MovePath] = None
        self.current_waypoint_index: int = 0
        self.piece_start_time: float = 0.0
        
    def initialize_board(self):
        """Initialize chess pieces at starting positions"""
        self.state.pieces = {}
        for piece_id, (col, row) in PIECE_START_POSITIONS.items():
            # Use board_coords_to_world to center pieces on their squares
            x, y = board_coords_to_world(col, row)
            pos = Position(x, y, 0)
            self.state.pieces[piece_id] = Piece(piece_id, pos, pos.copy())
    
    def randomize_positions(self):
        """Randomize piece positions on the board"""
        board_width = 8 * BOARD_SQUARE_SIZE
        board_height = 8 * BOARD_SQUARE_SIZE
        
        # Keep trying to place pieces until no collisions
        max_attempts = 100
        for attempt in range(max_attempts):
            positions = {}
            valid = True
            
            for piece_id in self.state.pieces.keys():
                for _ in range(10):  # Try up to 10 times per piece
                    x = BOARD_EXTRA_SIDE + random.uniform(0, board_width)
                    y = random.uniform(0, board_height)
                    angle = random.uniform(0, 360)
                    pos = Position(x, y, angle)
                    
                    # Check collision with existing positions
                    collision = False
                    for other_pos in positions.values():
                        if pos.distance_to(other_pos) < PIECE_RADIUS * 2 + 10:
                            collision = True
                            break
                    
                    if not collision:
                        positions[piece_id] = pos
                        break
            
            if len(positions) == len(self.state.pieces):
                for piece_id, pos in positions.items():
                    self.state.pieces[piece_id].position = pos
                return
        
        print("Warning: Could not randomize positions without collisions")
    
    def start_execution(self, paths: Dict[str, MovePath]):
        """Start executing movement paths"""
        self.state.paths = paths
        self.state.executing = True
        
        # Start with first piece that has a path
        for piece_id, path in paths.items():
            self.state.current_piece_executing = piece_id
            self.current_piece_path = path
            self.current_waypoint_index = 0
            self.piece_start_time = time.time()
            break
    
    def stop_execution(self):
        """Stop executing movements"""
        self.state.executing = False
        self.state.current_piece_executing = None
        self.current_piece_path = None
    
    def update(self, dt: float) -> bool:
        """Update simulator state. Returns True if still executing"""
        if not self.state.executing or not self.state.current_piece_executing:
            return False
        
        if not self.current_piece_path:
            return False
        
        piece_id = self.state.current_piece_executing
        piece = self.state.pieces[piece_id]
        path = self.current_piece_path
        
        # Check if we have any waypoints
        if len(path.waypoints) < 2:
            # Skip to next piece
            self._move_to_next_piece()
            return self.state.executing
        
        elapsed = time.time() - self.piece_start_time
        progress = min(1.0, elapsed / max(path.duration, 0.1))
        
        if progress >= 1.0:
            # Piece reached final waypoint, move to next piece
            piece.position = path.waypoints[-1].copy()
            self._move_to_next_piece()
            return self.state.executing
        else:
            # Calculate which segment we're on based on waypoint distances
            waypoints = path.waypoints
            
            # Calculate cumulative distances for each waypoint
            cumulative_distances = [0.0]
            for i in range(1, len(waypoints)):
                dist = waypoints[i-1].distance_to(waypoints[i])
                cumulative_distances.append(cumulative_distances[-1] + dist)
            
            total_distance = cumulative_distances[-1]
            
            if total_distance > 0:
                # Find target distance based on progress
                target_distance = total_distance * progress
                
                # Find which segment this falls into
                segment_idx = 0
                for i in range(len(cumulative_distances) - 1):
                    if target_distance >= cumulative_distances[i]:
                        segment_idx = i
                    else:
                        break
                
                # Get the start and end waypoints of this segment
                if segment_idx < len(waypoints) - 1:
                    start_wp = waypoints[segment_idx]
                    end_wp = waypoints[segment_idx + 1]
                    
                    # Calculate progress within this segment
                    segment_start_dist = cumulative_distances[segment_idx]
                    segment_end_dist = cumulative_distances[segment_idx + 1]
                    segment_length = segment_end_dist - segment_start_dist
                    
                    if segment_length > 0:
                        segment_progress = (target_distance - segment_start_dist) / segment_length
                        segment_progress = max(0.0, min(1.0, segment_progress))
                    else:
                        segment_progress = 0.0
                    
                    # Linear interpolation within segment
                    piece.position.x = start_wp.x + (end_wp.x - start_wp.x) * segment_progress
                    piece.position.y = start_wp.y + (end_wp.y - start_wp.y) * segment_progress
                    
                    # Interpolate orientation
                    piece.position.orientation = start_wp.orientation + \
                        (end_wp.orientation - start_wp.orientation) * segment_progress
        
        return True
    
    def _move_to_next_piece(self):
        """Move to the next piece that has a path"""
        if not self.state.current_piece_executing:
            return
        
        piece_ids = list(self.state.paths.keys())
        if not piece_ids:
            self.state.executing = False
            return
        
        current_index = piece_ids.index(self.state.current_piece_executing)
        
        found_next = False
        for i in range(current_index + 1, len(piece_ids)):
            next_id = piece_ids[i]
            if next_id in self.state.paths:
                self.state.current_piece_executing = next_id
                self.current_piece_path = self.state.paths[next_id]
                self.piece_start_time = time.time()
                found_next = True
                break
        
        if not found_next:
            # All pieces finished
            self.state.executing = False
    
    def check_collisions(self) -> List[Tuple[str, str]]:
        """Check for piece collisions. Returns list of colliding pairs"""
        collisions = []
        piece_ids = list(self.state.pieces.keys())
        
        for i, pid1 in enumerate(piece_ids):
            for pid2 in piece_ids[i+1:]:
                piece1 = self.state.pieces[pid1]
                piece2 = self.state.pieces[pid2]
                
                distance = piece1.position.distance_to(piece2.position)
                if distance < PIECE_RADIUS * 2:
                    collisions.append((pid1, pid2))
        
        return collisions
    
    def get_total_move_time(self) -> float:
        """Get estimated total move time for current paths"""
        if not self.state.paths:
            return 0.0
        return sum(path.duration for path in self.state.paths.values())

# ============================================================================
# UI and Rendering
# ============================================================================

class ChessRobotUI:
    """Pygame UI for the chess robot coordinator"""
    
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Chess Robot Coordinator - Simulator")
        self.clock = pygame.time.Clock()
        self.font_small = pygame.font.Font(None, 24)
        self.font_medium = pygame.font.Font(None, 32)
        self.font_large = pygame.font.Font(None, 48)
        
        self.simulator = SimulatorEngine()
        self.path_planner: PathPlanner = SequentialPathPlanner()
        self.running = True
        
        self.simulator.initialize_board()
        
        # UI state
        self.show_paths = True
        self.planner_index = 0
        self.available_planners = [SequentialPathPlanner(), CollisionAwarePathPlanner()]
        
    def get_display_position(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (mm) to display coordinates (pixels)"""
        # Account for board offset in world space (pieces placed with offset)
        # x is absolute position in mm including BOARD_EXTRA_SIDE offset
        # We need to convert to display space
        board_x = x - BOARD_EXTRA_SIDE  # Remove world offset to get board-relative position
        display_x = BOARD_DISPLAY_OFFSET_X + (board_x / BOARD_SQUARE_SIZE) * BOARD_DISPLAY_SQUARE_SIZE
        display_y = BOARD_DISPLAY_OFFSET_Y + (y / BOARD_SQUARE_SIZE) * BOARD_DISPLAY_SQUARE_SIZE
        return int(display_x), int(display_y)
    
    def draw_board(self):
        """Draw the chess board"""
        # Draw board squares
        for row in range(8):
            for col in range(8):
                x = BOARD_DISPLAY_OFFSET_X + col * BOARD_DISPLAY_SQUARE_SIZE
                y = BOARD_DISPLAY_OFFSET_Y + row * BOARD_DISPLAY_SQUARE_SIZE
                
                color = (240, 217, 181) if (row + col) % 2 == 0 else (181, 136, 99)
                pygame.draw.rect(self.screen, color, (x, y, BOARD_DISPLAY_SQUARE_SIZE, BOARD_DISPLAY_SQUARE_SIZE))
                pygame.draw.rect(self.screen, (0, 0, 0), (x, y, BOARD_DISPLAY_SQUARE_SIZE, BOARD_DISPLAY_SQUARE_SIZE), 1)
        
        # Draw coordinate labels
        for i in range(8):
            label = chr(ord('a') + i)
            x = BOARD_DISPLAY_OFFSET_X + i * BOARD_DISPLAY_SQUARE_SIZE + BOARD_DISPLAY_SQUARE_SIZE // 2
            y = BOARD_DISPLAY_OFFSET_Y + 8 * BOARD_DISPLAY_SQUARE_SIZE + 5
            text = self.font_small.render(label, True, (0, 0, 0))
            self.screen.blit(text, (x - text.get_width() // 2, y))
            
            label = str(8 - i)
            x = BOARD_DISPLAY_OFFSET_X - 25
            y = BOARD_DISPLAY_OFFSET_Y + i * BOARD_DISPLAY_SQUARE_SIZE + BOARD_DISPLAY_SQUARE_SIZE // 2
            text = self.font_small.render(label, True, (0, 0, 0))
            self.screen.blit(text, (x, y - text.get_height() // 2))
    
    def draw_pieces(self):
        """Draw all pieces"""
        for piece in self.simulator.state.pieces.values():
            display_x, display_y = self.get_display_position(piece.position.x, piece.position.y)
            
            # Draw piece circle
            color = (100, 200, 255) if piece.id.isupper() else (255, 150, 100)
            pygame.draw.circle(self.screen, color, (display_x, display_y), 15, 0)
            pygame.draw.circle(self.screen, (0, 0, 0), (display_x, display_y), 15, 2)
            
            # Draw orientation indicator (line from center)
            angle_rad = math.radians(piece.position.orientation)
            end_x = display_x + 12 * math.cos(angle_rad)
            end_y = display_y + 12 * math.sin(angle_rad)
            pygame.draw.line(self.screen, (0, 0, 0), (display_x, display_y), (end_x, end_y), 2)
            
            # Draw piece label
            label_text = self.font_small.render(piece.id, True, (0, 0, 0))
            self.screen.blit(label_text, (display_x - label_text.get_width() // 2, display_y - label_text.get_height() // 2))
    
    def draw_paths(self):
        """Draw planned movement paths as dotted lines"""
        if not self.show_paths or not self.simulator.state.paths:
            return
        
        for path in self.simulator.state.paths.values():
            waypoints = path.waypoints
            if len(waypoints) < 2:
                continue
            
            # Draw dotted line between waypoints
            for i in range(len(waypoints) - 1):
                start = waypoints[i]
                end = waypoints[i + 1]
                
                start_display = self.get_display_position(start.x, start.y)
                end_display = self.get_display_position(end.x, end.y)
                
                # Draw dotted line
                self._draw_dotted_line(start_display, end_display, (150, 150, 150), 3, 5)
            
            # Draw target position marker
            if len(waypoints) > 0:
                target = waypoints[-1]
                display_x, display_y = self.get_display_position(target.x, target.y)
                pygame.draw.circle(self.screen, (100, 255, 100), (display_x, display_y), 8, 2)
    
    def _draw_dotted_line(self, start: Tuple[int, int], end: Tuple[int, int],
                          color: Tuple[int, int, int], dot_size: int, gap: int):
        """Draw a dotted line"""
        x1, y1 = start
        x2, y2 = end
        
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if distance == 0:
            return
        
        steps = int(distance / (dot_size + gap))
        for i in range(steps):
            t = i / max(steps, 1)
            x = int(x1 + (x2 - x1) * t)
            y = int(y1 + (y2 - y1) * t)
            pygame.draw.circle(self.screen, color, (x, y), dot_size // 2)
    
    def draw_ui_buttons(self):
        """Draw UI control buttons"""
        buttons = [
            {"label": "Randomize", "action": "randomize", "x": 750, "y": 100},
            {"label": "Plan Moves", "action": "plan", "x": 750, "y": 160},
            {"label": "Execute", "action": "execute", "x": 750, "y": 220},
        ]
        
        for btn in buttons:
            rect = pygame.Rect(btn["x"], btn["y"], 150, 40)
            pygame.draw.rect(self.screen, (200, 200, 200), rect)
            pygame.draw.rect(self.screen, (0, 0, 0), rect, 2)
            
            text = self.font_small.render(btn["label"], True, (0, 0, 0))
            self.screen.blit(text, (rect.centerx - text.get_width() // 2, rect.centery - text.get_height() // 2))
            
            btn["rect"] = rect
    
    def draw_status(self):
        """Draw status information"""
        y_offset = 100
        
        # Planner name
        planner_name = self.path_planner.get_name()
        text = self.font_small.render(f"Planner: {planner_name}", True, (0, 0, 0))
        self.screen.blit(text, (750, y_offset + 300))
        
        # Execution status
        status = "Executing..." if self.simulator.state.executing else "Idle"
        status_color = (255, 0, 0) if self.simulator.state.executing else (0, 100, 0)
        text = self.font_small.render(f"Status: {status}", True, status_color)
        self.screen.blit(text, (750, y_offset + 330))
        
        # Total move time
        total_time = self.simulator.get_total_move_time()
        text = self.font_small.render(f"Move Time: {total_time:.1f}s", True, (0, 0, 0))
        self.screen.blit(text, (750, y_offset + 360))
        
        # Collision status
        collisions = self.simulator.check_collisions()
        if collisions:
            text = self.font_small.render(f"COLLISIONS: {len(collisions)}", True, (255, 0, 0))
        else:
            text = self.font_small.render("No collisions", True, (0, 100, 0))
        self.screen.blit(text, (750, y_offset + 390))
    
    def handle_input(self):
        """Handle user input"""
        mouse_pos = pygame.mouse.get_pos()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self.handle_button_click(mouse_pos)
            elif event.type == pygame.KEYDOWN:
                self.handle_key_press(event.key)
    
    def handle_button_click(self, mouse_pos: Tuple[int, int]):
        """Handle button clicks"""
        buttons = [
            {"action": "randomize", "rect": pygame.Rect(750, 100, 150, 40)},
            {"action": "plan", "rect": pygame.Rect(750, 160, 150, 40)},
            {"action": "execute", "rect": pygame.Rect(750, 220, 150, 40)},
        ]
        
        for btn in buttons:
            if btn["rect"].collidepoint(mouse_pos):
                if btn["action"] == "randomize":
                    self.simulator.randomize_positions()
                    print("Positions randomized")
                elif btn["action"] == "plan":
                    self.plan_movements()
                    print("Movements planned")
                elif btn["action"] == "execute":
                    if self.simulator.state.paths:
                        self.simulator.start_execution(self.simulator.state.paths)
                        print("Execution started")
    
    def handle_key_press(self, key: int):
        """Handle keyboard input"""
        if key == pygame.K_SPACE:
            # Toggle between planners
            self.planner_index = (self.planner_index + 1) % len(self.available_planners)
            self.path_planner = self.available_planners[self.planner_index]
            print(f"Switched to {self.path_planner.get_name()}")
        elif key == pygame.K_p:
            # Toggle path display
            self.show_paths = not self.show_paths
            print(f"Path display: {'ON' if self.show_paths else 'OFF'}")
        elif key == pygame.K_r:
            # Reset
            self.simulator.stop_execution()
            self.simulator.initialize_board()
            print("Board reset")
    
    def plan_movements(self):
        """Plan movements to start positions"""
        target_positions = {}
        for piece_id, (col, row) in PIECE_START_POSITIONS.items():
            # Use board_coords_to_world to center pieces on their target squares
            x, y = board_coords_to_world(col, row)
            target_positions[piece_id] = Position(x, y, 0)
        
        paths = self.path_planner.plan_movements(self.simulator.state.pieces, target_positions)
        self.simulator.state.paths = paths
    
    def run(self):
        """Main UI loop"""
        print("\n=== Chess Robot Coordinator ===")
        print("Controls:")
        print("  - Click 'Randomize' to randomize piece positions")
        print("  - Click 'Plan Moves' to plan movements to starting positions")
        print("  - Click 'Execute' to execute the planned movements")
        print("  - Press SPACE to switch between path planners")
        print("  - Press P to toggle path display")
        print("  - Press R to reset the board")
        print()
        
        while self.running:
            dt = self.clock.tick(60) / 1000.0
            
            self.handle_input()
            self.simulator.update(dt)
            
            # Draw everything
            self.screen.fill((255, 255, 255))
            self.draw_board()
            self.draw_paths()
            self.draw_pieces()
            self.draw_ui_buttons()
            self.draw_status()
            
            pygame.display.flip()
        
        pygame.quit()

# ============================================================================
# Testing and Benchmarking
# ============================================================================

class PathPlannerBenchmark:
    """Benchmark path planning algorithms"""
    
    def __init__(self, iterations: int = 10):
        self.iterations = iterations
        self.results = {}
    
    def benchmark_planner(self, planner: PathPlanner) -> Dict:
        """Benchmark a path planner over multiple randomizations"""
        simulator = SimulatorEngine()
        simulator.initialize_board()
        
        total_move_time = 0.0
        total_execution_time = 0.0
        collision_count = 0
        accuracy_errors = []
        
        print(f"\nBenchmarking {planner.get_name()}...")
        print(f"Running {self.iterations} iterations...")
        
        for iteration in range(self.iterations):
            # Randomize positions
            simulator.randomize_positions()
            
            # Plan movements
            target_positions = {}
            for piece_id, (col, row) in PIECE_START_POSITIONS.items():
                x = BOARD_EXTRA_SIDE + col * BOARD_SQUARE_SIZE
                y = row * BOARD_SQUARE_SIZE
                target_positions[piece_id] = Position(x, y, 0)
            
            paths = planner.plan_movements(simulator.state.pieces, target_positions)
            simulator.state.paths = paths
            
            # Execute and measure
            move_time = simulator.get_total_move_time()
            total_move_time += move_time
            
            # Simulate execution to check for collisions
            simulator.start_execution(paths)
            sim_start_time = time.time()
            
            while simulator.state.executing:
                simulator.update(0.016)  # ~60 FPS
            
            execution_time = time.time() - sim_start_time
            total_execution_time += execution_time
            
            # Check final positions
            collisions = simulator.check_collisions()
            collision_count += len(collisions)
            
            # Check accuracy
            for piece_id, piece in simulator.state.pieces.items():
                if piece_id in target_positions:
                    error = piece.position.distance_to(target_positions[piece_id])
                    accuracy_errors.append(error)
            
            if (iteration + 1) % max(1, self.iterations // 5) == 0:
                print(f"  {iteration + 1}/{self.iterations} completed")
        
        # Calculate statistics
        avg_move_time = total_move_time / self.iterations
        avg_execution_time = total_execution_time / self.iterations
        avg_accuracy = sum(accuracy_errors) / len(accuracy_errors) if accuracy_errors else 0
        collision_rate = collision_count / self.iterations
        
        results = {
            "planner": planner.get_name(),
            "iterations": self.iterations,
            "avg_move_time": avg_move_time,
            "avg_execution_time": avg_execution_time,
            "total_move_time": total_move_time,
            "collisions": collision_count,
            "collision_rate": collision_rate,
            "avg_accuracy_error": avg_accuracy,
        }
        
        return results
    
    def print_results(self, results: Dict):
        """Print benchmark results"""
        print("\n" + "=" * 60)
        print(f"Results for {results['planner']}")
        print("=" * 60)
        print(f"Iterations:           {results['iterations']}")
        print(f"Avg Move Time:        {results['avg_move_time']:.2f}s")
        print(f"Total Move Time:      {results['total_move_time']:.2f}s")
        print(f"Avg Execution Time:   {results['avg_execution_time']:.2f}s")
        print(f"Total Collisions:     {results['collisions']}")
        print(f"Collision Rate:       {results['collision_rate']:.2f}")
        print(f"Avg Accuracy Error:   {results['avg_accuracy_error']:.2f}mm")
        print("=" * 60)

# ============================================================================
# Main
# ============================================================================

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "benchmark":
        # Run benchmark mode
        iterations = int(sys.argv[2]) if len(sys.argv) > 2 else 10
        benchmark = PathPlannerBenchmark(iterations=iterations)
        
        planners = [SequentialPathPlanner(), CollisionAwarePathPlanner()]
        
        for planner in planners:
            results = benchmark.benchmark_planner(planner)
            benchmark.print_results(results)
    else:
        # Run UI mode
        ui = ChessRobotUI()
        ui.run()
