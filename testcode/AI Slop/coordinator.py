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

# Movement parameters
ANGULAR_VELOCITY = 90  # degrees per second
LINEAR_VELOCITY = 100  # mm per second

# Chess pieces: 8 pawns per side (row 1 & 6) + 8 major pieces per side (row 0 & 7)
# White pieces (bottom): pawns a2-h2, back row a1-h1
# Black pieces (top): pawns a7-h7, back row a8-h8
# Format: piece_id: (column, row)
PIECE_START_POSITIONS = {
    # White pawns (row 1)
    'p1': (0, 1), 'p2': (1, 1), 'p3': (2, 1), 'p4': (3, 1),
    'p5': (4, 1), 'p6': (5, 1), 'p7': (6, 1), 'p8': (7, 1),
    # White major pieces (row 0) - a1-h1
    'r1': (0, 0), 'n1': (1, 0), 'b1': (2, 0), 'q': (3, 0),
    'k': (4, 0), 'b2': (5, 0), 'n2': (6, 0), 'r2': (7, 0),
    # Black pawns (row 6)
    'P1': (0, 6), 'P2': (1, 6), 'P3': (2, 6), 'P4': (3, 6),
    'P5': (4, 6), 'P6': (5, 6), 'P7': (6, 6), 'P8': (7, 6),
    # Black major pieces (row 7) - a8-h8
    'R1': (0, 7), 'N1': (1, 7), 'B1': (2, 7), 'Q': (3, 7),
    'K': (4, 7), 'B2': (5, 7), 'N2': (6, 7), 'R2': (7, 7),
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

class CommandType(Enum):
    ROTATE = "rotate"
    MOVE_STRAIGHT = "move_straight"
    WAIT = "wait"

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
class PieceCommand:
    """A single command for a piece to execute"""
    command_type: CommandType
    duration: float  # seconds
    # Parameters based on command type:
    # ROTATE: target_orientation (degrees)
    # MOVE_STRAIGHT: distance (mm), direction (degrees - current orientation)
    # WAIT: no additional params
    target_orientation: Optional[float] = None  # For ROTATE
    distance: Optional[float] = None  # For MOVE_STRAIGHT
    start_time: float = 0.0  # When this command starts in the sequence

@dataclass
class PieceCommandSequence:
    """Sequence of commands for a single piece"""
    piece_id: str
    commands: List[PieceCommand] = field(default_factory=list)
    total_duration: float = 0.0
    start_time: float = 0.0  # When this sequence starts (for queuing multiple sequences)
    
    def add_command(self, command: PieceCommand):
        """Add a command to the sequence"""
        command.start_time = self.total_duration
        self.commands.append(command)
        self.total_duration += command.duration

@dataclass
class ExecutionPlan:
    """Complete execution plan for all pieces"""
    sequences: Dict[str, List[PieceCommandSequence]] = field(default_factory=dict)  # piece_id -> list of sequences
    
    def get_total_duration(self) -> float:
        """Get the maximum duration across all piece sequences"""
        if not self.sequences:
            return 0.0
        max_time = 0.0
        for seq_list in self.sequences.values():
            for seq in seq_list:
                end_time = seq.start_time + seq.total_duration
                max_time = max(max_time, end_time)
        return max_time
    
    def add_sequence(self, sequence: PieceCommandSequence):
        """Add a piece command sequence to the plan"""
        if sequence.piece_id not in self.sequences:
            self.sequences[sequence.piece_id] = []
            sequence.start_time = 0.0
        else:
            # Queue this sequence after the last one for this piece
            last_seq = self.sequences[sequence.piece_id][-1]
            sequence.start_time = last_seq.start_time + last_seq.total_duration
        
        self.sequences[sequence.piece_id].append(sequence)
    
    def add_sequence_at_time(self, sequence: PieceCommandSequence, start_time: float):
        """Add a sequence that starts at a specific time (for parallel movements)"""
        if sequence.piece_id not in self.sequences:
            self.sequences[sequence.piece_id] = []
        
        sequence.start_time = start_time
        self.sequences[sequence.piece_id].append(sequence)
        # Sort sequences by start time
        self.sequences[sequence.piece_id].sort(key=lambda s: s.start_time)

@dataclass
class SimulatorState:
    """State of the simulator"""
    pieces: Dict[str, Piece]
    execution_plan: Optional[ExecutionPlan] = None
    executing: bool = False
    execution_start_time: float = 0.0
    # Track current sequence and command index for each piece
    executing_pieces: Dict[str, Tuple[int, int]] = field(default_factory=dict)  # piece_id -> (sequence_index, command_index)


# ============================================================================
# Path Planning Interface
# ============================================================================

class PathPlanner(ABC):
    """Abstract base class for path planning algorithms"""
    
    @abstractmethod
    def plan_movements(self, pieces: Dict[str, Piece], 
                       target_positions: Dict[str, Position]) -> ExecutionPlan:
        """
        Plan movements for all pieces to reach target positions
        
        Args:
            pieces: Current pieces with their positions
            target_positions: Target positions for each piece
            
        Returns:
            ExecutionPlan containing command sequences for all pieces
        """
        pass
    
    @abstractmethod
    def get_name(self) -> str:
        """Get the name of this planner"""
        pass
    
    def _create_rotate_command(self, current_orientation: float, target_orientation: float) -> PieceCommand:
        """Helper to create a rotation command"""
        # Normalize angles
        current = current_orientation % 360
        target = target_orientation % 360
        
        # Calculate shortest rotation
        angle_diff = (target - current) % 360
        if angle_diff > 180:
            angle_diff -= 360
        
        duration = abs(angle_diff) / ANGULAR_VELOCITY
        return PieceCommand(
            command_type=CommandType.ROTATE,
            duration=duration,
            target_orientation=target
        )
    
    def _create_move_command(self, distance: float) -> PieceCommand:
        """Helper to create a straight movement command"""
        duration = distance / LINEAR_VELOCITY
        return PieceCommand(
            command_type=CommandType.MOVE_STRAIGHT,
            duration=duration,
            distance=distance
        )
    
    def _create_wait_command(self, wait_time: float) -> PieceCommand:
        """Helper to create a wait/dwell command"""
        return PieceCommand(
            command_type=CommandType.WAIT,
            duration=wait_time
        )

class AI_Planner(PathPlanner):
    """Iterative planner that tries multiple random orderings to find optimal collision-free paths"""
    
    def plan_movements(self, pieces: Dict[str, Piece], 
                       target_positions: Dict[str, Position]) -> ExecutionPlan:
        """
        Plan movements using iterative optimization with random priority.
        Pieces move one at a time, avoiding collisions with all other pieces.
        """
        best_plan = None
        best_score = float('inf')
        best_time = float('inf')
        
        # Try 100 different random orderings
        for attempt in range(20):
            plan = ExecutionPlan()
            
            # Track simulated positions for collision checking
            simulated_positions = {pid: piece.position.copy() for pid, piece in pieces.items()}
            
            # Optimize target assignments for identical pieces (with random strategy)
            optimize_chance = random.random()
            if optimize_chance < 0.7:  # 70% use optimization
                optimized_targets = self._optimize_target_assignments(pieces, target_positions, simulated_positions)
            elif optimize_chance < 0.85:  # 15% use partial random swaps
                optimized_targets = self._random_swap_targets(pieces, target_positions, simulated_positions)
            else:  # 15% use original assignments
                optimized_targets = target_positions.copy()
            
            # Calculate distances to target for prioritization
            distances = {}
            for piece_id in pieces.keys():
                if piece_id in optimized_targets:
                    dist = simulated_positions[piece_id].distance_to(optimized_targets[piece_id])
                    distances[piece_id] = dist
            
            remaining_pieces = set(distances.keys())
            current_time = 0.0
            max_iterations = 50  # Prevent infinite loops
            
            for iteration in range(max_iterations):
                if not remaining_pieces:
                    break
                
                # Calculate priority: distance + random + back rank bonus
                priorities = []
                for piece_id in remaining_pieces:
                    dist = simulated_positions[piece_id].distance_to(target_positions[piece_id])
                    
                    # Skip if already at target
                    if dist < 1.0:
                        continue
                    
                    # Back rank pieces (row 0 or 7) get priority
                    target_col, target_row = None, None
                    for pid, (col, row) in PIECE_START_POSITIONS.items():
                        if pid == piece_id:
                            target_col, target_row = col, row
                            break
                    
                    back_rank_bonus = 100.0 if target_row in [0, 7] else 0.0
                    # Vary the random factor and weighting per attempt
                    random_factor = random.uniform(0, 100)
                    distance_weight = random.uniform(0.5, 2.0)  # Vary importance of distance
                    priority = dist * distance_weight + back_rank_bonus + random_factor
                    
                    priorities.append((priority, piece_id))
                
                if not priorities:
                    break
                
                # Sort by priority (highest first)
                priorities.sort(reverse=True)
                
                # Try to move each piece in priority order
                moved_any = False
                for _, piece_id in priorities:
                    target_pos = optimized_targets[piece_id]
                    current_pos = simulated_positions[piece_id]
                    
                    # Check if already at target
                    if current_pos.distance_to(target_pos) < 1.0:
                        remaining_pieces.discard(piece_id)
                        continue
                    
                    # Try to find a collision-free path
                    sequence = self._plan_piece_movement(
                        piece_id, current_pos, target_pos, simulated_positions,
                        plan, current_time, optimized_targets, remaining_pieces
                    )
                    
                    if sequence and len(sequence.commands) > 0:
                        # Add sequence to plan at current time
                        plan.add_sequence_at_time(sequence, current_time)
                        current_time += sequence.total_duration
                        
                        # Update simulated position
                        self._update_simulated_position(simulated_positions[piece_id], sequence)
                        
                        # Check if reached target
                        if simulated_positions[piece_id].distance_to(target_pos) < 1.0:
                            remaining_pieces.discard(piece_id)
                        
                        moved_any = True
                        break  # Move to next iteration
                
                if not moved_any:
                    # No piece could move, try moving pieces out of the way
                    moved_blocking = self._try_move_blocking_pieces(
                        plan, simulated_positions, optimized_targets, 
                        remaining_pieces, current_time
                    )
                    
                    if moved_blocking:
                        # Update current time and continue
                        current_time = plan.get_total_duration()
                    else:
                        # Still stuck, break out
                        break
            
            # Score this attempt
            total_time = plan.get_total_duration()
            accuracy_score = sum(
                simulated_positions[pid].distance_to(optimized_targets[pid])
                for pid in optimized_targets.keys()
            )
            
            # Perfect accuracy = pieces at target, otherwise prefer better accuracy
            if accuracy_score < 10.0:  # Near perfect
                score = total_time
            else:
                score = accuracy_score * 1000 + total_time
            
            if attempt % 10 == 0:
                print(f"  Attempt {attempt}: time={total_time:.1f}s, accuracy={accuracy_score:.1f}mm, score={score:.1f}")
            
            # Keep best solution
            if score < best_score:
                best_score = score
                best_time = total_time
                best_plan = plan
        
        print(f"Best solution: time={best_time:.1f}s, score={best_score:.1f}")
        return best_plan if best_plan else ExecutionPlan()
    
    def _optimize_target_assignments(self, pieces: Dict[str, Piece], 
                                     target_positions: Dict[str, Position],
                                     simulated_positions: Dict[str, Position]) -> Dict[str, Position]:
        """Optimize target assignments for interchangeable pieces"""
        optimized = target_positions.copy()
        
        # Group pieces by type
        piece_groups = {
            'white_pawns': [],
            'black_pawns': [],
            'white_rooks': [],
            'black_rooks': [],
            'white_knights': [],
            'black_knights': [],
            'white_bishops': [],
            'black_bishops': [],
        }
        
        target_groups = {
            'white_pawns': [],
            'black_pawns': [],
            'white_rooks': [],
            'black_rooks': [],
            'white_knights': [],
            'black_knights': [],
            'white_bishops': [],
            'black_bishops': [],
        }
        
        # Classify pieces and their targets
        for piece_id in pieces.keys():
            if piece_id not in target_positions:
                continue
            
            # White pawns: p1-p8
            if piece_id.startswith('p') and piece_id[1:].isdigit():
                piece_groups['white_pawns'].append(piece_id)
                target_groups['white_pawns'].append(target_positions[piece_id])
            # Black pawns: P1-P8
            elif piece_id.startswith('P') and piece_id[1:].isdigit():
                piece_groups['black_pawns'].append(piece_id)
                target_groups['black_pawns'].append(target_positions[piece_id])
            # White rooks: r1, r2
            elif piece_id in ['r1', 'r2']:
                piece_groups['white_rooks'].append(piece_id)
                target_groups['white_rooks'].append(target_positions[piece_id])
            # Black rooks: R1, R2
            elif piece_id in ['R1', 'R2']:
                piece_groups['black_rooks'].append(piece_id)
                target_groups['black_rooks'].append(target_positions[piece_id])
            # White knights: n1, n2
            elif piece_id in ['n1', 'n2']:
                piece_groups['white_knights'].append(piece_id)
                target_groups['white_knights'].append(target_positions[piece_id])
            # Black knights: N1, N2
            elif piece_id in ['N1', 'N2']:
                piece_groups['black_knights'].append(piece_id)
                target_groups['black_knights'].append(target_positions[piece_id])
            # White bishops: b1, b2
            elif piece_id in ['b1', 'b2']:
                piece_groups['white_bishops'].append(piece_id)
                target_groups['white_bishops'].append(target_positions[piece_id])
            # Black bishops: B1, B2
            elif piece_id in ['B1', 'B2']:
                piece_groups['black_bishops'].append(piece_id)
                target_groups['black_bishops'].append(target_positions[piece_id])
        
        # Optimize each group
        for group_name in piece_groups.keys():
            piece_ids = piece_groups[group_name]
            targets = target_groups[group_name]
            
            if len(piece_ids) <= 1:
                continue
            
            # Find optimal assignment using greedy matching
            assignments = self._find_optimal_assignment(piece_ids, targets, simulated_positions)
            
            for piece_id, target_pos in assignments.items():
                optimized[piece_id] = target_pos
        
        return optimized
    
    def _find_optimal_assignment(self, piece_ids: List[str], targets: List[Position],
                                simulated_positions: Dict[str, Position]) -> Dict[str, Position]:
        """Find optimal target assignment to minimize total distance"""
        import itertools
        
        if len(piece_ids) != len(targets):
            # Shouldn't happen, but handle gracefully
            return {pid: targets[i] for i, pid in enumerate(piece_ids)}
        
        best_assignment = {}
        best_total_distance = float('inf')
        
        # Try all permutations to find minimum total distance
        for target_permutation in itertools.permutations(targets):
            total_distance = 0
            for i, piece_id in enumerate(piece_ids):
                distance = simulated_positions[piece_id].distance_to(target_permutation[i])
                total_distance += distance
            
            if total_distance < best_total_distance:
                best_total_distance = total_distance
                best_assignment = {piece_ids[i]: target_permutation[i] for i in range(len(piece_ids))}
        
        return best_assignment
    
    def _random_swap_targets(self, pieces: Dict[str, Piece],
                            target_positions: Dict[str, Position],
                            simulated_positions: Dict[str, Position]) -> Dict[str, Position]:
        """Randomly swap some targets for identical pieces (exploratory strategy)"""
        optimized = target_positions.copy()
        
        # Get piece groups
        piece_groups = {
            'white_pawns': [],
            'black_pawns': [],
            'white_rooks': [],
            'black_rooks': [],
            'white_knights': [],
            'black_knights': [],
            'white_bishops': [],
            'black_bishops': [],
        }
        
        target_groups = {
            'white_pawns': [],
            'black_pawns': [],
            'white_rooks': [],
            'black_rooks': [],
            'white_knights': [],
            'black_knights': [],
            'white_bishops': [],
            'black_bishops': [],
        }
        
        # Classify pieces
        for piece_id in pieces.keys():
            if piece_id not in target_positions:
                continue
            
            if piece_id.startswith('p') and piece_id[1:].isdigit():
                piece_groups['white_pawns'].append(piece_id)
                target_groups['white_pawns'].append(target_positions[piece_id])
            elif piece_id.startswith('P') and piece_id[1:].isdigit():
                piece_groups['black_pawns'].append(piece_id)
                target_groups['black_pawns'].append(target_positions[piece_id])
            elif piece_id in ['r1', 'r2']:
                piece_groups['white_rooks'].append(piece_id)
                target_groups['white_rooks'].append(target_positions[piece_id])
            elif piece_id in ['R1', 'R2']:
                piece_groups['black_rooks'].append(piece_id)
                target_groups['black_rooks'].append(target_positions[piece_id])
            elif piece_id in ['n1', 'n2']:
                piece_groups['white_knights'].append(piece_id)
                target_groups['white_knights'].append(target_positions[piece_id])
            elif piece_id in ['N1', 'N2']:
                piece_groups['black_knights'].append(piece_id)
                target_groups['black_knights'].append(target_positions[piece_id])
            elif piece_id in ['b1', 'b2']:
                piece_groups['white_bishops'].append(piece_id)
                target_groups['white_bishops'].append(target_positions[piece_id])
            elif piece_id in ['B1', 'B2']:
                piece_groups['black_bishops'].append(piece_id)
                target_groups['black_bishops'].append(target_positions[piece_id])
        
        # Randomly shuffle targets within each group
        for group_name in piece_groups.keys():
            piece_ids = piece_groups[group_name]
            targets = target_groups[group_name]
            
            if len(piece_ids) <= 1:
                continue
            
            # Shuffle targets
            shuffled_targets = targets.copy()
            random.shuffle(shuffled_targets)
            
            for i, piece_id in enumerate(piece_ids):
                optimized[piece_id] = shuffled_targets[i]
        
        return optimized
    
    def _plan_piece_movement(self, piece_id: str, current_pos: Position, 
                            target_pos: Position, 
                            simulated_positions: Dict[str, Position],
                            plan: ExecutionPlan,
                            current_time: float,
                            target_positions: Dict[str, Position],
                            remaining_pieces: set) -> Optional[PieceCommandSequence]:
        """Plan movement for a single piece, checking for collisions"""
        
        # Randomly choose strategy order (50% try direct first, 50% try route-around first)
        try_route_first = random.random() < 0.3
        
        if not try_route_first:
            # Try direct path first
            sequence = self._try_direct_path(piece_id, current_pos, target_pos, simulated_positions)
            if sequence:
                return sequence
        else:
            # Try routing around first (more exploratory)
            sequence = self._try_route_around(piece_id, current_pos, target_pos, simulated_positions)
            if sequence:
                return sequence
            # Fall back to direct
            sequence = self._try_direct_path(piece_id, current_pos, target_pos, simulated_positions)
            if sequence:
                return sequence
            return None
        
        # Direct path is blocked - identify what's blocking
        blocker_id = self._find_blocking_piece(piece_id, current_pos, target_pos, simulated_positions)
        
        if blocker_id and blocker_id not in remaining_pieces:
            # The blocker has already reached its target, try to move it out of the way
            if self._try_move_single_blocker(blocker_id, piece_id, plan, simulated_positions, 
                                            target_positions, current_time):
                # Try direct path again after moving blocker
                sequence = self._try_direct_path(piece_id, current_pos, target_pos, simulated_positions)
                if sequence:
                    return sequence
        
        elif blocker_id and blocker_id in remaining_pieces:
            # The blocker also needs to move - try to move it toward its target first
            blocker_target = target_positions.get(blocker_id)
            if blocker_target:
                blocker_pos = simulated_positions[blocker_id]
                blocker_sequence = self._try_direct_path(blocker_id, blocker_pos, blocker_target, simulated_positions)
                
                if blocker_sequence and len(blocker_sequence.commands) > 0:
                    # Move the blocker first
                    plan.add_sequence_at_time(blocker_sequence, current_time)
                    self._update_simulated_position(simulated_positions[blocker_id], blocker_sequence)
                    
                    # Check if blocker reached its target
                    if simulated_positions[blocker_id].distance_to(blocker_target) < 1.0:
                        remaining_pieces.discard(blocker_id)
                    
                    # Try our path again
                    sequence = self._try_direct_path(piece_id, current_pos, target_pos, simulated_positions)
                    if sequence:
                        return sequence
        
        # If still blocked, try routing around obstacles
        sequence = self._try_route_around(piece_id, current_pos, target_pos, simulated_positions)
        if sequence:
            return sequence
        
        return None
    
    def _try_direct_path(self, piece_id: str, current_pos: Position,
                        target_pos: Position,
                        simulated_positions: Dict[str, Position]) -> Optional[PieceCommandSequence]:
        """Try a direct path to target"""
        sequence = PieceCommandSequence(piece_id=piece_id)
        
        # Calculate angle to target
        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 1.0:
            return None
        
        target_angle = math.degrees(math.atan2(dy, dx)) % 360
        
        # Add rotation command
        rotate_cmd = self._create_rotate_command(current_pos.orientation, target_angle)
        sequence.add_command(rotate_cmd)
        
        # Check for collisions along the path
        max_safe_distance = self._find_max_safe_distance(
            current_pos, target_angle, distance, piece_id, simulated_positions
        )
        
        if max_safe_distance > 1.0:
            # Move as far as safely possible
            move_cmd = self._create_move_command(max_safe_distance)
            sequence.add_command(move_cmd)
            return sequence
        
        return None
    
    def _try_route_around(self, piece_id: str, current_pos: Position,
                         target_pos: Position,
                         simulated_positions: Dict[str, Position]) -> Optional[PieceCommandSequence]:
        """Try to route around obstacles by going to an intermediate waypoint"""
        sequence = PieceCommandSequence(piece_id=piece_id)
        
        # Try several intermediate waypoints around the obstacle
        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y
        direct_distance = math.sqrt(dx*dx + dy*dy)
        
        if direct_distance < 1.0:
            return None
        
        # Try waypoints at different angles offset from the direct path
        best_waypoint_sequence = None
        best_waypoint_progress = 0.0
        
        # Randomize which angles to try and in what order
        angle_options = [45, -45, 90, -90, 135, -135]
        random.shuffle(angle_options)
        
        for angle_offset in angle_options:
            # Calculate waypoint perpendicular to direct path
            mid_x = current_pos.x + dx * 0.5
            mid_y = current_pos.y + dy * 0.5
            
            # Offset perpendicular to path
            perp_angle = math.atan2(dy, dx) + math.radians(angle_offset)
            offset_distance = min(100, direct_distance * 0.5)  # 100mm or half distance
            
            waypoint_x = mid_x + offset_distance * math.cos(perp_angle)
            waypoint_y = mid_y + offset_distance * math.sin(perp_angle)
            
            # Check if waypoint is valid
            if not self._is_position_valid(waypoint_x, waypoint_y, piece_id, simulated_positions):
                continue
            
            # Try path to waypoint
            waypoint_pos = Position(waypoint_x, waypoint_y, 0)
            
            # Calculate path to waypoint
            wp_dx = waypoint_x - current_pos.x
            wp_dy = waypoint_y - current_pos.y
            wp_distance = math.sqrt(wp_dx*wp_dx + wp_dy*wp_dy)
            wp_angle = math.degrees(math.atan2(wp_dy, wp_dx)) % 360
            
            # Check if we can reach waypoint
            safe_dist_to_wp = self._find_max_safe_distance(
                current_pos, wp_angle, wp_distance, piece_id, simulated_positions
            )
            
            if safe_dist_to_wp > wp_distance * 0.9:  # Can reach at least 90% of the way
                # Calculate progress toward goal this waypoint gives us
                progress = safe_dist_to_wp / direct_distance
                
                if progress > best_waypoint_progress:
                    test_sequence = PieceCommandSequence(piece_id=piece_id)
                    rotate_cmd = self._create_rotate_command(current_pos.orientation, wp_angle)
                    test_sequence.add_command(rotate_cmd)
                    move_cmd = self._create_move_command(safe_dist_to_wp)
                    test_sequence.add_command(move_cmd)
                    
                    best_waypoint_sequence = test_sequence
                    best_waypoint_progress = progress
        
        return best_waypoint_sequence
    
    def _find_max_safe_distance(self, start_pos: Position, angle: float, 
                               max_distance: float, moving_piece_id: str,
                               simulated_positions: Dict[str, Position]) -> float:
        """Find maximum distance piece can move without collision"""
        angle_rad = math.radians(angle)
        safe_distance = max_distance
        
        # Check collision with every other piece
        for other_id, other_pos in simulated_positions.items():
            if other_id == moving_piece_id:
                continue
            
            # Check multiple points along the path
            for test_dist in np.linspace(0, max_distance, 20):
                test_x = start_pos.x + test_dist * math.cos(angle_rad)
                test_y = start_pos.y + test_dist * math.sin(angle_rad)
                
                dx = test_x - other_pos.x
                dy = test_y - other_pos.y
                dist_to_other = math.sqrt(dx*dx + dy*dy)
                
                if dist_to_other < PIECE_RADIUS * 2 + 5:  # 5mm safety margin
                    safe_distance = min(safe_distance, test_dist - 10)  # Stop before collision
                    break
        
        return max(0, safe_distance)
    
    def _update_simulated_position(self, pos: Position, sequence: PieceCommandSequence):
        """Update a simulated position based on command sequence"""
        for command in sequence.commands:
            if command.command_type == CommandType.ROTATE:
                pos.orientation = command.target_orientation
            elif command.command_type == CommandType.MOVE_STRAIGHT:
                angle_rad = math.radians(pos.orientation)
                pos.x += command.distance * math.cos(angle_rad)
                pos.y += command.distance * math.sin(angle_rad)
    
    def _find_blocking_piece(self, piece_id: str, start_pos: Position, target_pos: Position,
                            simulated_positions: Dict[str, Position]) -> Optional[str]:
        """Find which piece is blocking the direct path"""
        dx = target_pos.x - start_pos.x
        dy = target_pos.y - start_pos.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 1.0:
            return None
        
        angle = math.degrees(math.atan2(dy, dx))
        angle_rad = math.radians(angle)
        
        # Check for pieces along the path
        closest_blocker = None
        closest_dist = float('inf')
        
        for other_id, other_pos in simulated_positions.items():
            if other_id == piece_id:
                continue
            
            # Check multiple points along the path
            for test_t in np.linspace(0.05, 1.0, 20):
                test_x = start_pos.x + distance * test_t * math.cos(angle_rad)
                test_y = start_pos.y + distance * test_t * math.sin(angle_rad)
                
                dx_to_other = test_x - other_pos.x
                dy_to_other = test_y - other_pos.y
                dist_to_other = math.sqrt(dx_to_other*dx_to_other + dy_to_other*dy_to_other)
                
                if dist_to_other < PIECE_RADIUS * 2 + 5:
                    # This piece is blocking
                    path_dist = distance * test_t
                    if path_dist < closest_dist:
                        closest_dist = path_dist
                        closest_blocker = other_id
                    break
        
        return closest_blocker
    
    def _try_move_single_blocker(self, blocker_id: str, blocked_id: str, plan: ExecutionPlan,
                                 simulated_positions: Dict[str, Position],
                                 target_positions: Dict[str, Position],
                                 current_time: float) -> bool:
        """Try to move a single blocking piece out of the way"""
        blocker_pos = simulated_positions[blocker_id]
        blocked_pos = simulated_positions[blocked_id]
        blocked_target = target_positions[blocked_id]
        
        # Calculate perpendicular direction to move blocker
        path_angle = math.atan2(blocked_target.y - blocked_pos.y, blocked_target.x - blocked_pos.x)
        
        # Randomize escape angles
        perp_offsets = [90, -90, 45, -45, 135, -135]
        random.shuffle(perp_offsets)
        
        for perp_offset in perp_offsets:
            move_angle = path_angle + math.radians(perp_offset)
            move_distance = random.uniform(80, 120)
            
            temp_x = blocker_pos.x + move_distance * math.cos(move_angle)
            temp_y = blocker_pos.y + move_distance * math.sin(move_angle)
            
            if self._is_position_valid(temp_x, temp_y, blocker_id, simulated_positions):
                temp_pos = Position(temp_x, temp_y, 0)
                sequence = self._try_direct_path(
                    blocker_id, blocker_pos, temp_pos, simulated_positions
                )
                
                if sequence and len(sequence.commands) > 0:
                    plan.add_sequence_at_time(sequence, current_time)
                    self._update_simulated_position(simulated_positions[blocker_id], sequence)
                    print(f"  Moved blocker {blocker_id} out of way for {blocked_id}")
                    return True
        
        return False
    
    def _try_move_blocking_pieces(self, plan: ExecutionPlan, 
                                  simulated_positions: Dict[str, Position],
                                  target_positions: Dict[str, Position],
                                  remaining_pieces: set, 
                                  current_time: float) -> bool:
        """Try to move pieces that are blocking others out of the way"""
        
        # Find pieces that are blocking the path of remaining pieces
        blocking_info = []
        
        for blocked_id in remaining_pieces:
            blocked_pos = simulated_positions[blocked_id]
            target_pos = target_positions[blocked_id]
            
            # Calculate direct path
            dx = target_pos.x - blocked_pos.x
            dy = target_pos.y - blocked_pos.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 1.0:
                continue
            
            target_angle = math.degrees(math.atan2(dy, dx))
            angle_rad = math.radians(target_angle)
            
            # Check which pieces are in the way
            for other_id, other_pos in simulated_positions.items():
                if other_id == blocked_id or other_id in remaining_pieces:
                    continue
                
                # Check if this piece is near the path
                for test_t in np.linspace(0.1, 0.9, 9):
                    test_x = blocked_pos.x + distance * test_t * math.cos(angle_rad)
                    test_y = blocked_pos.y + distance * test_t * math.sin(angle_rad)
                    
                    dist_to_blocker = math.sqrt((test_x - other_pos.x)**2 + (test_y - other_pos.y)**2)
                    
                    if dist_to_blocker < PIECE_RADIUS * 2 + 20:  # Within blocking distance
                        blocking_info.append((other_id, blocked_id, dist_to_blocker))
                        break
        
        # Try to move blocking pieces
        if blocking_info:
            # Sort by how much they're blocking (closest blockers first)
            blocking_info.sort(key=lambda x: x[2])
            
            for blocker_id, blocked_id, _ in blocking_info[:3]:  # Try top 3 blockers
                blocker_pos = simulated_positions[blocker_id]
                
                # Try to move blocker to a safe position away from the path
                blocked_pos = simulated_positions[blocked_id]
                target_pos = target_positions[blocked_id]
                
                # Calculate perpendicular direction to move blocker
                path_angle = math.atan2(target_pos.y - blocked_pos.y, target_pos.x - blocked_pos.x)
                
                for perp_offset in [90, -90, 45, -45, 135, -135]:
                    move_angle = path_angle + math.radians(perp_offset)
                    move_distance = random.uniform(80, 120)
                    
                    temp_x = blocker_pos.x + move_distance * math.cos(move_angle)
                    temp_y = blocker_pos.y + move_distance * math.sin(move_angle)
                    
                    if self._is_position_valid(temp_x, temp_y, blocker_id, simulated_positions):
                        temp_pos = Position(temp_x, temp_y, 0)
                        sequence = self._try_direct_path(
                            blocker_id, blocker_pos, temp_pos, simulated_positions
                        )
                        
                        if sequence and len(sequence.commands) > 0:
                            plan.add_sequence_at_time(sequence, current_time)
                            self._update_simulated_position(simulated_positions[blocker_id], sequence)
                            print(f"  Moved blocker {blocker_id} out of way for {blocked_id}")
                            return True
        
        return False
    
    def _is_position_valid(self, x: float, y: float, piece_id: str,
                          simulated_positions: Dict[str, Position]) -> bool:
        """Check if a position is on the board and collision-free"""
        # Check board bounds
        board_width = 8 * BOARD_SQUARE_SIZE
        board_height = 8 * BOARD_SQUARE_SIZE
        
        if x < BOARD_EXTRA_SIDE or x > BOARD_EXTRA_SIDE + board_width:
            return False
        if y < 0 or y > board_height:
            return False
        
        # Check collisions
        for other_id, other_pos in simulated_positions.items():
            if other_id == piece_id:
                continue
            
            dx = x - other_pos.x
            dy = y - other_pos.y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < PIECE_RADIUS * 2 + 5:
                return False
        
        return True
    
    def get_name(self) -> str:
        return "AI Planner V1"

class OutsmartingAIPlannerV1(PathPlanner):
    """Simple sequential planner with differential drive physics"""
    
    def plan_movements(self, pieces: Dict[str, Piece], 
                       target_positions: Dict[str, Position]) -> ExecutionPlan:
        """
        Plan movements using differential drive physics.
        Pieces can only move forward/backward in the direction they're facing.
        """
        paths = {}
        
        # Create a list of distance to target for each piece

        # Start a loop to iterate path planning attempts 100 times

            # Initialize var to store total move time for this iteration

            # Start a second loop for multiple iterations through pieces to move to target position

                # Calculate priority based on distance to target, back rank or not, and a random value. Skip pieces already at target.

                # Loop through pieces in priority order
                
                    # Plan piece waypoints avoiding collisions with all other pieces

                    # If no path found, move as close as possible to target without collision

                    # If path to target found without collisions, add to paths and update locally saved piece position. Always rotate to face next waypoint, then move. Then rotate then move, and so on.

                    # Calculate time taken for this piece and add to running total

                # Check if all pieces have reached their targets. Break if yes, otherwise continue this loop.

            # Store total move time and score move accuracy. Print these for debugging.

        # Select the best iteration's paths (fastest total time if perfect accuracy, otherwise best accuracy) and return those paths.

        return paths
    
    def get_name(self) -> str:
        return "SuperDuper Anti-AI Planner V1"

class SequentialPathPlanner(PathPlanner):
    """Simple sequential planner with command-based execution"""
    
    def plan_movements(self, pieces: Dict[str, Piece],
                       target_positions: Dict[str, Position]) -> ExecutionPlan:
        """
        Plan movements using command sequences.
        All pieces execute in parallel (no collision avoidance).
        """
        plan = ExecutionPlan()
        
        for piece_id, target_pos in target_positions.items():
            if piece_id not in pieces:
                continue
                
            piece = pieces[piece_id]
            sequence = PieceCommandSequence(piece_id=piece_id)
            
            # Calculate angle to target
            dx = target_pos.x - piece.position.x
            dy = target_pos.y - piece.position.y
            
            if abs(dx) < 0.1 and abs(dy) < 0.1:
                # Already at target
                continue
            
            target_angle = math.degrees(math.atan2(dy, dx)) % 360
            
            # Command 1: Rotate to face target
            rotate_cmd = self._create_rotate_command(piece.position.orientation, target_angle)
            sequence.add_command(rotate_cmd)
            
            # Command 2: Move straight to target
            distance = piece.position.distance_to(target_pos)
            move_cmd = self._create_move_command(distance)
            sequence.add_command(move_cmd)
            
            plan.add_sequence(sequence)
        
        return plan
    
    def get_name(self) -> str:
        return "Sequential Planner"


# ============================================================================
# Simulator Engine
# ============================================================================

class SimulatorEngine:
    """Manages piece simulation and movement execution"""
    
    def __init__(self):
        self.state = SimulatorState(pieces={})
        # Track starting positions for each command
        self.command_start_positions: Dict[str, Position] = {}
        
    def initialize_board(self):
        """Initialize chess pieces at starting positions"""
        self.state.pieces = {}
        for piece_id, (col, row) in PIECE_START_POSITIONS.items():
            # Use board_coords_to_world to center pieces on their squares
            x, y = board_coords_to_world(col, row)
            pos = Position(x, y, 0)
            self.state.pieces[piece_id] = Piece(piece_id, pos, pos.copy())
    
    def randomize_positions(self):
        """Randomize piece positions on the board ensuring no collisions"""
        board_width = 8 * BOARD_SQUARE_SIZE
        board_height = 8 * BOARD_SQUARE_SIZE
        min_distance = PIECE_RADIUS * 2 + 10
        
        positions = {}
        
        for piece_id in self.state.pieces.keys():
            max_attempts = 100
            placed = False
            
            for attempt in range(max_attempts):
                x = BOARD_EXTRA_SIDE + random.uniform(0, board_width)
                y = random.uniform(0, board_height)
                angle = random.uniform(0, 360)
                pos = Position(x, y, angle)
                
                # Check collision with all previously placed pieces
                collision = False
                for other_pos in positions.values():
                    if pos.distance_to(other_pos) < min_distance:
                        collision = True
                        break
                
                if not collision:
                    positions[piece_id] = pos
                    placed = True
                    break
            
            if not placed:
                # If we can't place this piece, replace a random existing piece
                if positions:
                    replaced_id = random.choice(list(positions.keys()))
                    del positions[replaced_id]
                    # Try again for this piece
                    for attempt in range(max_attempts):
                        x = BOARD_EXTRA_SIDE + random.uniform(0, board_width)
                        y = random.uniform(0, board_height)
                        angle = random.uniform(0, 360)
                        pos = Position(x, y, angle)
                        
                        collision = False
                        for other_pos in positions.values():
                            if pos.distance_to(other_pos) < min_distance:
                                collision = True
                                break
                        
                        if not collision:
                            positions[piece_id] = pos
                            break
        
        # Apply positions
        for piece_id, pos in positions.items():
            self.state.pieces[piece_id].position = pos
    
    def start_execution(self, plan: ExecutionPlan):
        """Start executing the execution plan"""
        self.state.execution_plan = plan
        self.state.executing = True
        self.state.execution_start_time = time.time()
        
        # Store start positions for visualization
        for piece_id, piece in self.state.pieces.items():
            piece.start_position = piece.position.copy()
        
        # Initialize execution tracking for each piece
        self.state.executing_pieces = {}
        self.command_start_positions = {}  # Clear old command tracking
        if plan and plan.sequences:
            for piece_id in plan.sequences.keys():
                self.state.executing_pieces[piece_id] = (0, 0)  # (sequence_index, command_index)
    
    def stop_execution(self):
        """Stop executing movements"""
        self.state.executing = False
        self.state.execution_plan = None
        self.state.executing_pieces = {}
    
    def update(self, dt: float) -> bool:
        """Update simulator state. Returns True if still executing"""
        if not self.state.executing or not self.state.execution_plan:
            return False
        
        plan = self.state.execution_plan
        elapsed = time.time() - self.state.execution_start_time
        
        any_executing = False
        
        # Update each piece
        for piece_id, sequences in plan.sequences.items():
            if piece_id not in self.state.executing_pieces:
                continue
            
            piece = self.state.pieces[piece_id]
            seq_idx, cmd_idx = self.state.executing_pieces[piece_id]
            
            # Check if this piece is done
            if seq_idx >= len(sequences):
                continue
            
            sequence = sequences[seq_idx]
            
            # Check if this sequence should have started yet
            if elapsed < sequence.start_time:
                any_executing = True
                continue
            
            # Check if done with all commands in this sequence
            if cmd_idx >= len(sequence.commands):
                # Move to next sequence
                seq_idx += 1
                cmd_idx = 0
                self.state.executing_pieces[piece_id] = (seq_idx, cmd_idx)
                
                if seq_idx < len(sequences):
                    any_executing = True
                continue
            
            command = sequence.commands[cmd_idx]
            time_in_sequence = elapsed - sequence.start_time
            command_start = command.start_time
            command_end = command.start_time + command.duration
            
            # Check if command should have started
            if time_in_sequence < command_start:
                any_executing = True
                continue
            
            # Check if command is complete
            if time_in_sequence >= command_end:
                # Clear the cached start position for this completed command
                cmd_key = f"{piece_id}_seq{seq_idx}_cmd{cmd_idx}"
                if cmd_key in self.command_start_positions:
                    del self.command_start_positions[cmd_key]
                
                # Execute final state of command only if we haven't moved to next yet
                self._apply_command_final_state(piece, command)
                
                # Move to next command
                cmd_idx += 1
                self.state.executing_pieces[piece_id] = (seq_idx, cmd_idx)
                any_executing = True
                continue
            
            # Command is in progress - interpolate
            progress = (time_in_sequence - command_start) / max(command.duration, 0.001)
            progress = min(1.0, max(0.0, progress))
            
            self._apply_command_progress(piece, command, progress, seq_idx, cmd_idx)
            any_executing = True
        
        if not any_executing:
            self.state.executing = False
            self.state.execution_plan = None  # Clear the plan to remove drawn paths
        
        return self.state.executing
    
    def _apply_command_final_state(self, piece: Piece, command: PieceCommand):
        """Apply the final state of a completed command"""
        # Note: This is only called when transitioning to the next command
        # The piece should already be at or very close to the final position from interpolation
        # This ensures exact final state regardless of timing precision
        if command.command_type == CommandType.ROTATE:
            piece.position.orientation = command.target_orientation
        elif command.command_type == CommandType.MOVE_STRAIGHT:
            # The position should already be set by _apply_command_progress at 100%
            # Just ensure orientation is correct
            pass
        # WAIT does nothing to position
    
    def _apply_command_progress(self, piece: Piece, command: PieceCommand, progress: float, seq_idx: int, cmd_idx: int):
        """Apply partial progress of a command"""
        # Get or store the start position for this command
        # Use sequence and command index to uniquely identify each command execution
        cmd_key = f"{piece.id}_seq{seq_idx}_cmd{cmd_idx}"
        
        if cmd_key not in self.command_start_positions:
            self.command_start_positions[cmd_key] = piece.position.copy()
        
        start_pos = self.command_start_positions[cmd_key]
        
        if command.command_type == CommandType.ROTATE:
            # Interpolate rotation
            start_orientation = start_pos.orientation
            target_orientation = command.target_orientation
            
            # Calculate shortest rotation path
            angle_diff = (target_orientation - start_orientation) % 360
            if angle_diff > 180:
                angle_diff -= 360
            
            piece.position.orientation = (start_orientation + angle_diff * progress) % 360
            
        elif command.command_type == CommandType.MOVE_STRAIGHT:
            # Move in the direction the piece is facing
            angle_rad = math.radians(piece.position.orientation)
            distance_traveled = command.distance * progress
            
            piece.position.x = start_pos.x + distance_traveled * math.cos(angle_rad)
            piece.position.y = start_pos.y + distance_traveled * math.sin(angle_rad)
        # WAIT does nothing
    
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
        """Get estimated total move time for current execution plan"""
        if not self.state.execution_plan:
            return 0.0
        return self.state.execution_plan.get_total_duration()

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
        self.available_planners = [
            SequentialPathPlanner(),
            AI_Planner(),
        ]
        
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
        if not self.show_paths or not self.simulator.state.execution_plan:
            return
        
        plan = self.simulator.state.execution_plan
        for piece_id, sequences in plan.sequences.items():
            if piece_id not in self.simulator.state.pieces:
                continue
            
            piece = self.simulator.state.pieces[piece_id]
            # Start from current position when planning, or start_position during execution
            if self.simulator.state.executing and piece.start_position:
                current_pos = piece.start_position.copy()
            else:
                current_pos = piece.position.copy()
            
            for sequence in sequences:
                for command in sequence.commands:
                    if command.command_type == CommandType.ROTATE:
                        # Draw rotation arc indicator
                        display_pos = self.get_display_position(current_pos.x, current_pos.y)
                        pygame.draw.circle(self.screen, (200, 150, 100), display_pos, 10, 1)
                        current_pos.orientation = command.target_orientation
                        
                    elif command.command_type == CommandType.MOVE_STRAIGHT:
                        # Draw movement line
                        start_display = self.get_display_position(current_pos.x, current_pos.y)
                        
                        angle_rad = math.radians(current_pos.orientation)
                        end_x = current_pos.x + command.distance * math.cos(angle_rad)
                        end_y = current_pos.y + command.distance * math.sin(angle_rad)
                        end_display = self.get_display_position(end_x, end_y)
                        
                        self._draw_dotted_line(start_display, end_display, (150, 150, 150), 3, 5)
                        
                        current_pos.x = end_x
                        current_pos.y = end_y
            
            # Draw final target marker
            display_x, display_y = self.get_display_position(current_pos.x, current_pos.y)
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
                    if self.simulator.state.execution_plan:
                        self.simulator.start_execution(self.simulator.state.execution_plan)
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
        
        plan = self.path_planner.plan_movements(self.simulator.state.pieces, target_positions)
        self.simulator.state.execution_plan = plan
    
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
        
        planners = [
            SequentialPathPlanner(),
        ]
        
        for planner in planners:
            results = benchmark.benchmark_planner(planner)
            benchmark.print_results(results)
    else:
        # Run UI mode
        ui = ChessRobotUI()
        ui.run()
