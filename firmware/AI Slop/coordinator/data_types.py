"""
Data types and classes for Chess Robot Coordinator
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
from enum import Enum


class CommandType(Enum):
    ROTATE = "rotate"
    MOVE_STRAIGHT = "move_straight"
    MOVE_ARC = "move_arc"
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
    hex_id: Optional[str] = None  # Hex address (e.g., "00", "01", etc.)
    
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
    # MOVE_ARC: target_position (Position), arc_radius (mm, positive=CW, negative=CCW)
    # WAIT: no additional params
    target_orientation: Optional[float] = None  # For ROTATE
    distance: Optional[float] = None  # For MOVE_STRAIGHT
    target_position: Optional['Position'] = None  # For MOVE_ARC
    arc_radius: Optional[float] = None  # For MOVE_ARC (positive=CW, negative=CCW)
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
