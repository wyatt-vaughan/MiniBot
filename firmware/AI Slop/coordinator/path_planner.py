"""
Base PathPlanner class and helper functions
"""

import math
from abc import ABC, abstractmethod
from typing import Dict
from .data_types import Piece, Position, PieceCommand, ExecutionPlan, CommandType
from .constants import ANGULAR_VELOCITY, LINEAR_VELOCITY


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
    
    def _create_arc_command(self, current_pos: Position, target_pos: Position, arc_radius: float) -> PieceCommand:
        """
        Helper to create an arc movement command
        
        Args:
            current_pos: Current position with orientation
            target_pos: Target position to reach
            arc_radius: Radius of arc in mm (positive=CW, negative=CCW)
            
        Returns:
            PieceCommand for arc movement
            
        The arc starts tangent to the current orientation and curves to reach the target.
        The final orientation is calculated based on the arc geometry.
        """
        # Calculate the straight-line distance from start to end
        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y
        chord_length = math.sqrt(dx*dx + dy*dy)
        
        # Calculate arc length using the relationship between chord and radius
        # For a circular arc: chord = 2 * |radius| * sin(theta/2)
        # where theta is the central angle in radians
        abs_radius = abs(arc_radius)
        
        if chord_length > 2 * abs_radius:
            # Chord is too long for this radius - use straight line as fallback
            return self._create_move_command(chord_length)
        
        # Calculate central angle (theta)
        # sin(theta/2) = chord / (2 * radius)
        sin_half_theta = chord_length / (2 * abs_radius)
        sin_half_theta = min(1.0, max(-1.0, sin_half_theta))  # Clamp to valid range
        half_theta = math.asin(sin_half_theta)
        theta = 2 * half_theta  # radians
        
        # Arc length = radius * theta
        arc_length = abs_radius * theta
        
        # Calculate duration based on arc length
        duration = arc_length / LINEAR_VELOCITY
        
        return PieceCommand(
            command_type=CommandType.MOVE_ARC,
            duration=duration,
            target_position=target_pos.copy(),
            arc_radius=arc_radius
        )
