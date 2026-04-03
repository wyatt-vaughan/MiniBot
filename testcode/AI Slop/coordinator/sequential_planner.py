"""
Sequential Path Planner - Simple parallel movement without collision avoidance
"""

import math
from typing import Dict
from .path_planner import PathPlanner
from .data_types import Piece, Position, ExecutionPlan, PieceCommandSequence


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
