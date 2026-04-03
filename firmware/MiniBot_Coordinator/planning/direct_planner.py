"""
planning/direct_planner.py  —  MiniBot Chess Swarm Coordinator

DirectPlanner: the simplest possible path planner.

Each piece moves in a straight line from its current position to its target.
All moves are assigned the same sequence_num (they execute concurrently).
Duration is estimated from straight-line distance.

This is a stub planner.  It does not check for collisions or chess legality
beyond calling the optional validator hook.
"""

from __future__ import annotations

import math
from typing import Callable, Dict, List, Optional, Tuple

from planning.base_planner import BasePlanner, MoveCommand
from config import PLANNING


class DirectPlanner(BasePlanner):
    """Straight-line planner — all pieces move simultaneously."""

    @property
    def name(self) -> str:
        return 'Direct (straight-line)'

    def plan_moves(
        self,
        piece_positions: Dict[int, Tuple[float, float]],
        targets: Dict[int, Tuple[float, float]],
        orientations: Optional[Dict[int, float]] = None,
        validator: Optional[Callable[[int, float, float], bool]] = None,
    ) -> List[MoveCommand]:
        """Generate one concurrent move per targeted piece.

        All moves share sequence_num=0 (parallel execution).

        Args:
            piece_positions: Current positions keyed by piece_id.
            targets:         Desired target positions keyed by piece_id.
            orientations:    Current orientations (unused in this planner).
            validator:       Optional chess rules hook.

        Returns:
            List of MoveCommand, one per targeted piece.
        """
        commands: List[MoveCommand] = []

        for piece_id, (target_x, target_y) in targets.items():
            # Chess engine integration point: skip illegal moves
            if validator is not None:
                if not validator(piece_id, target_x, target_y):
                    continue

            current = piece_positions.get(piece_id)
            if current is None:
                # Piece not tracked; skip
                continue

            cur_x, cur_y = current
            distance = math.hypot(target_x - cur_x, target_y - cur_y)
            duration_ms = self.duration_for_distance(distance)

            # Compute facing direction toward target
            theta: Optional[float] = None
            if distance > 1.0:  # don't rotate for micro-moves
                angle_rad = math.atan2(target_y - cur_y, target_x - cur_x)
                # Convert atan2 (0=right, CCW positive) to robot convention
                # (0=up/+Y, CW positive from robot's perspective)
                theta = (90.0 - math.degrees(angle_rad)) % 360.0

            commands.append(MoveCommand(
                piece_id     = piece_id,
                target_x_mm  = target_x,
                target_y_mm  = target_y,
                target_theta = theta,
                duration_ms  = duration_ms,
                sequence_num = 0,
            ))

        return commands
