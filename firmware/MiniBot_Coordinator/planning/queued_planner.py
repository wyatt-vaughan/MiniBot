"""
planning/queued_planner.py  —  MiniBot Chess Swarm Coordinator

QueuedPlanner: a sequential turn-by-turn planner.

Pieces move one at a time in ascending piece_id order.  Each move is
assigned an incrementing sequence_num so the dispatch loop can send them
with appropriate timing gaps.

This is a stub planner.  It does not check for physical collisions.
"""

from __future__ import annotations

import math
from typing import Callable, Dict, List, Optional, Tuple

from planning.base_planner import BasePlanner, MoveCommand
from config import PLANNING


class QueuedPlanner(BasePlanner):
    """Sequential planner — pieces move one after another in ID order."""

    @property
    def name(self) -> str:
        return 'Queued (sequential)'

    def plan_moves(
        self,
        piece_positions: Dict[int, Tuple[float, float]],
        targets: Dict[int, Tuple[float, float]],
        orientations: Optional[Dict[int, float]] = None,
        validator: Optional[Callable[[int, float, float], bool]] = None,
    ) -> List[MoveCommand]:
        """Generate one sequential move per targeted piece.

        Moves are assigned sequence_num values 0, 1, 2, … in ascending
        piece_id order.  The GUI / dispatcher is responsible for
        honouring sequence_num by waiting for each move to complete
        before sending the next.

        Args:
            piece_positions: Current positions keyed by piece_id.
            targets:         Desired target positions keyed by piece_id.
            orientations:    Current orientations (unused in this planner).
            validator:       Optional chess rules hook.

        Returns:
            Ordered list of MoveCommand objects with incrementing sequence_num.
        """
        commands: List[MoveCommand] = []
        seq = 0

        for piece_id in sorted(targets.keys()):
            target_x, target_y = targets[piece_id]

            # Chess engine integration point: skip illegal moves
            if validator is not None:
                if not validator(piece_id, target_x, target_y):
                    continue

            current = piece_positions.get(piece_id)
            if current is None:
                continue

            cur_x, cur_y = current
            distance    = math.hypot(target_x - cur_x, target_y - cur_y)
            duration_ms = self.duration_for_distance(distance)

            theta: Optional[float] = None
            if distance > 1.0:
                angle_rad = math.atan2(target_y - cur_y, target_x - cur_x)
                theta = (90.0 - math.degrees(angle_rad)) % 360.0

            commands.append(MoveCommand(
                piece_id     = piece_id,
                target_x_mm  = target_x,
                target_y_mm  = target_y,
                target_theta = theta,
                duration_ms  = duration_ms,
                sequence_num = seq,
            ))
            seq += 1

        return commands
