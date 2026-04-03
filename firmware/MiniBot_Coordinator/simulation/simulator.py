"""
simulation/simulator.py  —  MiniBot Chess Swarm Coordinator

Software-in-the-loop motion simulator.

When simulator mode is active in the GUI, MoveCommands are passed here
instead of being sent over serial.  A QTimer drives a motion tick at
SIMULATOR.UPDATE_INTERVAL_MS.  Each tick advances every active piece
toward its target at a configurable speed, then:

  1. Boundary enforcement: piece centre is clamped to the table limits
     (playing area + all four border margins), such that the piece body
     never crosses the outer hard line.

  2. Piece–piece collision detection: if the desired new position would
     place a piece closer than (2 × radius + margin) to any other piece,
     the move for that piece is blocked for this tick (the piece holds
     its current position).  Static (non-moving) pieces are checked
     against a snapshot taken at the start of the tick; this avoids
     order-dependency within a single tick.

Signals:
  position_updated(int, float, float, float)  — id, x_mm, y_mm, theta_deg
  move_complete(int)                           — id when a piece reaches target
  log_message(str)                             — human-readable event string
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from PyQt6.QtCore import QObject, QTimer, pyqtSignal, pyqtSlot

from config import PIECES, SIMULATOR
from models.piece import BoardState
from planning.base_planner import MoveCommand


class MotionSimulator(QObject):
    """Simulates robot motion for all active MoveCommands.

    Thread safety:
        Intended to run on the GUI thread (driven by QTimer).
        All calls to queue_moves() must also be on the GUI thread.
    """

    position_updated = pyqtSignal(int, float, float, float)  # id, x, y, theta
    move_complete    = pyqtSignal(int)                        # id
    log_message      = pyqtSignal(str)

    def __init__(
        self,
        board_state: BoardState,
        speed_mm_s: float = SIMULATOR.DEFAULT_SPEED_MM_S,
        parent: Optional[QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._board  = board_state
        self._speed  = speed_mm_s
        self._active: Dict[int, MoveCommand] = {}  # piece_id → command

        self._timer = QTimer(self)
        self._timer.setInterval(SIMULATOR.UPDATE_INTERVAL_MS)
        self._timer.timeout.connect(self._tick)

        # Boundary constants (piece centre limits)
        r = float(PIECES.CIRCLE_RADIUS_MM)
        m = float(SIMULATOR.COLLISION_MARGIN_MM)
        self._x_min = float(SIMULATOR.X_MIN_MM) + r
        self._x_max = float(SIMULATOR.X_MAX_MM) - r
        self._y_min = float(SIMULATOR.Y_MIN_MM) + r
        self._y_max = float(SIMULATOR.Y_MAX_MM) - r
        self._collision_dist = 2.0 * r + m

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def speed_mm_s(self) -> float:
        return self._speed

    @speed_mm_s.setter
    def speed_mm_s(self, value: float) -> None:
        self._speed = max(1.0, value)

    def queue_moves(self, commands: List[MoveCommand]) -> None:
        """Accept a list of MoveCommands to simulate.

        Commands are keyed by piece_id; a new command for an already-moving
        piece replaces the previous target immediately.
        """
        for cmd in commands:
            self._active[cmd.piece_id] = cmd
        if self._active and not self._timer.isActive():
            self._timer.start()

    def stop_all(self) -> None:
        """Cancel all active moves and stop the timer."""
        self._active.clear()
        self._timer.stop()

    @property
    def is_running(self) -> bool:
        return self._timer.isActive()

    # ------------------------------------------------------------------
    # Timer tick
    # ------------------------------------------------------------------

    @pyqtSlot()
    def _tick(self) -> None:
        dt       = SIMULATOR.UPDATE_INTERVAL_MS / 1000.0
        step     = self._speed * dt
        rot_step = SIMULATOR.ROTATION_SPEED_DEG_S * dt

        # Snapshot of all NON-moving piece positions for collision checks.
        snapshot: Dict[int, Tuple[float, float]] = {}
        for piece in self._board.active_pieces():
            if piece.piece_id not in self._active:
                snapshot[piece.piece_id] = (piece.x_mm, piece.y_mm)

        arrived_ids: List[int] = []

        for pid, cmd in list(self._active.items()):
            piece = self._board.get_piece(pid)
            if piece is None or piece.is_captured:
                arrived_ids.append(pid)
                continue

            cur_x, cur_y   = piece.x_mm, piece.y_mm
            dx              = cmd.target_x_mm - cur_x
            dy              = cmd.target_y_mm - cur_y
            dist            = math.hypot(dx, dy)
            at_position     = dist <= step

            # ── Heading logic (differential drive) ───────────────────────
            # travel_theta: direction the robot must face to move toward target.
            # final_theta:  desired orientation once at the target position.
            if dist > 1.0:
                travel_theta = (90.0 - math.degrees(math.atan2(dy, dx))) % 360.0
            else:
                travel_theta = piece.orientation_deg  # don't spin if basically there

            final_theta = (cmd.target_theta % 360.0) if cmd.target_theta is not None \
                          else travel_theta

            # While still travelling → rotate to face target.
            # Once at position → rotate to final_theta.
            rot_target     = final_theta if at_position else travel_theta
            heading_diff   = (rot_target - piece.orientation_deg + 180.0) % 360.0 - 180.0

            if abs(heading_diff) <= rot_step:
                new_theta = rot_target
            else:
                new_theta = (piece.orientation_deg + math.copysign(rot_step, heading_diff)) % 360.0

            # ── Translation (only once heading is sufficiently aligned) ──
            # Check against travel_theta (not rot_target) so in-place final
            # rotation after arrival doesn't mistakenly move the piece.
            travel_diff = (travel_theta - piece.orientation_deg + 180.0) % 360.0 - 180.0
            heading_ok  = abs(travel_diff) <= SIMULATOR.HEADING_TOLERANCE_DEG

            if at_position:
                new_x, new_y = cmd.target_x_mm, cmd.target_y_mm
            elif heading_ok:
                # Move forward along the current heading (not a free-direction
                # move): project the step onto the travel direction.
                new_x = cur_x + (dx / dist) * step
                new_y = cur_y + (dy / dist) * step
            else:
                # Still rotating to face target — hold position
                new_x, new_y = cur_x, cur_y

            # ── 1. Boundary enforcement (clamp to table limits) ──────────
            clamped_x = max(self._x_min, min(self._x_max, new_x))
            clamped_y = max(self._y_min, min(self._y_max, new_y))
            if (clamped_x, clamped_y) != (new_x, new_y):
                self.log_message.emit(
                    f'SIM: 0x{pid:02X} boundary clamped '
                    f'({new_x:.1f},{new_y:.1f})→({clamped_x:.1f},{clamped_y:.1f})'
                )
                new_x, new_y = clamped_x, clamped_y

            # ── 2. Piece–piece collision check (vs. static snapshot) ─────
            blocked = False
            for other_pid, (ox, oy) in snapshot.items():
                if math.hypot(new_x - ox, new_y - oy) < self._collision_dist:
                    self.log_message.emit(
                        f'SIM: 0x{pid:02X} blocked by 0x{other_pid:02X}'
                    )
                    blocked = True
                    break

            if blocked:
                # Hold position this tick but allow in-place rotation
                new_x, new_y = cur_x, cur_y

            # ── Arrival check ─────────────────────────────────────────────
            # Arrived when spatial target reached AND final orientation settled.
            pos_done = math.hypot(new_x - cmd.target_x_mm, new_y - cmd.target_y_mm) <= 0.5
            rot_done = abs((final_theta - new_theta + 180.0) % 360.0 - 180.0) < 0.5
            if pos_done and rot_done:
                arrived_ids.append(pid)

            # ── Commit to board state and emit ───────────────────────────
            self._board.update_piece_position(pid, new_x, new_y, new_theta)
            self.position_updated.emit(pid, new_x, new_y, new_theta)

        # Clean up finished moves
        for pid in arrived_ids:
            self._active.pop(pid, None)
            self.move_complete.emit(pid)
            self.log_message.emit(f'SIM: 0x{pid:02X} arrived at target')

        if not self._active:
            self._timer.stop()
