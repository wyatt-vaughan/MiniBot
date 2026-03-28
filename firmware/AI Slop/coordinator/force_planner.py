"""
Force Simulation Path Planner - Spring-based physics simulation for collision-free movement
"""

import math
from typing import Dict, List, Tuple, Optional
from .path_planner import PathPlanner
from .data_types import Piece, Position, ExecutionPlan, PieceCommandSequence
from .constants import BOARD_SQUARE_SIZE, BOARD_EXTRA_SIDE, PIECE_RADIUS


class ForceSimulationPlanner(PathPlanner):
    """
    Simulates spring forces to move pieces toward targets while avoiding collisions.

    Each piece has a spring pulling it toward its target. Pieces repel each other
    when they get close, and walls repel pieces that approach travel limits.
    Moving pieces are less affected by repulsion so they push stationary ones aside.
    After simulation, collinear steps are collapsed into single moves.
    """

    def __init__(self, time_step: float = 0.5, max_iterations: int = 5000,
                 arrival_threshold: float = 1.0, stall_threshold: float = 0.05,
                 stall_frames: int = 100):
        self.time_step = time_step            # seconds per simulation step
        self.max_iterations = max_iterations
        self.arrival_threshold = arrival_threshold  # mm - close enough to target
        self.stall_threshold = stall_threshold      # mm - minimum movement per frame to not be "stalled"
        self.stall_frames = stall_frames            # frames of no movement before giving up

        # Spring constants
        self.target_spring_k = 0.5        # pull toward target
        self.piece_repel_k = 8.0          # piece-piece repulsion (stronger than target)
        self.wall_repel_k = 12.0          # wall repulsion (strongest)

        # Repulsion distances (surface-to-surface)
        self.piece_repel_dist = 5.0       # mm - start repelling when surfaces are this close
        self.wall_repel_dist = 5.0        # mm - start repelling from walls

        # Damping to prevent oscillation
        self.damping = 0.7

        # Moving pieces get reduced repulsion effect (they push others aside)
        self.moving_repel_scale = 0.3     # multiplier on repulsion for pieces heading to target
        self.stationary_repel_scale = 1.0 # multiplier for pieces already at target

        # Board travel limits
        board_width = 8 * BOARD_SQUARE_SIZE
        board_height = 8 * BOARD_SQUARE_SIZE
        self.x_min = 0.0
        self.x_max = BOARD_EXTRA_SIDE + board_width + BOARD_EXTRA_SIDE
        self.y_min = 0.0
        self.y_max = board_height

        # Path simplification angle tolerance (degrees)
        self.collinear_angle_tolerance = 10.0

    def plan_movements(self, pieces: Dict[str, Piece],
                       target_positions: Dict[str, Position]) -> ExecutionPlan:
        # Build simulation state: positions and velocities
        sim_positions: Dict[str, List[float]] = {}
        sim_velocities: Dict[str, List[float]] = {}
        sim_history: Dict[str, List[Tuple[float, float]]] = {}

        for pid, piece in pieces.items():
            sim_positions[pid] = [piece.position.x, piece.position.y]
            sim_velocities[pid] = [0.0, 0.0]
            sim_history[pid] = [(piece.position.x, piece.position.y)]

        piece_ids = list(pieces.keys())
        target_map: Dict[str, Tuple[float, float]] = {}
        for pid, tpos in target_positions.items():
            if pid in pieces:
                target_map[pid] = (tpos.x, tpos.y)

        # Run simulation
        stall_counter = 0
        for iteration in range(self.max_iterations):
            total_movement = 0.0
            all_arrived = True

            for pid in piece_ids:
                if pid not in target_map:
                    continue

                pos = sim_positions[pid]
                vel = sim_velocities[pid]
                tx, ty = target_map[pid]

                dx_to_target = tx - pos[0]
                dy_to_target = ty - pos[1]
                dist_to_target = math.sqrt(dx_to_target ** 2 + dy_to_target ** 2)

                at_target = dist_to_target < self.arrival_threshold
                if not at_target:
                    all_arrived = False

                # --- Target spring force ---
                fx = self.target_spring_k * dx_to_target
                fy = self.target_spring_k * dy_to_target

                # Determine repulsion scale for this piece
                repel_scale = self.stationary_repel_scale if at_target else self.moving_repel_scale

                # --- Piece-piece repulsion ---
                for other_pid in piece_ids:
                    if other_pid == pid:
                        continue
                    opos = sim_positions[other_pid]
                    ddx = pos[0] - opos[0]
                    ddy = pos[1] - opos[1]
                    center_dist = math.sqrt(ddx ** 2 + ddy ** 2)
                    surface_dist = center_dist - 2 * PIECE_RADIUS

                    if surface_dist < self.piece_repel_dist and center_dist > 0.001:
                        # Overlap or close: repel
                        overlap = self.piece_repel_dist - surface_dist
                        force_mag = self.piece_repel_k * overlap * repel_scale
                        nx = ddx / center_dist
                        ny = ddy / center_dist
                        fx += force_mag * nx
                        fy += force_mag * ny

                # --- Wall repulsion ---
                # Left wall
                wall_surface = pos[0] - PIECE_RADIUS - self.x_min
                if wall_surface < self.wall_repel_dist:
                    overlap = self.wall_repel_dist - wall_surface
                    fx += self.wall_repel_k * overlap * repel_scale

                # Right wall
                wall_surface = self.x_max - (pos[0] + PIECE_RADIUS)
                if wall_surface < self.wall_repel_dist:
                    overlap = self.wall_repel_dist - wall_surface
                    fx -= self.wall_repel_k * overlap * repel_scale

                # Bottom wall
                wall_surface = pos[1] - PIECE_RADIUS - self.y_min
                if wall_surface < self.wall_repel_dist:
                    overlap = self.wall_repel_dist - wall_surface
                    fy += self.wall_repel_k * overlap * repel_scale

                # Top wall
                wall_surface = self.y_max - (pos[1] + PIECE_RADIUS)
                if wall_surface < self.wall_repel_dist:
                    overlap = self.wall_repel_dist - wall_surface
                    fy -= self.wall_repel_k * overlap * repel_scale

                # --- Integrate velocity and position ---
                vel[0] = (vel[0] + fx * self.time_step) * self.damping
                vel[1] = (vel[1] + fy * self.time_step) * self.damping

                new_x = pos[0] + vel[0] * self.time_step
                new_y = pos[1] + vel[1] * self.time_step

                # Hard clamp to bounds
                new_x = max(self.x_min + PIECE_RADIUS, min(self.x_max - PIECE_RADIUS, new_x))
                new_y = max(self.y_min + PIECE_RADIUS, min(self.y_max - PIECE_RADIUS, new_y))

                step_dist = math.sqrt((new_x - pos[0]) ** 2 + (new_y - pos[1]) ** 2)
                total_movement += step_dist

                pos[0] = new_x
                pos[1] = new_y

                # Record history
                last = sim_history[pid][-1]
                if abs(new_x - last[0]) > 0.01 or abs(new_y - last[1]) > 0.01:
                    sim_history[pid].append((new_x, new_y))

            if all_arrived:
                break

            if total_movement < self.stall_threshold:
                stall_counter += 1
                if stall_counter >= self.stall_frames:
                    break
            else:
                stall_counter = 0

        # Collapse collinear waypoints and build execution plan
        plan = ExecutionPlan()

        for pid in piece_ids:
            if pid not in target_map:
                continue

            waypoints = self._collapse_collinear(sim_history[pid])

            if len(waypoints) < 2:
                continue

            piece = pieces[pid]
            current_orientation = piece.position.orientation

            sequence = PieceCommandSequence(piece_id=pid)

            for i in range(1, len(waypoints)):
                wx, wy = waypoints[i]
                px, py = waypoints[i - 1]
                dx = wx - px
                dy = wy - py
                dist = math.sqrt(dx ** 2 + dy ** 2)

                if dist < 0.1:
                    continue

                target_angle = math.degrees(math.atan2(dy, dx)) % 360

                rotate_cmd = self._create_rotate_command(current_orientation, target_angle)
                if rotate_cmd.duration > 0.001:
                    sequence.add_command(rotate_cmd)

                move_cmd = self._create_move_command(dist)
                sequence.add_command(move_cmd)

                current_orientation = target_angle

            if sequence.commands:
                plan.add_sequence(sequence)

        return plan

    def _collapse_collinear(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Remove intermediate points that lie along a nearly straight line."""
        if len(points) <= 2:
            return list(points)

        result = [points[0]]
        i = 0

        while i < len(points) - 1:
            # Direction from current anchor to next point
            j = i + 1
            dx0 = points[j][0] - points[i][0]
            dy0 = points[j][1] - points[i][1]
            base_angle = math.atan2(dy0, dx0)

            # Extend as far as possible while staying collinear
            while j + 1 < len(points):
                dx1 = points[j + 1][0] - points[j][0]
                dy1 = points[j + 1][1] - points[j][1]
                seg_len = math.sqrt(dx1 ** 2 + dy1 ** 2)
                if seg_len < 0.001:
                    j += 1
                    continue
                next_angle = math.atan2(dy1, dx1)
                angle_diff = abs(math.degrees(next_angle - base_angle))
                angle_diff = min(angle_diff, 360 - angle_diff)
                if angle_diff <= self.collinear_angle_tolerance:
                    j += 1
                else:
                    break

            result.append(points[j])
            i = j

        return result

    def get_name(self) -> str:
        return "Force Simulation"
