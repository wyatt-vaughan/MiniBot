"""
planning/wyatt_planner.py — MiniBot Chess Swarm Coordinator

WyattPlanner is the public setup-oriented swarm planner.  Internally this is a setup-oriented
swarm planner intended for:

* standard starting-position setup,
* arbitrary puzzle setup,
* dense or loosely piled starting positions,
* staging unused pieces out of the way, and
* moving independent pieces in parallel waves.

Planning pipeline
-----------------
1. Validate the requested target layout.
2. Generate a lane-aware temporary staging layout.
3. Decompress every physical piece from the pile into staging.  Dense-start
   moves are allowed only when they never reduce separation from an already
   too-close neighbour.
4. Compute access-preserving placement layers by virtually removing pieces
   from the completed position back to their assigned staging locations.
5. Reverse those layers and route pieces from staging to their final targets.
6. Group mutually safe moves into parallel waves.
7. Independently audit every emitted command before returning it.

This planner deliberately fails closed: it raises PlanningError rather than
returning an incomplete or unaudited physical plan.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Set, Tuple

from config import PIECES, PLANNING, SIMULATOR
from planning.base_planner import BasePlanner, MoveCommand


Vec2 = Tuple[float, float]
Validator = Callable[[int, float, float], bool]


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

_RADIUS_MM = float(PIECES.CIRCLE_RADIUS_MM)
_CLEARANCE_MM = 2.0 * _RADIUS_MM + float(
    getattr(PLANNING, "WYATT_SAFETY_MARGIN_MM", 2.0)
)
_ARRIVAL_EPS_MM = float(getattr(PLANNING, "WYATT_ARRIVAL_EPS_MM", 2.0))
_GRID_MM = float(getattr(PLANNING, "WYATT_GRID_MM", 12.5))
_VALIDATOR_SAMPLE_MM = float(
    getattr(PLANNING, "WYATT_VALIDATOR_SAMPLE_MM", 5.0)
)
_STAGE_SLOT_GAP_MM = float(
    getattr(PLANNING, "WYATT_STAGE_SLOT_GAP_MM", 4.0)
)
_STAGE_AISLE_EXTRA_MM = float(
    getattr(PLANNING, "WYATT_STAGE_AISLE_EXTRA_MM", 4.0)
)
_STAGE_BANK_SIZE = int(getattr(PLANNING, "WYATT_STAGE_BANK_SIZE", 2))
_MAX_PARALLEL_MOVERS = int(
    getattr(PLANNING, "WYATT_MAX_PARALLEL_MOVERS", 8)
)
_MAX_DECOMPRESSION_ROUNDS = int(
    getattr(PLANNING, "WYATT_MAX_DECOMPRESSION_ROUNDS", 250)
)
_MAX_LAYER_WAVES = int(getattr(PLANNING, "WYATT_MAX_LAYER_WAVES", 500))
_MAX_ASTAR_EXPANSIONS = int(
    getattr(PLANNING, "WYATT_MAX_ASTAR_EXPANSIONS", 50_000)
)
_ASTAR_HEURISTIC_WEIGHT = float(
    getattr(PLANNING, "WYATT_ASTAR_HEURISTIC_WEIGHT", 1.15)
)
_MAX_PARALLEL_ASTAR_ATTEMPTS = int(
    getattr(PLANNING, "WYATT_MAX_PARALLEL_ASTAR_ATTEMPTS", 12)
)
_REVERSE_PATH_BRANCH_LIMIT = int(
    getattr(PLANNING, "WYATT_REVERSE_PATH_BRANCH_LIMIT", 64)
)
_REVERSE_SEARCH_NODE_LIMIT = int(
    getattr(PLANNING, "WYATT_REVERSE_SEARCH_NODE_LIMIT", 20_000)
)

_EPS = 1e-9


class PlanningError(RuntimeError):
    """Raised when a complete, audited setup plan cannot be produced."""


@dataclass(frozen=True)
class _Move:
    piece_id: int
    start: Vec2
    end: Vec2
    duration_ms: int
    note: str


@dataclass
class _PlanState:
    positions: Dict[int, Vec2]
    commands: List[MoveCommand]
    sequence_num: int = 0

    def emit_wave(self, wave: Sequence[_Move]) -> None:
        if not wave:
            return

        moving_ids = set()
        for move in wave:
            if move.piece_id in moving_ids:
                raise PlanningError(
                    f"Piece {move.piece_id} appears twice in one movement wave"
                )
            moving_ids.add(move.piece_id)

            actual_start = self.positions.get(move.piece_id)
            if actual_start is None:
                raise PlanningError(f"Unknown piece ID {move.piece_id}")
            if _dist(actual_start, move.start) > 0.5:
                raise PlanningError(
                    f"Planner state mismatch for piece {move.piece_id}: "
                    f"expected {actual_start}, move starts at {move.start}"
                )

            self.commands.append(
                MoveCommand(
                    piece_id=move.piece_id,
                    target_x_mm=move.end[0],
                    target_y_mm=move.end[1],
                    duration_ms=move.duration_ms,
                    sequence_num=self.sequence_num,
                    planner_debug=move.note,
                )
            )

        for move in wave:
            self.positions[move.piece_id] = move.end

        self.sequence_num += 1


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


def _dist_sq(a: Vec2, b: Vec2) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


def _dist(a: Vec2, b: Vec2) -> float:
    return math.sqrt(_dist_sq(a, b))


def _dot(a: Vec2, b: Vec2) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _sub(a: Vec2, b: Vec2) -> Vec2:
    return a[0] - b[0], a[1] - b[1]


def _add(a: Vec2, b: Vec2) -> Vec2:
    return a[0] + b[0], a[1] + b[1]


def _scale(a: Vec2, value: float) -> Vec2:
    return a[0] * value, a[1] * value


class _ObstacleIndex:
    """Small spatial hash used by A* collision checks.

    A board has only a few dozen robots, but A* can test thousands of short
    edges. Looking at nearby obstacle buckets avoids rescanning every robot for
    every edge.
    """

    def __init__(self, obstacles: Dict[int, Vec2]) -> None:
        self.cell_size = max(_CLEARANCE_MM, 1.0)
        self.cells: Dict[Tuple[int, int], List[Tuple[int, Vec2]]] = {}
        for piece_id, point in obstacles.items():
            key = self._cell(point)
            self.cells.setdefault(key, []).append((piece_id, point))

    def _cell(self, point: Vec2) -> Tuple[int, int]:
        return (
            int(math.floor(point[0] / self.cell_size)),
            int(math.floor(point[1] / self.cell_size)),
        )

    def near_segment(self, start: Vec2, end: Vec2) -> Iterable[Tuple[int, Vec2]]:
        margin = _CLEARANCE_MM
        min_x = int(math.floor((min(start[0], end[0]) - margin) / self.cell_size))
        max_x = int(math.floor((max(start[0], end[0]) + margin) / self.cell_size))
        min_y = int(math.floor((min(start[1], end[1]) - margin) / self.cell_size))
        max_y = int(math.floor((max(start[1], end[1]) + margin) / self.cell_size))
        for cell_x in range(min_x, max_x + 1):
            for cell_y in range(min_y, max_y + 1):
                yield from self.cells.get((cell_x, cell_y), ())


def _pt_seg_dist_sq(point: Vec2, start: Vec2, end: Vec2) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    denom = dx * dx + dy * dy
    if denom <= _EPS:
        return _dist_sq(point, start)

    t = (
        (point[0] - start[0]) * dx
        + (point[1] - start[1]) * dy
    ) / denom
    t = max(0.0, min(1.0, t))
    closest = start[0] + t * dx, start[1] + t * dy
    return _dist_sq(point, closest)


def _in_bounds(point: Vec2) -> bool:
    return (
        float(SIMULATOR.X_MIN_MM) + _RADIUS_MM <= point[0]
        <= float(SIMULATOR.X_MAX_MM) - _RADIUS_MM
        and float(SIMULATOR.Y_MIN_MM) + _RADIUS_MM <= point[1]
        <= float(SIMULATOR.Y_MAX_MM) - _RADIUS_MM
    )


def _segment_validator_ok(
    piece_id: int,
    start: Vec2,
    end: Vec2,
    validator: Optional[Validator],
) -> bool:
    if not _in_bounds(start) or not _in_bounds(end):
        return False
    if validator is None:
        return True

    length = _dist(start, end)
    samples = max(1, int(math.ceil(length / _VALIDATOR_SAMPLE_MM)))
    for index in range(samples + 1):
        t = index / samples
        x = start[0] + (end[0] - start[0]) * t
        y = start[1] + (end[1] - start[1]) * t
        if not validator(piece_id, x, y):
            return False
    return True


def _segment_clear_static(
    piece_id: int,
    start: Vec2,
    end: Vec2,
    obstacles: Dict[int, Vec2],
    validator: Optional[Validator],
) -> bool:
    if not _segment_validator_ok(piece_id, start, end, validator):
        return False

    clearance_sq = _CLEARANCE_MM * _CLEARANCE_MM
    for other_id, other in obstacles.items():
        if other_id == piece_id:
            continue
        if _pt_seg_dist_sq(other, start, end) < clearance_sq - _EPS:
            return False
    return True


def _decompression_segment_clear(
    piece_id: int,
    start: Vec2,
    end: Vec2,
    obstacles: Dict[int, Vec2],
    validator: Optional[Validator],
    require_clear_endpoint: bool,
) -> bool:
    """Collision check that safely permits escape from an over-dense pile.

    A neighbour already closer than normal clearance is allowed only when the
    proposed motion never decreases the separation.  The squared distance to a
    stationary neighbour is convex along a line segment; a non-negative
    derivative at t=0 therefore guarantees it never decreases later.
    """

    if not _segment_validator_ok(piece_id, start, end, validator):
        return False

    move_vec = _sub(end, start)
    clearance_sq = _CLEARANCE_MM * _CLEARANCE_MM

    for other_id, other in obstacles.items():
        if other_id == piece_id:
            continue

        start_delta = _sub(start, other)
        start_sq = _dist_sq(start, other)
        end_sq = _dist_sq(end, other)

        if start_sq < clearance_sq - _EPS:
            # Derivative of squared distance at t=0.
            if _dot(start_delta, move_vec) < -_EPS:
                return False
            if end_sq <= start_sq + 1.0:
                return False
            if require_clear_endpoint and end_sq < clearance_sq - _EPS:
                return False
        elif _pt_seg_dist_sq(other, start, end) < clearance_sq - _EPS:
            return False

    return True


def _position_during(move: _Move, time_ms: float) -> Vec2:
    if move.duration_ms <= 0 or time_ms >= move.duration_ms:
        return move.end
    if time_ms <= 0:
        return move.start
    t = time_ms / move.duration_ms
    return (
        move.start[0] + (move.end[0] - move.start[0]) * t,
        move.start[1] + (move.end[1] - move.start[1]) * t,
    )


def _velocity_during(move: _Move, time_ms: float) -> Vec2:
    if move.duration_ms <= 0 or time_ms >= move.duration_ms:
        return 0.0, 0.0
    return (
        (move.end[0] - move.start[0]) / move.duration_ms,
        (move.end[1] - move.start[1]) / move.duration_ms,
    )


def _timed_min_distance_sq(a: _Move, b: _Move) -> float:
    """Exact minimum center distance while two same-wave moves execute.

    Each robot moves linearly, then remains at its endpoint until the slower
    robot finishes.  Relative motion is linear between duration breakpoints,
    so the minimum on each interval has a closed-form solution.
    """

    end_time = float(max(a.duration_ms, b.duration_ms))
    breakpoints = sorted(
        {
            0.0,
            float(min(a.duration_ms, end_time)),
            float(min(b.duration_ms, end_time)),
            end_time,
        }
    )

    best = float("inf")
    for left, right in zip(breakpoints, breakpoints[1:]):
        if right < left + _EPS:
            continue

        pa = _position_during(a, left)
        pb = _position_during(b, left)
        relative = _sub(pa, pb)

        va = _velocity_during(a, left + _EPS)
        vb = _velocity_during(b, left + _EPS)
        relative_velocity = _sub(va, vb)

        interval = right - left
        speed_sq = _dot(relative_velocity, relative_velocity)
        if speed_sq <= _EPS:
            tau = 0.0
        else:
            tau = -_dot(relative, relative_velocity) / speed_sq
            tau = max(0.0, min(interval, tau))

        closest = _add(relative, _scale(relative_velocity, tau))
        best = min(best, _dot(closest, closest))

        # Guard against numerical issues at the right boundary.
        pa_right = _position_during(a, right)
        pb_right = _position_during(b, right)
        best = min(best, _dist_sq(pa_right, pb_right))

    if not breakpoints or end_time <= _EPS:
        return _dist_sq(a.end, b.end)
    return best


def _moves_mutually_safe(a: _Move, b: _Move, relaxed_start: bool) -> bool:
    minimum_sq = _timed_min_distance_sq(a, b)
    initial_sq = _dist_sq(a.start, b.start)
    final_sq = _dist_sq(a.end, b.end)
    clearance_sq = _CLEARANCE_MM * _CLEARANCE_MM

    if relaxed_start and initial_sq < clearance_sq - _EPS:
        # They begin too close.  Never permit them to become even closer, and
        # require the wave to finish with normal clearance restored.
        return (
            minimum_sq >= initial_sq - 1.0
            and final_sq >= clearance_sq - _EPS
        )

    return minimum_sq >= clearance_sq - _EPS


# ---------------------------------------------------------------------------
# Planner
# ---------------------------------------------------------------------------


class WyattPlanner(BasePlanner):
    """Setup-oriented multi-piece swarm planner."""

    @property
    def name(self) -> str:
        return "Wyatt Planner"

    def plan_moves(
        self,
        piece_positions: Dict[int, Tuple[float, float]],
        targets: Dict[int, Tuple[float, float]],
        orientations: Optional[Dict[int, float]] = None,
        validator: Optional[Validator] = None,
        skip_target_optimisation: bool = False,
    ) -> List[MoveCommand]:
        del orientations, skip_target_optimisation

        if not piece_positions:
            return []

        initial_positions: Dict[int, Vec2] = {
            pid: (float(point[0]), float(point[1]))
            for pid, point in piece_positions.items()
        }
        goals: Dict[int, Vec2] = {
            pid: (float(point[0]), float(point[1]))
            for pid, point in targets.items()
        }

        unknown_targets = sorted(set(goals) - set(initial_positions))
        if unknown_targets:
            raise PlanningError(
                "Targets reference unknown piece IDs: "
                + ", ".join(str(pid) for pid in unknown_targets)
            )

        self._validate_target_layout(goals, validator)

        state = _PlanState(
            positions=dict(initial_positions),
            commands=[],
        )

        # Stage every physical piece. Pieces absent from targets remain in their
        # staging slots as puzzle-storage pieces.
        staging_slots = self._generate_staging_slots(
            count=len(initial_positions),
            goals=goals,
        )
        staged_positions = self._decompress_to_staging(
            state=state,
            goals=goals,
            available_slots=staging_slots,
            validator=validator,
        )

        required_ids = set(goals)
        unused_ids = set(initial_positions) - required_ids
        fixed_storage = {
            pid: staged_positions[pid]
            for pid in unused_ids
        }

        placement_layers = self._compute_placement_layers(
            required_ids=required_ids,
            staged_positions=staged_positions,
            goals=goals,
            fixed_storage=fixed_storage,
            validator=validator,
        )

        for layer_index, layer in enumerate(placement_layers):
            self._route_layer(
                state=state,
                layer=layer,
                goals=goals,
                validator=validator,
                layer_index=layer_index,
            )

        compacted_commands = self._compact_parallel_waves(
            initial_positions=initial_positions,
            commands=state.commands,
            validator=validator,
        )
        self._audit_plan(
            initial_positions=initial_positions,
            goals=goals,
            commands=compacted_commands,
            validator=validator,
        )
        return compacted_commands

    # ------------------------------------------------------------------
    # Input validation
    # ------------------------------------------------------------------

    def _validate_target_layout(
        self,
        goals: Dict[int, Vec2],
        validator: Optional[Validator],
    ) -> None:
        for pid, goal in goals.items():
            if not _in_bounds(goal):
                raise PlanningError(
                    f"Target for piece {pid} is outside legal bounds: {goal}"
                )
            if validator is not None and not validator(pid, goal[0], goal[1]):
                raise PlanningError(
                    f"Target for piece {pid} is rejected by the validator: {goal}"
                )

        clearance_sq = _CLEARANCE_MM * _CLEARANCE_MM
        for (pid_a, goal_a), (pid_b, goal_b) in itertools.combinations(
            goals.items(), 2
        ):
            if _dist_sq(goal_a, goal_b) < clearance_sq - _EPS:
                raise PlanningError(
                    f"Targets for pieces {pid_a} and {pid_b} are too close: "
                    f"{_dist(goal_a, goal_b):.1f} mm < {_CLEARANCE_MM:.1f} mm"
                )

    # ------------------------------------------------------------------
    # Lane-aware staging
    # ------------------------------------------------------------------

    @staticmethod
    def _axis_bank_positions(low: float, high: float) -> List[float]:
        slot_pitch = _CLEARANCE_MM + _STAGE_SLOT_GAP_MM
        aisle_pitch = 2.0 * _CLEARANCE_MM + _STAGE_AISLE_EXTRA_MM
        value = low + _RADIUS_MM + 2.0
        maximum = high - _RADIUS_MM - 2.0

        result: List[float] = []
        index_in_bank = 0
        while value <= maximum + _EPS:
            result.append(value)
            index_in_bank += 1
            if index_in_bank >= _STAGE_BANK_SIZE:
                value += aisle_pitch
                index_in_bank = 0
            else:
                value += slot_pitch
        return result

    def _generate_staging_slots(
        self,
        count: int,
        goals: Dict[int, Vec2],
    ) -> List[Vec2]:
        xs = self._axis_bank_positions(
            float(SIMULATOR.X_MIN_MM),
            float(SIMULATOR.X_MAX_MM),
        )
        ys = self._axis_bank_positions(
            float(SIMULATOR.Y_MIN_MM),
            float(SIMULATOR.Y_MAX_MM),
        )

        candidates = [(x, y) for y in ys for x in xs if _in_bounds((x, y))]
        if not candidates:
            raise PlanningError("No legal staging positions exist inside the bounds")

        target_points = list(goals.values())
        center = (
            (float(SIMULATOR.X_MIN_MM) + float(SIMULATOR.X_MAX_MM)) / 2.0,
            (float(SIMULATOR.Y_MIN_MM) + float(SIMULATOR.Y_MAX_MM)) / 2.0,
        )

        def target_clearance(point: Vec2) -> float:
            if not target_points:
                return float("inf")
            return min(_dist(point, goal) for goal in target_points)

        # Prefer slots outside final target footprints and farther from the
        # target-dense region.  The bank pattern already leaves wide aisles.
        hard_clear = [
            point
            for point in candidates
            if target_clearance(point) >= _CLEARANCE_MM
        ]
        soft_clear = [point for point in candidates if point not in hard_clear]

        hard_clear.sort(
            key=lambda point: (
                target_clearance(point),
                _dist(point, center),
            ),
            reverse=True,
        )
        soft_clear.sort(
            key=lambda point: (
                target_clearance(point),
                _dist(point, center),
            ),
            reverse=True,
        )

        ordered = hard_clear + soft_clear
        if len(ordered) < count:
            raise PlanningError(
                f"Need {count} staging slots but only {len(ordered)} fit. "
                "Increase the simulator movement bounds or reduce the configured "
                "staging spacing."
            )
        return ordered

    def _assign_staging_slots(
        self,
        positions: Dict[int, Vec2],
        goals: Dict[int, Vec2],
        slots: Sequence[Vec2],
        validator: Optional[Validator],
    ) -> Dict[int, Vec2]:
        """Globally assign unique staging slots with a rectangular Hungarian solve.

        The dominant penalty prevents a piece from parking inside another
        piece's final target clearance. A slot near its *own* target is allowed,
        which avoids the common two-piece staging swap deadlock.
        """
        piece_ids = sorted(positions)
        if len(slots) < len(piece_ids):
            raise PlanningError(
                f"Need {len(piece_ids)} staging slots, found {len(slots)}"
            )

        costs: List[List[float]] = []
        for pid in piece_ids:
            row: List[float] = []
            start = positions[pid]
            own_goal = goals.get(pid)
            for slot in slots:
                cost = _dist(start, slot)
                if validator is not None and not validator(pid, slot[0], slot[1]):
                    row.append(1.0e12)
                    continue

                if own_goal is not None:
                    cost += 0.35 * _dist(slot, own_goal)
                    for other_id, other_goal in goals.items():
                        if other_id == pid:
                            continue
                        gap = _dist(slot, other_goal)
                        if gap < _CLEARANCE_MM:
                            cost += 100_000.0 + (_CLEARANCE_MM - gap) * 10_000.0
                else:
                    # Unused puzzle pieces become fixed storage obstacles.
                    # Keep them well away from every final target.
                    for other_goal in goals.values():
                        gap = _dist(slot, other_goal)
                        if gap < _CLEARANCE_MM:
                            cost += 1_000_000.0 + (_CLEARANCE_MM - gap) * 10_000.0
                        else:
                            cost -= min(gap, _CLEARANCE_MM * 5.0) * 0.25
                row.append(cost)
            costs.append(row)

        assignment = self._hungarian_rectangular(costs)
        result: Dict[int, Vec2] = {}
        for row_index, column_index in enumerate(assignment):
            if column_index < 0 or column_index >= len(slots):
                raise PlanningError("Staging assignment returned an invalid slot")
            if costs[row_index][column_index] >= 1.0e11:
                raise PlanningError(
                    f"No validator-approved staging slot for piece {piece_ids[row_index]}"
                )
            result[piece_ids[row_index]] = slots[column_index]
        return result

    @staticmethod
    def _hungarian_rectangular(costs: Sequence[Sequence[float]]) -> List[int]:
        """Minimum-cost assignment for rows <= columns, O(rows^2 * columns)."""
        n = len(costs)
        if n == 0:
            return []
        m = len(costs[0])
        if m < n or any(len(row) != m for row in costs):
            raise PlanningError("Invalid rectangular staging cost matrix")

        # 1-indexed implementation of the shortest augmenting path variant.
        u = [0.0] * (n + 1)
        v = [0.0] * (m + 1)
        p = [0] * (m + 1)
        way = [0] * (m + 1)

        for i in range(1, n + 1):
            p[0] = i
            minv = [float("inf")] * (m + 1)
            used = [False] * (m + 1)
            j0 = 0

            while True:
                used[j0] = True
                i0 = p[j0]
                delta = float("inf")
                j1 = 0
                for j in range(1, m + 1):
                    if used[j]:
                        continue
                    cur = costs[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j

                if not math.isfinite(delta):
                    raise PlanningError("No finite staging assignment exists")

                for j in range(m + 1):
                    if used[j]:
                        u[p[j]] += delta
                        v[j] -= delta
                    else:
                        minv[j] -= delta
                j0 = j1
                if p[j0] == 0:
                    break

            while True:
                j1 = way[j0]
                p[j0] = p[j1]
                j0 = j1
                if j0 == 0:
                    break

        assignment = [-1] * n
        for j in range(1, m + 1):
            if p[j] != 0:
                assignment[p[j] - 1] = j - 1
        return assignment

    def _decompress_to_staging(
        self,
        state: _PlanState,
        goals: Dict[int, Vec2],
        available_slots: List[Vec2],
        validator: Optional[Validator],
    ) -> Dict[int, Vec2]:
        unstaged: Set[int] = set(state.positions)
        staged_positions: Dict[int, Vec2] = {}
        free_slots = list(available_slots)
        assigned_slots = self._assign_staging_slots(
            positions=state.positions,
            goals=goals,
            slots=free_slots,
            validator=validator,
        )
        pile_gap_factor = 1.00 if len(state.positions) >= 24 else 1.35

        for round_index in range(_MAX_DECOMPRESSION_ROUNDS):
            if not unstaged:
                return staged_positions

            center = self._centroid(state.positions[pid] for pid in unstaged)
            candidates: List[Tuple[float, int, Vec2]] = []

            for pid in unstaged:
                start = state.positions[pid]
                obstacles = {
                    other_id: point
                    for other_id, point in state.positions.items()
                    if other_id != pid
                }

                preferred_slot = assigned_slots[pid]

                def slot_is_target_compatible(slot: Vec2) -> bool:
                    if pid in goals:
                        return all(
                            other_id == pid
                            or _dist(slot, other_goal) >= _CLEARANCE_MM
                            for other_id, other_goal in goals.items()
                        )
                    return all(
                        _dist(slot, other_goal) >= _CLEARANCE_MM
                        for other_goal in goals.values()
                    )

                alternatives = [
                    slot
                    for slot in free_slots
                    if slot != preferred_slot
                    and slot_is_target_compatible(slot)
                ]
                alternatives.sort(
                    key=lambda slot: self._staging_slot_cost(
                        pid=pid,
                        start=start,
                        slot=slot,
                        goals=goals,
                    )
                )
                slots = []
                if preferred_slot in free_slots and slot_is_target_compatible(
                    preferred_slot
                ):
                    slots.append(preferred_slot)
                slots.extend(alternatives)

                close_neighbours = sum(
                    1
                    for point in obstacles.values()
                    if _dist(start, point) < _CLEARANCE_MM * 1.75
                )
                exposure = _dist(start, center)

                for slot in slots:
                    if validator is not None and not validator(pid, slot[0], slot[1]):
                        continue

                    # Do not park directly against the still-dense pile.  A
                    # technically legal 33 mm endpoint can form a new wall and
                    # trap the next extraction layer.  Keep newly staged pieces
                    # at least 1.35 clearances from every other unstaged piece.
                    other_unstaged = [
                        state.positions[other_id]
                        for other_id in unstaged
                        if other_id != pid
                    ]
                    min_unstaged_gap = min(
                        (_dist(slot, point) for point in other_unstaged),
                        default=float("inf"),
                    )
                    if min_unstaged_gap < _CLEARANCE_MM * pile_gap_factor:
                        continue

                    if not _decompression_segment_clear(
                        piece_id=pid,
                        start=start,
                        end=slot,
                        obstacles=obstacles,
                        validator=validator,
                        require_clear_endpoint=True,
                    ):
                        continue

                    travel = _dist(start, slot)
                    score = (
                        close_neighbours * 1_000.0
                        + exposure * 4.0
                        + min(min_unstaged_gap, _CLEARANCE_MM * 5.0) * 2.0
                        - travel
                        - self._staging_slot_cost(pid, start, slot, goals) * 0.1
                    )
                    candidates.append((score, pid, slot))

            wave: List[_Move] = []
            used_pids: Set[int] = set()
            used_slots: Set[Vec2] = set()

            for _, pid, slot in sorted(candidates, reverse=True):
                if pid in used_pids or slot in used_slots:
                    continue
                move = self._make_move(
                    pid,
                    state.positions[pid],
                    slot,
                    note=f"decompress_stage r={round_index}",
                )
                if not all(
                    _moves_mutually_safe(move, accepted, relaxed_start=True)
                    for accepted in wave
                ):
                    continue

                wave.append(move)
                used_pids.add(pid)
                used_slots.add(slot)
                if len(wave) >= _MAX_PARALLEL_MOVERS:
                    break

            if wave:
                state.emit_wave(wave)
                for move in wave:
                    staged_positions[move.piece_id] = move.end
                    unstaged.remove(move.piece_id)
                    free_slots.remove(move.end)
                continue

            # No piece can reach a staging slot directly.  Move one exposed
            # piece outward, then retry the full staging search next round.
            escape = self._find_escape_move(
                unstaged=unstaged,
                positions=state.positions,
                validator=validator,
                round_index=round_index,
            )
            if escape is None:
                raise PlanningError(
                    "Unable to decompress the starting pile. No piece has a "
                    "safe outward escape path to create additional space."
                )
            state.emit_wave([escape])

        raise PlanningError(
            f"Pile decompression exceeded {_MAX_DECOMPRESSION_ROUNDS} rounds"
        )

    def _staging_slot_cost(
        self,
        pid: int,
        start: Vec2,
        slot: Vec2,
        goals: Dict[int, Vec2],
    ) -> float:
        travel_to_slot = _dist(start, slot)
        if pid in goals:
            # Keep a piece roughly near its destination side without allowing
            # this secondary objective to overwhelm safe extraction.
            return travel_to_slot + 0.35 * _dist(slot, goals[pid])

        # Puzzle pieces not used in the target layout become fixed storage.
        # Prefer slots far away from every requested goal.
        if goals:
            nearest_goal = min(_dist(slot, goal) for goal in goals.values())
        else:
            nearest_goal = 0.0
        return travel_to_slot - nearest_goal * 2.0

    def _find_escape_move(
        self,
        unstaged: Set[int],
        positions: Dict[int, Vec2],
        validator: Optional[Validator],
        round_index: int,
    ) -> Optional[_Move]:
        center = self._centroid(positions[pid] for pid in unstaged)
        ordered = sorted(
            unstaged,
            key=lambda pid: _dist(positions[pid], center),
            reverse=True,
        )

        for pid in ordered:
            start = positions[pid]
            obstacles = {
                other_id: point
                for other_id, point in positions.items()
                if other_id != pid
            }

            repulse_x = 0.0
            repulse_y = 0.0
            for point in obstacles.values():
                distance = max(_dist(start, point), 1.0)
                if distance > _CLEARANCE_MM * 2.5:
                    continue
                weight = 1.0 / (distance * distance)
                repulse_x += (start[0] - point[0]) * weight
                repulse_y += (start[1] - point[1]) * weight

            if abs(repulse_x) + abs(repulse_y) <= _EPS:
                repulse_x = start[0] - center[0]
                repulse_y = start[1] - center[1]
            if abs(repulse_x) + abs(repulse_y) <= _EPS:
                repulse_x = 1.0

            base_angle = math.atan2(repulse_y, repulse_x)
            angle_offsets = [
                0.0,
                math.radians(22.5),
                -math.radians(22.5),
                math.radians(45.0),
                -math.radians(45.0),
                math.radians(67.5),
                -math.radians(67.5),
                math.pi,
            ]

            for distance_factor in (0.75, 1.25, 1.75, 2.5, 3.5):
                travel = _CLEARANCE_MM * distance_factor
                for offset in angle_offsets:
                    angle = base_angle + offset
                    end = (
                        start[0] + math.cos(angle) * travel,
                        start[1] + math.sin(angle) * travel,
                    )
                    if not _decompression_segment_clear(
                        piece_id=pid,
                        start=start,
                        end=end,
                        obstacles=obstacles,
                        validator=validator,
                        require_clear_endpoint=False,
                    ):
                        continue
                    return self._make_move(
                        pid,
                        start,
                        end,
                        note=f"decompress_escape r={round_index}",
                    )
        return None

    # ------------------------------------------------------------------
    # Reverse-fill accessibility planning
    # ------------------------------------------------------------------

    def _compute_placement_layers(
        self,
        required_ids: Set[int],
        staged_positions: Dict[int, Vec2],
        goals: Dict[int, Vec2],
        fixed_storage: Dict[int, Vec2],
        validator: Optional[Validator],
    ) -> List[List[int]]:
        """Peel the completed target layout into accessibility layers.

        Every piece in one removal layer can reach its assigned staging point
        while all targets in that same layer are still occupied.  Reversing the
        layers therefore gives a safe inside-out placement dependency order.
        The exact order inside each forward layer is solved separately against
        the real staged positions.
        """
        if not required_ids:
            return []

        remaining = set(required_ids)
        removed_to_stage: Set[int] = set()
        removal_layers: List[List[int]] = []

        while remaining:
            candidates: List[Tuple[float, int]] = []
            for pid in remaining:
                obstacles: Dict[int, Vec2] = dict(fixed_storage)
                obstacles.update(
                    {
                        other_id: goals[other_id]
                        for other_id in remaining
                        if other_id != pid
                    }
                )
                obstacles.update(
                    {
                        other_id: staged_positions[other_id]
                        for other_id in removed_to_stage
                    }
                )

                reverse_path = self._find_path(
                    piece_id=pid,
                    start=goals[pid],
                    goal=staged_positions[pid],
                    obstacles=obstacles,
                    validator=validator,
                )
                if reverse_path is None:
                    continue

                opens = sum(
                    1
                    for other_id in remaining
                    if other_id != pid
                    and _pt_seg_dist_sq(
                        goals[pid],
                        goals[other_id],
                        staged_positions[other_id],
                    )
                    < (_CLEARANCE_MM * 1.5) ** 2
                )
                path_length = sum(
                    _dist(a, b)
                    for a, b in zip(reverse_path, reverse_path[1:])
                )
                candidates.append((opens * 1_000.0 - path_length, pid))

            if not candidates:
                unresolved = ", ".join(str(pid) for pid in sorted(remaining))
                raise PlanningError(
                    "Reverse-fill analysis found no accessible target. "
                    f"Unresolved pieces: {unresolved}. Add more staging aisles "
                    "or enlarge the legal movement area."
                )

            layer = [pid for _, pid in sorted(candidates, reverse=True)]
            removal_layers.append(layer)
            remaining.difference_update(layer)
            removed_to_stage.update(layer)

        removal_layers.reverse()
        return removal_layers

    # ------------------------------------------------------------------
    # Final routing
    # ------------------------------------------------------------------

    def _route_layer(
        self,
        state: _PlanState,
        layer: Sequence[int],
        goals: Dict[int, Vec2],
        validator: Optional[Validator],
        layer_index: int,
    ) -> None:
        """Route one reverse-fill layer with cached rolling-horizon paths.

        The old implementation recursively searched many possible orders and
        then reran A* for every unfinished piece after every wave. This version
        uses a cheap access heuristic for priority and keeps each route until
        its next segment is invalidated by a changed obstacle layout.
        """
        order = [
            pid
            for pid in layer
            if _dist_sq(state.positions[pid], goals[pid])
            > _ARRIVAL_EPS_MM * _ARRIVAL_EPS_MM
        ]
        if not order:
            return

        clearance_sq = _CLEARANCE_MM * _CLEARANCE_MM

        def priority_key(pid: int) -> Tuple[int, float]:
            start = state.positions[pid]
            goal = goals[pid]
            blockers = sum(
                1
                for other_id, point in state.positions.items()
                if other_id != pid
                and _pt_seg_dist_sq(point, start, goal) < clearance_sq - _EPS
            )
            return blockers, _dist_sq(start, goal)

        # Most constrained and longest routes get first access to lanes.
        order.sort(key=priority_key, reverse=True)
        order_rank = {pid: index for index, pid in enumerate(order)}
        route_cache: Dict[int, List[Vec2]] = {}
        emitted_waves = 0

        while True:
            unresolved = [
                pid
                for pid in order
                if _dist_sq(state.positions[pid], goals[pid])
                > _ARRIVAL_EPS_MM * _ARRIVAL_EPS_MM
            ]
            if not unresolved:
                break

            if emitted_waves >= _MAX_LAYER_WAVES:
                raise PlanningError(
                    f"Placement layer {layer_index} exceeded "
                    f"{_MAX_LAYER_WAVES} scheduled waves"
                )

            snapshot = dict(state.positions)

            def obstacles_for(pid: int) -> Dict[int, Vec2]:
                return {
                    other_id: point
                    for other_id, point in snapshot.items()
                    if other_id != pid
                }

            def get_path(pid: int, allow_astar: bool) -> Optional[List[Vec2]]:
                start = snapshot[pid]
                goal = goals[pid]
                obstacles = obstacles_for(pid)

                cached = route_cache.get(pid)
                if (
                    cached is not None
                    and len(cached) >= 2
                    and _dist_sq(cached[0], start) <= 0.25
                    and _segment_clear_static(
                        pid, cached[0], cached[1], obstacles, validator
                    )
                ):
                    return cached

                # Direct paths are common and avoid entering A* entirely.
                if _segment_clear_static(pid, start, goal, obstacles, validator):
                    direct = [start, goal]
                    route_cache[pid] = direct
                    return direct

                if not allow_astar:
                    route_cache.pop(pid, None)
                    return None

                path = self._find_path(
                    piece_id=pid,
                    start=start,
                    goal=goal,
                    obstacles=obstacles,
                    validator=validator,
                )
                if path is None or len(path) < 2:
                    route_cache.pop(pid, None)
                    return None
                route_cache[pid] = path
                return path

            # Find the first reachable priority piece. Only this search is
            # guaranteed to invoke A*; later candidates use a limited budget.
            priority_pid: Optional[int] = None
            priority_path: Optional[List[Vec2]] = None
            for pid in unresolved:
                path = get_path(pid, allow_astar=True)
                if path is not None:
                    priority_pid = pid
                    priority_path = path
                    break

            if priority_pid is None or priority_path is None:
                raise PlanningError(
                    f"Placement layer {layer_index} has no currently "
                    f"reachable piece. Unresolved pieces: {unresolved}"
                )

            priority_move = self._make_move(
                priority_pid,
                priority_path[0],
                priority_path[1],
                note=(
                    f"setup layer={layer_index} "
                    f"priority={order_rank[priority_pid]} cached_rolling"
                ),
            )
            if not self._wave_is_safe_from_snapshot(
                [priority_move], snapshot, validator
            ):
                route_cache.pop(priority_pid, None)
                raise PlanningError(
                    f"Placement layer {layer_index}: freshly planned move "
                    f"for priority piece {priority_pid} failed safety check"
                )

            wave: List[_Move] = [priority_move]
            protected_segments = list(zip(priority_path[1:], priority_path[2:]))
            protected_goal = goals[priority_pid]
            astar_attempts = 0

            for pid in unresolved:
                if pid == priority_pid:
                    continue
                if len(wave) >= _MAX_PARALLEL_MOVERS:
                    break

                # Cached/direct routes are nearly free. Limit fresh A* work for
                # optional parallel movers so one wave cannot trigger dozens of
                # full searches.
                cached = route_cache.get(pid)
                allow_astar = cached is not None or astar_attempts < _MAX_PARALLEL_ASTAR_ATTEMPTS
                if cached is None and allow_astar:
                    astar_attempts += 1
                path = get_path(pid, allow_astar=allow_astar)
                if path is None:
                    continue

                candidate = self._make_move(
                    pid,
                    path[0],
                    path[1],
                    note=(
                        f"setup layer={layer_index} "
                        f"order={order_rank[pid]} cached_parallel"
                    ),
                )

                if _dist_sq(candidate.end, protected_goal) < clearance_sq - _EPS:
                    continue
                if any(
                    _pt_seg_dist_sq(candidate.end, segment_start, segment_end)
                    < clearance_sq - _EPS
                    for segment_start, segment_end in protected_segments
                ):
                    continue

                proposed = wave + [candidate]
                if self._wave_is_safe_from_snapshot(proposed, snapshot, validator):
                    wave = proposed

            state.emit_wave(wave)

            # Advance only the routes that actually moved. All other cached
            # routes remain available and will be revalidated next iteration.
            for move in wave:
                cached = route_cache.get(move.piece_id)
                if (
                    cached is not None
                    and len(cached) >= 2
                    and _dist_sq(cached[1], move.end) <= 0.25
                ):
                    advanced = cached[1:]
                    if len(advanced) >= 2:
                        route_cache[move.piece_id] = advanced
                    else:
                        route_cache.pop(move.piece_id, None)
                else:
                    route_cache.pop(move.piece_id, None)

            emitted_waves += 1

    # ------------------------------------------------------------------
    # A* pathfinding and simplification
    # ------------------------------------------------------------------

    def _find_path(
        self,
        piece_id: int,
        start: Vec2,
        goal: Vec2,
        obstacles: Dict[int, Vec2],
        validator: Optional[Validator],
    ) -> Optional[List[Vec2]]:
        if _dist(start, goal) <= _ARRIVAL_EPS_MM:
            return [start]
        if _segment_clear_static(piece_id, start, goal, obstacles, validator):
            return [start, goal]

        x_min = float(SIMULATOR.X_MIN_MM) + _RADIUS_MM
        y_min = float(SIMULATOR.Y_MIN_MM) + _RADIUS_MM
        x_max = float(SIMULATOR.X_MAX_MM) - _RADIUS_MM
        y_max = float(SIMULATOR.Y_MAX_MM) - _RADIUS_MM

        width = int(math.floor((x_max - x_min) / _GRID_MM)) + 1
        height = int(math.floor((y_max - y_min) / _GRID_MM)) + 1
        if width <= 0 or height <= 0:
            return None

        def to_point(node: Tuple[int, int]) -> Vec2:
            return x_min + node[0] * _GRID_MM, y_min + node[1] * _GRID_MM

        def nearest_node(point: Vec2) -> Tuple[int, int]:
            return (
                max(0, min(width - 1, int(round((point[0] - x_min) / _GRID_MM)))),
                max(0, min(height - 1, int(round((point[1] - y_min) / _GRID_MM)))),
            )

        obstacle_index = _ObstacleIndex(obstacles)
        clearance_sq = _CLEARANCE_MM * _CLEARANCE_MM

        def segment_clear(a: Vec2, b: Vec2) -> bool:
            if not _segment_validator_ok(piece_id, a, b, validator):
                return False
            for other_id, point in obstacle_index.near_segment(a, b):
                if other_id == piece_id:
                    continue
                if _pt_seg_dist_sq(point, a, b) < clearance_sq - _EPS:
                    return False
            return True

        start_node = self._nearest_connectable_node(
            start,
            nearest_node(start),
            width,
            height,
            to_point,
            segment_clear,
        )
        goal_node = self._nearest_connectable_node(
            goal,
            nearest_node(goal),
            width,
            height,
            to_point,
            segment_clear,
        )
        if start_node is None or goal_node is None:
            return None

        frontier: List[Tuple[float, int, Tuple[int, int]]] = []
        counter = itertools.count()
        heapq.heappush(frontier, (0.0, next(counter), start_node))

        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {
            start_node: None
        }
        cost_so_far: Dict[Tuple[int, int], float] = {start_node: 0.0}

        neighbour_offsets = (
            (-1, -1),
            (0, -1),
            (1, -1),
            (-1, 0),
            (1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
        )

        expansions = 0
        closed: Set[Tuple[int, int]] = set()
        goal_point = to_point(goal_node)
        while frontier and expansions < _MAX_ASTAR_EXPANSIONS:
            _, _, current = heapq.heappop(frontier)
            if current in closed:
                continue
            closed.add(current)
            expansions += 1
            if current == goal_node:
                break

            current_point = to_point(current)
            for dx, dy in neighbour_offsets:
                neighbour = current[0] + dx, current[1] + dy
                if not (0 <= neighbour[0] < width and 0 <= neighbour[1] < height):
                    continue

                neighbour_point = to_point(neighbour)
                if not segment_clear(current_point, neighbour_point):
                    continue

                step_cost = _GRID_MM * (math.sqrt(2.0) if dx and dy else 1.0)
                new_cost = cost_so_far[current] + step_cost
                if new_cost >= cost_so_far.get(neighbour, float("inf")) - _EPS:
                    continue

                cost_so_far[neighbour] = new_cost
                heuristic = _dist(neighbour_point, goal_point)
                heapq.heappush(
                    frontier,
                    (
                        new_cost + _ASTAR_HEURISTIC_WEIGHT * heuristic,
                        next(counter),
                        neighbour,
                    ),
                )
                came_from[neighbour] = current

        if goal_node not in came_from:
            return None

        nodes: List[Tuple[int, int]] = []
        current: Optional[Tuple[int, int]] = goal_node
        while current is not None:
            nodes.append(current)
            current = came_from[current]
        nodes.reverse()

        path: List[Vec2] = [start]
        for node in nodes:
            point = to_point(node)
            if _dist(path[-1], point) > 0.5:
                path.append(point)
        if _dist(path[-1], goal) > 0.5:
            path.append(goal)

        return self._simplify_path(
            path=path,
            segment_clear=segment_clear,
        )

    def _nearest_connectable_node(
        self,
        exact_point: Vec2,
        seed: Tuple[int, int],
        width: int,
        height: int,
        to_point: Callable[[Tuple[int, int]], Vec2],
        segment_clear: Callable[[Vec2, Vec2], bool],
    ) -> Optional[Tuple[int, int]]:
        for radius in range(0, 5):
            candidates: List[Tuple[float, Tuple[int, int]]] = []
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    node = seed[0] + dx, seed[1] + dy
                    if not (0 <= node[0] < width and 0 <= node[1] < height):
                        continue
                    point = to_point(node)
                    candidates.append((_dist(exact_point, point), node))

            for _, node in sorted(candidates):
                point = to_point(node)
                if segment_clear(exact_point, point):
                    return node
        return None

    def _simplify_path(
        self,
        path: Sequence[Vec2],
        segment_clear: Callable[[Vec2, Vec2], bool],
    ) -> List[Vec2]:
        if len(path) <= 2:
            return list(path)

        simplified = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            chosen = anchor + 1
            for candidate in range(len(path) - 1, anchor, -1):
                if segment_clear(path[anchor], path[candidate]):
                    chosen = candidate
                    break
            simplified.append(path[chosen])
            anchor = chosen
        return simplified

    # ------------------------------------------------------------------
    # Safe parallel compaction
    # ------------------------------------------------------------------

    def _compact_parallel_waves(
        self,
        initial_positions: Dict[int, Vec2],
        commands: Sequence[MoveCommand],
        validator: Optional[Validator],
    ) -> List[MoveCommand]:
        """Merge adjacent sequential waves when the merged timed wave is safe.

        Reverse-fill placement is intentionally planned in a strict order for
        correctness.  This pass recovers parallelism without weakening that
        guarantee: a later wave is pulled earlier only when every segment is
        safe from the earlier snapshot and every moving pair passes the exact
        timed closest-approach check.
        """
        if not commands:
            return []

        grouped: Dict[int, List[MoveCommand]] = {}
        for command in commands:
            grouped.setdefault(command.sequence_num, []).append(command)

        original_positions = dict(initial_positions)
        original_waves: List[List[_Move]] = []
        for sequence_num in sorted(grouped):
            wave: List[_Move] = []
            for command in grouped[sequence_num]:
                start = original_positions[command.piece_id]
                end = float(command.target_x_mm), float(command.target_y_mm)
                wave.append(
                    _Move(
                        piece_id=command.piece_id,
                        start=start,
                        end=end,
                        duration_ms=int(command.duration_ms),
                        note=command.planner_debug or "",
                    )
                )
            original_waves.append(wave)
            for move in wave:
                original_positions[move.piece_id] = move.end

        compacted: List[MoveCommand] = []
        compact_positions = dict(initial_positions)
        output_sequence = 0
        index = 0

        while index < len(original_waves):
            combined = list(original_waves[index])
            consumed = 1

            while index + consumed < len(original_waves):
                next_wave = original_waves[index + consumed]
                if len(combined) + len(next_wave) > _MAX_PARALLEL_MOVERS:
                    break

                combined_ids = {move.piece_id for move in combined}
                if any(move.piece_id in combined_ids for move in next_wave):
                    break

                # A command can move earlier only if its original start is the
                # same position it occupies at the beginning of this compacted
                # wave. Otherwise it depends on an earlier command for itself.
                if any(
                    _dist(compact_positions[move.piece_id], move.start) > 0.5
                    for move in next_wave
                ):
                    break

                proposed = combined + list(next_wave)
                if not self._wave_is_safe_from_snapshot(
                    proposed,
                    compact_positions,
                    validator,
                ):
                    break

                combined = proposed
                consumed += 1

            if not self._wave_is_safe_from_snapshot(
                combined,
                compact_positions,
                validator,
            ):
                raise PlanningError(
                    f"Parallel compaction encountered an unsafe source wave "
                    f"at original wave index {index}"
                )

            for move in combined:
                compacted.append(
                    MoveCommand(
                        piece_id=move.piece_id,
                        target_x_mm=move.end[0],
                        target_y_mm=move.end[1],
                        duration_ms=move.duration_ms,
                        sequence_num=output_sequence,
                        planner_debug=(move.note + " [parallel_compact]").strip(),
                    )
                )
            for move in combined:
                compact_positions[move.piece_id] = move.end

            output_sequence += 1
            index += consumed

        return compacted

    def _wave_is_safe_from_snapshot(
        self,
        wave: Sequence[_Move],
        positions: Dict[int, Vec2],
        validator: Optional[Validator],
    ) -> bool:
        moving_ids = {move.piece_id for move in wave}
        if len(moving_ids) != len(wave):
            return False

        static = {
            pid: point
            for pid, point in positions.items()
            if pid not in moving_ids
        }

        for move in wave:
            if _dist(positions[move.piece_id], move.start) > 0.5:
                return False
            if move.note.startswith("decompress_"):
                if not _decompression_segment_clear(
                    piece_id=move.piece_id,
                    start=move.start,
                    end=move.end,
                    obstacles=static,
                    validator=validator,
                    require_clear_endpoint=False,
                ):
                    return False
            elif not _segment_clear_static(
                piece_id=move.piece_id,
                start=move.start,
                end=move.end,
                obstacles=static,
                validator=validator,
            ):
                return False

        for a, b in itertools.combinations(wave, 2):
            relaxed = (
                a.note.startswith("decompress_")
                or b.note.startswith("decompress_")
            )
            if not _moves_mutually_safe(a, b, relaxed_start=relaxed):
                return False

        return True

    # ------------------------------------------------------------------
    # Independent plan audit
    # ------------------------------------------------------------------

    def _audit_plan(
        self,
        initial_positions: Dict[int, Vec2],
        goals: Dict[int, Vec2],
        commands: Sequence[MoveCommand],
        validator: Optional[Validator],
    ) -> None:
        positions = dict(initial_positions)
        waves: Dict[int, List[MoveCommand]] = {}
        for command in commands:
            waves.setdefault(command.sequence_num, []).append(command)

        expected_sequences = list(range(len(waves)))
        actual_sequences = sorted(waves)
        if actual_sequences != expected_sequences:
            raise PlanningError(
                f"Non-contiguous sequence numbers: {actual_sequences}"
            )

        for sequence_num in actual_sequences:
            commands_in_wave = waves[sequence_num]
            moving_ids = {command.piece_id for command in commands_in_wave}
            if len(moving_ids) != len(commands_in_wave):
                raise PlanningError(
                    f"Duplicate piece command in wave {sequence_num}"
                )

            moves: List[_Move] = []
            for command in commands_in_wave:
                if command.piece_id not in positions:
                    raise PlanningError(
                        f"Wave {sequence_num} references unknown piece "
                        f"{command.piece_id}"
                    )
                start = positions[command.piece_id]
                end = float(command.target_x_mm), float(command.target_y_mm)
                moves.append(
                    _Move(
                        piece_id=command.piece_id,
                        start=start,
                        end=end,
                        duration_ms=int(command.duration_ms),
                        note=command.planner_debug or "",
                    )
                )

            static = {
                pid: point
                for pid, point in positions.items()
                if pid not in moving_ids
            }

            for move in moves:
                relaxed = move.note.startswith("decompress_")
                if relaxed:
                    safe = _decompression_segment_clear(
                        piece_id=move.piece_id,
                        start=move.start,
                        end=move.end,
                        obstacles=static,
                        validator=validator,
                        require_clear_endpoint=False,
                    )
                else:
                    safe = _segment_clear_static(
                        piece_id=move.piece_id,
                        start=move.start,
                        end=move.end,
                        obstacles=static,
                        validator=validator,
                    )
                if not safe:
                    raise PlanningError(
                        f"Audit rejected piece {move.piece_id} in wave "
                        f"{sequence_num}: {move.start} -> {move.end}"
                    )

            for a, b in itertools.combinations(moves, 2):
                relaxed = (
                    a.note.startswith("decompress_")
                    or b.note.startswith("decompress_")
                )
                if not _moves_mutually_safe(a, b, relaxed_start=relaxed):
                    raise PlanningError(
                        f"Audit found a same-wave collision between pieces "
                        f"{a.piece_id} and {b.piece_id} in wave {sequence_num}"
                    )

            for move in moves:
                positions[move.piece_id] = move.end

        unresolved = [
            pid
            for pid, goal in goals.items()
            if _dist(positions[pid], goal) > _ARRIVAL_EPS_MM
        ]
        if unresolved:
            raise PlanningError(
                "Audited plan does not complete targets for pieces: "
                + ", ".join(str(pid) for pid in sorted(unresolved))
            )

        clearance_sq = _CLEARANCE_MM * _CLEARANCE_MM
        for (pid_a, point_a), (pid_b, point_b) in itertools.combinations(
            positions.items(), 2
        ):
            if _dist_sq(point_a, point_b) < clearance_sq - _EPS:
                raise PlanningError(
                    f"Final positions for pieces {pid_a} and {pid_b} violate "
                    f"clearance: {_dist(point_a, point_b):.1f} mm"
                )

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    def _make_move(self, pid: int, start: Vec2, end: Vec2, note: str) -> _Move:
        return _Move(
            piece_id=pid,
            start=start,
            end=end,
            duration_ms=self.duration_for_distance(_dist(start, end)),
            note=note,
        )

    @staticmethod
    def _centroid(points: Iterable[Vec2]) -> Vec2:
        points_list = list(points)
        if not points_list:
            return 0.0, 0.0
        return (
            sum(point[0] for point in points_list) / len(points_list),
            sum(point[1] for point in points_list) / len(points_list),
        )
