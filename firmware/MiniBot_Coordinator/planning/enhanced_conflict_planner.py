"""
planning/enhanced_conflict_planner.py  -  MiniBot Chess Swarm Coordinator

EnhancedConflictPlanner: collision-safe parallel-wave iterative planner with
active deadlock resolution.

When both sea-parting and single-blocker detour fail on the same iteration,
a "make way" cycle is triggered:

  1. The stuck piece *furthest from its goal* is selected as the priority
     piece for this cycle.
  2. Every other piece whose current position falls within _CLEARANCE of
     the priority piece's direct path is given a "park" position outside
     that corridor.  Multiple blocking pieces are moved in parallel per
     clearing round.  Cascades (blocker A can't park because blocker B is
     in the way) are resolved iteratively across MAKE_WAY_PARK_ROUNDS rounds.
  3. Once the corridor is clear, the priority piece is sent directly to its
     goal in a single wave.
  4. The main iterative planner resumes from the new positions.

If a new deadlock occurs after a make-way cycle, steps 1-4 repeat until
MAKE_WAY_MAX_CYCLES interventions have been exhausted.

Clean-room redesign with collision safety as the foundational invariant.

Priority order (highest first):
  1. Collision safety  - every emitted move passes a swept-capsule check against
                         ALL other pieces before it is accepted.  No exceptions.
  2. Target completion - iterative replanning + detour waves keep going until
                         every piece reaches its goal (or the iteration cap hits).
  3. Parallel motion   - pieces whose moves are mutually safe share a wave
                         (same sequence_num) so the bots move simultaneously.
  4. Move chaining     - after planning, consecutive single-piece collinear waves
                         are merged into one command to simplify communication.

Algorithm overview
------------------
Each outer iteration tries to build one "wave" (a set of parallel moves):

  _build_wave()
    - Processes pieces in decreasing remaining-distance order (furthest first).
    - For each candidate piece:
        1. Try the direct path to goal at scales 1.0, 0.75, 0.5, 0.25.
        2. If all direct scales are blocked, try bypass waypoints offset
           perpendicularly from the nearest blocker (both sides x 3 distances).
    - For pieces already committed to THIS wave, their endpoint (not starting
      position) is used as their effective position.  This correctly models
      simultaneous motion: two pieces moving at the same time cannot end up
      within clearance of each other.
    - _safe() is THE single collision gate, checked against:
        a) swept capsule vs. all static piece positions
        b) endpoint   vs. all static piece positions
        c) swept capsule vs. all committed wave segments
        d) endpoint   vs. all committed wave endpoints

  When no piece can make forward progress:
    _find_detour()
      - Counts how many stuck pieces each other piece is blocking.
      - Moves the highest-vote blocker to the nearest safe escape position
        (8 compass directions x 3 clearance multiples).

  After all waves are built, _chain_merge() fuses consecutive waves that
  contain only one piece and whose segments are collinear (<=30 deg turn).
"""

from __future__ import annotations

import itertools
import logging
import math
from typing import Callable, Dict, List, Optional, Tuple

from config import PIECES, PLANNING, SIMULATOR
from planning.base_planner import BasePlanner, MoveCommand

Vec2 = Tuple[float, float]

# -- Planning constants --------------------------------------------------------

# Centre-to-centre minimum gap used during planning.
# Physical diameter = 2 x 15.5 = 31 mm.  We add 2 mm so control-loop errors
# and rounding do not cause real collisions.
_CLEARANCE: float = 2.0 * float(PIECES.CIRCLE_RADIUS_MM) + 2.0   # 33 mm

_MIN_SEG:   float = float(getattr(PLANNING, 'CONFLICT_MIN_SEGMENT_MM', 20.0))
_ARRIVE:    float = float(getattr(PLANNING, 'CONFLICT_ARRIVAL_EPS_MM',  2.0))
_DOCK:      float = float(getattr(PLANNING, 'CONFLICT_DOCK_EPS_MM',     8.0))
_MAX_ITER:  int   = int(getattr(PLANNING,   'CONFLICT_MAX_ITERATIONS',  50))
_MAX_STALL:      int = 8   # consecutive no-progress iterations before giving up
_MAX_SEA_ROUNDS: int = 8   # maximum clearing waves planned per sea-parting attempt

# Make-way deadlock resolution
_MW_STALL_TRIGGER: int   = int(getattr(PLANNING, 'MAKE_WAY_DEADLOCK_TRIGGER', 4))
_MW_MAX_CYCLES:    int   = int(getattr(PLANNING, 'MAKE_WAY_MAX_CYCLES',       5))
_MW_PARK_ROUNDS:   int   = int(getattr(PLANNING, 'MAKE_WAY_PARK_ROUNDS',      8))
# Maximum times the priority piece is allowed to yield per _make_way call.
_MW_YIELD_MAX:     int   = int(getattr(PLANNING, 'MAKE_WAY_YIELD_MAX',        2))

# Net-progress stall counter (proposal 2)
_MW_NET_PROGRESS_MM: float = float(getattr(PLANNING, 'MAKE_WAY_NET_PROGRESS_MM', 5.0))
_MW_NET_STALL_CAP:   int   = int(getattr(PLANNING,   'MAKE_WAY_NET_STALL_CAP',  14))

# Cycle detector (proposal 3)
_MW_CYCLE_GRID:    int = int(getattr(PLANNING, 'MAKE_WAY_CYCLE_GRID_MM',  5))
_MW_CYCLE_HISTORY: int = int(getattr(PLANNING, 'MAKE_WAY_CYCLE_HISTORY', 10))

log = logging.getLogger(__name__)


# -- Module-level geometry (no self, no allocations) ---------------------------

def _dist(a: Vec2, b: Vec2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _pt_seg_dist(p: Vec2, a: Vec2, b: Vec2) -> float:
    """Minimum distance from point p to segment a->b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    denom = dx * dx + dy * dy
    if denom < 1e-12:
        return _dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / denom))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _seg_seg_dist(a1: Vec2, a2: Vec2, b1: Vec2, b2: Vec2) -> float:
    """Minimum distance between segments a1->a2 and b1->b2."""
    def _cross(o: Vec2, u: Vec2, v: Vec2) -> float:
        return (u[0] - o[0]) * (v[1] - o[1]) - (u[1] - o[1]) * (v[0] - o[0])

    if ((_cross(a1, a2, b1) > 0) != (_cross(a1, a2, b2) > 0) and
            (_cross(b1, b2, a1) > 0) != (_cross(b1, b2, a2) > 0)):
        return 0.0   # segments cross
    return min(
        _pt_seg_dist(a1, b1, b2),
        _pt_seg_dist(a2, b1, b2),
        _pt_seg_dist(b1, a1, a2),
        _pt_seg_dist(b2, a1, a2),
    )


def _in_bounds(p: Vec2) -> bool:
    r = float(PIECES.CIRCLE_RADIUS_MM)
    return (
        SIMULATOR.X_MIN_MM + r <= p[0] <= SIMULATOR.X_MAX_MM - r
        and SIMULATOR.Y_MIN_MM + r <= p[1] <= SIMULATOR.Y_MAX_MM - r
    )


# Internal move record: (piece_id, start, end, note)
_Move = Tuple[int, Vec2, Vec2, str]


# -- Target optimisation (brute-force optimal within each interchangeable group) --

def _optimise_targets_optimal(
    pos:   Dict[int, Vec2],
    goals: Dict[int, Vec2],
) -> Dict[int, Vec2]:
    """Reassign goals among interchangeable pieces to minimise total travel.

    Uses brute-force permutation search within each (colour, rank) group so
    the result is globally optimal.  Max group = 8 pawns ? 8! = 40 320 iters.
    """
    if not goals:
        return goals

    groups: Dict[tuple, List[int]] = {}
    for pid in goals:
        rank = PIECES.PIECE_RANKS.get(pid, '')
        if rank in ('pawn', 'bishop', 'rook', 'knight'):
            colour = 'white' if pid in PIECES.WHITE_IDS else 'black'
            groups.setdefault((colour, rank), []).append(pid)

    new_goals = dict(goals)
    for _key, pids in groups.items():
        if len(pids) < 2:
            continue
        slots = [goals[pid] for pid in pids]
        best_total = sum(_dist(pos[pids[i]], slots[i]) for i in range(len(pids)))
        best_perm: Optional[tuple] = None
        for perm in itertools.permutations(slots):
            total = sum(_dist(pos[pids[i]], perm[i]) for i in range(len(pids)))
            if total < best_total - 1.0:
                best_total = total
                best_perm = perm
        if best_perm is not None:
            for i, pid in enumerate(pids):
                new_goals[pid] = best_perm[i]
    return new_goals


# -- Planner -------------------------------------------------------------------

class EnhancedConflictPlanner(BasePlanner):
    """Collision-safe parallel-wave planner with make-way deadlock resolution."""

    @property
    def name(self) -> str:
        return 'Enhanced Conflict'

    # -- Public entry point ----------------------------------------------------

    def plan_moves(
        self,
        piece_positions: Dict[int, Tuple[float, float]],
        targets:         Dict[int, Tuple[float, float]],
        orientations:    Optional[Dict[int, float]] = None,
        validator:       Optional[Callable[[int, float, float], bool]] = None,
    ) -> List[MoveCommand]:
        del orientations
        if not targets:
            return []

        pos:   Dict[int, Vec2] = {
            pid: (float(x), float(y)) for pid, (x, y) in piece_positions.items()
        }
        goals: Dict[int, Vec2] = {
            pid: (float(x), float(y))
            for pid, (x, y) in targets.items()
            if pid in pos
        }
        if not goals:
            return []

        # -- Optimization: reassign targets among interchangeable pieces -------
        goals = self._optimise_target_assignment(pos, goals)

        commands: List[MoveCommand] = []
        seq             = 0
        stalls          = 0
        make_way_cycles = 0

        # -- Anti-oscillation state -------------------------------------------
        # Proposal 2: net-progress stall counter.
        # Tracks the minimum remaining distance across all goals.  Resets when
        # any piece makes meaningful forward progress (>= _MW_NET_PROGRESS_MM).
        best_min_rem:  float = float('inf')
        net_stalls:    int   = 0

        # Proposal 3: position fingerprint cycle detector.
        # Each iteration we hash all piece positions rounded to _MW_CYCLE_GRID.
        # If the fingerprint appeared in the recent history window, a cycle is
        # confirmed and we escalate immediately to make-way (or exit).
        fp_history: List[frozenset] = []

        # Auxiliary data for chain-merge (priority 4):
        #   wstarts[(seq, pid)]  = position of pid just before wave seq began
        #   wpos_snap[seq]       = full position snapshot before wave seq
        wstarts:   Dict[Tuple[int, int], Vec2] = {}
        wpos_snap: Dict[int, Dict[int, Vec2]]  = {}

        for _iter in range(_MAX_ITER):
            remaining = [
                pid for pid in goals
                if _dist(pos[pid], goals[pid]) > _ARRIVE
            ]
            if not remaining:
                log.info('[ECP] all targets reached in %d waves', seq)
                break

            # -- Proposal 3: cycle detection ----------------------------------
            fingerprint = frozenset(
                (pid,
                 int(round(pos[pid][0] / _MW_CYCLE_GRID)),
                 int(round(pos[pid][1] / _MW_CYCLE_GRID)))
                for pid in remaining
            )
            if fingerprint in fp_history:
                log.warning('[ECP] iter=%d: position cycle detected � escalating', _iter)
                # Jump directly to make-way, bypassing sea-part / detour
                if make_way_cycles < _MW_MAX_CYCLES:
                    mw_waves = self._make_way(remaining, pos, goals, validator)
                    if mw_waves:
                        for mw_wave in mw_waves:
                            snap = dict(pos)
                            for pid_mv, start, end, note in mw_wave:
                                commands.append(MoveCommand(
                                    piece_id     = pid_mv,
                                    target_x_mm  = end[0],
                                    target_y_mm  = end[1],
                                    duration_ms  = self.duration_for_distance(_dist(start, end)),
                                    sequence_num = seq,
                                    planner_debug= note + ' [cycle]',
                                ))
                                wstarts[(seq, pid_mv)] = start
                                pos[pid_mv] = end
                            wpos_snap[seq] = snap
                            seq += 1
                        make_way_cycles += 1
                        stalls = net_stalls = 0
                        fp_history.clear()   # reset history after intervention
                        log.info('[ECP] iter=%d: cycle-triggered make-way %d (%d waves)',
                                 _iter, make_way_cycles, len(mw_waves))
                        continue
                log.warning('[ECP] iter=%d: cycle confirmed, make-way exhausted � giving up', _iter)
                break
            fp_history.append(fingerprint)
            if len(fp_history) > _MW_CYCLE_HISTORY:
                fp_history.pop(0)

            # -- Proposal 2: net-progress stall check -------------------------
            cur_min_rem = min(_dist(pos[pid], goals[pid]) for pid in remaining)
            if cur_min_rem < best_min_rem - _MW_NET_PROGRESS_MM:
                best_min_rem = cur_min_rem
                net_stalls   = 0
            else:
                net_stalls += 1
            if net_stalls > _MW_NET_STALL_CAP:
                log.warning('[ECP] iter=%d: no net progress for %d iters � giving up',
                            _iter, net_stalls)
                break

            # -- Try to build a parallel wave ----------------------------------
            wave = self._build_wave(remaining, pos, goals, validator)

            if wave:
                snap = dict(pos)
                for mv in wave:
                    pid, start, end, note = mv
                    commands.append(MoveCommand(
                        piece_id     = pid,
                        target_x_mm  = end[0],
                        target_y_mm  = end[1],
                        duration_ms  = self.duration_for_distance(_dist(start, end)),
                        sequence_num = seq,
                        planner_debug= note,
                    ))
                    wstarts[(seq, pid)] = start
                    pos[pid] = end
                wpos_snap[seq] = snap
                seq    += 1
                stalls  = 0
                net_stalls = 0
                log.debug('[ECP] iter=%d wave=%d size=%d', _iter, seq - 1, len(wave))
                continue

            # -- No direct progress: try to clear the corridor, then detour ---
            stalls += 1
            if stalls > _MAX_STALL:
                log.warning('[ECP] iter=%d: %d consecutive stalls, giving up', _iter, stalls)
                break

            # Sea-parting: plan a cascade of clearing waves that open a lane
            # for the most-stuck remaining piece.  Multiple waves are pre-planned
            # using simulated positions so all of them are emitted back-to-back
            # before the main loop can re-insert displaced pieces.
            sea_waves = self._part_the_sea(remaining, pos, goals, validator)
            if sea_waves:
                for sea_wave in sea_waves:
                    snap = dict(pos)
                    for pid_mv, start, end, note in sea_wave:
                        commands.append(MoveCommand(
                            piece_id     = pid_mv,
                            target_x_mm  = end[0],
                            target_y_mm  = end[1],
                            duration_ms  = self.duration_for_distance(_dist(start, end)),
                            sequence_num = seq,
                            planner_debug= note,
                        ))
                        wstarts[(seq, pid_mv)] = start
                        pos[pid_mv] = end
                    wpos_snap[seq] = snap
                    seq += 1
                stalls = 0
                net_stalls = 0
                log.debug('[ECP] iter=%d: sea-part %d waves emitted', _iter, len(sea_waves))
                continue

            # Fall back to single-blocker detour
            det = self._find_detour(remaining, pos, goals, validator)
            if det is None:
                # Sea-parting and detour both failed � try make-way
                if make_way_cycles < _MW_MAX_CYCLES:
                    mw_waves = self._make_way(remaining, pos, goals, validator)
                    if mw_waves:
                        for mw_wave in mw_waves:
                            snap = dict(pos)
                            for pid_mv, start, end, note in mw_wave:
                                commands.append(MoveCommand(
                                    piece_id     = pid_mv,
                                    target_x_mm  = end[0],
                                    target_y_mm  = end[1],
                                    duration_ms  = self.duration_for_distance(_dist(start, end)),
                                    sequence_num = seq,
                                    planner_debug= note,
                                ))
                                wstarts[(seq, pid_mv)] = start
                                pos[pid_mv] = end
                            wpos_snap[seq] = snap
                            seq += 1
                        make_way_cycles += 1
                        stalls = net_stalls = 0
                        fp_history.clear()   # positions changed � old fingerprints stale
                        log.info('[ECP] iter=%d: make-way cycle %d complete (%d waves)',
                                 _iter, make_way_cycles, len(mw_waves))
                        continue
                log.warning('[ECP] iter=%d: no resolution found', _iter)
                break

            pid, start, end, note = det
            snap = dict(pos)
            commands.append(MoveCommand(
                piece_id     = pid,
                target_x_mm  = end[0],
                target_y_mm  = end[1],
                duration_ms  = self.duration_for_distance(_dist(start, end)),
                sequence_num = seq,
                planner_debug= note,
            ))
            wstarts[(seq, pid)] = start
            pos[pid] = end
            wpos_snap[seq] = snap
            seq += 1
            log.debug('[ECP] iter=%d: detour wave=%d piece=0x%02X %s', _iter, seq - 1, pid, note)

        # -- Cleanup pass: direct-move any piece within 25 mm of its goal -----
        # After the main loop, pieces that stalled just short of their target
        # (e.g. because of iteration cap or late-stage detour leftovers) get a
        # single direct corrective move appended as individual sequential waves.
        _CLEANUP_RADIUS = 25.0
        for pid in sorted(goals):
            remaining_d = _dist(pos[pid], goals[pid])
            if _ARRIVE < remaining_d <= _CLEANUP_RADIUS:
                gx, gy = goals[pid]
                commands.append(MoveCommand(
                    piece_id     = pid,
                    target_x_mm  = gx,
                    target_y_mm  = gy,
                    duration_ms  = self.duration_for_distance(remaining_d),
                    sequence_num = seq,
                    planner_debug= f'cleanup rem={remaining_d:.1f}mm',
                ))
                pos[pid] = goals[pid]
                seq += 1
                log.debug('[ECP] cleanup: piece=0x%02X rem=%.1fmm', pid, remaining_d)

        return self._chain_merge(commands, wstarts, wpos_snap)

    # -- Target assignment optimisation ----------------------------------------

    @staticmethod
    def _optimise_target_assignment(
        pos:   Dict[int, Vec2],
        goals: Dict[int, Vec2],
    ) -> Dict[int, Vec2]:
        """
        Reassign target positions among interchangeable (same rank + colour)
        pieces to minimise total travel distance.

        The physical bots keep their piece IDs; only the target coordinates
        are redistributed.  This is a greedy nearest-neighbour assignment
        within each interchangeable group, which gives the optimal result for
        groups of up to ~8 pieces in typical chess configurations.

        Interchangeable groups:
          - All pawns of the same colour
          - Both bishops of the same colour
          - Both rooks of the same colour
          - Both knights of the same colour
          (Kings and queens are unique per colour � never swapped.)
        """
        if not goals:
            return goals

        # Build interchangeable groups: key = (colour, rank)
        # Only ranks with >1 piece per colour can be swapped.
        groups: Dict[Tuple[str, str], List[int]] = {}
        for pid in goals:
            rank = PIECES.PIECE_RANKS.get(pid, '')
            if rank in ('pawn', 'bishop', 'rook', 'knight'):
                colour = 'white' if pid in PIECES.WHITE_IDS else 'black'
                groups.setdefault((colour, rank), []).append(pid)

        new_goals = dict(goals)

        for key, pids in groups.items():
            if len(pids) < 2:
                continue
            # Collect the target positions originally assigned to this group
            target_slots: List[Vec2] = [goals[pid] for pid in pids]

            # Greedy nearest-neighbour assignment:
            #   For each piece (in arbitrary order), pick the closest remaining
            #   target slot.  This is O(n�) but n = 8 for pawns.
            assigned_targets: List[Vec2] = []
            available = list(target_slots)
            for pid in pids:
                p = pos[pid]
                best_idx = min(range(len(available)), key=lambda i: _dist(p, available[i]))
                assigned_targets.append(available.pop(best_idx))

            # Check whether the reassignment reduces total travel
            original_total = sum(_dist(pos[pids[i]], goals[pids[i]]) for i in range(len(pids)))
            new_total      = sum(_dist(pos[pids[i]], assigned_targets[i]) for i in range(len(pids)))
            if new_total < original_total - 1.0:   # 1 mm hysteresis to avoid pointless swaps
                log.debug(
                    '[ECP] target-swap %s %s: total %.0f -> %.0f mm',
                    key[0], key[1], original_total, new_total,
                )
                for i, pid in enumerate(pids):
                    new_goals[pid] = assigned_targets[i]

        return new_goals

    # -- Wave builder ----------------------------------------------------------

    def _build_wave(
        self,
        remaining: List[int],
        pos:       Dict[int, Vec2],
        goals:     Dict[int, Vec2],
        validator: Optional[Callable],
    ) -> List[_Move]:
        """
        Greedily assign collision-safe partial moves to one parallel wave.

        Pieces are processed furthest-from-goal first (highest priority).
        Pieces already committed to this wave use their *endpoint* as their
        effective position so that simultaneous motion is modelled correctly.
        """
        wave:    List[_Move]     = []
        wave_ep: Dict[int, Vec2] = {}   # pid -> committed endpoint this wave

        for pid in sorted(remaining, key=lambda p: -_dist(pos[p], goals[p])):
            start = pos[pid]
            goal  = goals[pid]

            # Effective positions: committed wave pieces at endpoints
            effective: Dict[int, Vec2] = {
                oid: (wave_ep[oid] if oid in wave_ep else pos[oid])
                for oid in pos
                if oid != pid
            }

            mv = self._find_move(pid, start, goal, effective, wave, validator)
            if mv is not None:
                wave.append(mv)
                wave_ep[pid] = mv[2]   # record endpoint for subsequent pieces

        return wave

    # -- Move finder -----------------------------------------------------------

    def _find_move(
        self,
        pid:       int,
        start:     Vec2,
        goal:      Vec2,
        static:    Dict[int, Vec2],
        wave:      List[_Move],
        validator: Optional[Callable],
    ) -> Optional[_Move]:
        """
        Find the best single move for pid toward goal that passes _safe().

        Strategy
        --------
        1. Direct path at scales 1.0, 0.75, 0.5, 0.25.
        2. If all direct scales fail, try bypass waypoints perpendicular to the
           path at the nearest blocker (both sides x 3 clearance multiples).
        """
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        d  = math.hypot(dx, dy)
        if d <= _ARRIVE:
            return None

        # 1. Direct path (four scale levels)
        for scale in (1.0, 0.75, 0.5, 0.25):
            end = (start[0] + dx * scale, start[1] + dy * scale)
            seg = _dist(start, end)
            # Skip micro-segments unless docking near the final goal
            if seg < _MIN_SEG and _dist(end, goal) > _DOCK:
                continue
            if not _in_bounds(end):
                continue
            if validator and not validator(pid, end[0], end[1]):
                continue
            if self._safe(start, end, static, wave):
                return (
                    pid, start, end,
                    f'direct s={scale:.2f} rem={_dist(end, goal):.0f}mm',
                )

        # 2. Bypass around nearest path blocker
        blocker = self._nearest_blocker(start, goal, static)
        if blocker is None:
            # Direct scales failed but no clear static blocker found �
            # wave segment conflicts are responsible; skip this piece.
            return None

        for wp in self._bypass_wps(start, goal, blocker):
            if not _in_bounds(wp):
                continue
            if validator and not validator(pid, wp[0], wp[1]):
                continue
            if self._safe(start, wp, static, wave):
                return (
                    pid, start, wp,
                    f'bypass ({blocker[0]:.0f},{blocker[1]:.0f})',
                )

        return None

    # -- THE collision gate ----------------------------------------------------

    def _safe(
        self,
        start:  Vec2,
        end:    Vec2,
        static: Dict[int, Vec2],
        wave:   List[_Move],
    ) -> bool:
        """
        Return True only if the move start->end is fully collision-free.

        Checks (ALL must pass):
          a) Swept capsule of start->end does not pass within _CLEARANCE of any
             static piece (point-to-segment distance check).
          b) Swept capsule does not intersect any committed wave segment
             (segment-to-segment distance check).
          c) Endpoint does not land within _CLEARANCE of any committed
             wave endpoint.

        Note: (a) covers endpoint checks against static pieces because
        _pt_seg_dist includes the endpoints of the segment.
        """
        cl = _CLEARANCE

        # (a) against static pieces
        for p in static.values():
            if _pt_seg_dist(p, start, end) < cl:
                return False

        # (b/c) against committed wave moves
        for _, ms, me, _ in wave:
            if _seg_seg_dist(start, end, ms, me) < cl:
                return False
            if _dist(end, me) < cl:
                return False

        return True

    # -- Helpers ---------------------------------------------------------------

    def _nearest_blocker(
        self,
        start:  Vec2,
        goal:   Vec2,
        static: Dict[int, Vec2],
    ) -> Optional[Vec2]:
        """Return the position of the closest piece blocking path start->goal."""
        best: Optional[Tuple[float, Vec2]] = None
        for p in static.values():
            if _pt_seg_dist(p, start, goal) < _CLEARANCE:
                d = _dist(start, p)
                if best is None or d < best[0]:
                    best = (d, p)
        return best[1] if best else None

    def _bypass_wps(self, start: Vec2, goal: Vec2, blocker: Vec2) -> List[Vec2]:
        """
        Generate bypass waypoints perpendicular to start->goal at the blocker,
        on both sides at 1.5x, 2.5x, and 3.5x the planning clearance.
        """
        dx, dy = goal[0] - start[0], goal[1] - start[1]
        mag = math.hypot(dx, dy)
        if mag < 1e-6:
            return []
        px, py = -dy / mag, dx / mag   # left perpendicular unit vector
        return [
            (blocker[0] + sx * _CLEARANCE * f, blocker[1] + sy * _CLEARANCE * f)
            for (sx, sy) in ((px, py), (-px, -py))
            for f in (1.5, 2.5, 3.5)
            if _in_bounds(
                (blocker[0] + sx * _CLEARANCE * f, blocker[1] + sy * _CLEARANCE * f)
            )
        ]

    # -- Detour finder ---------------------------------------------------------

    def _find_detour(
        self,
        remaining: List[int],
        pos:       Dict[int, Vec2],
        goals:     Dict[int, Vec2],
        validator: Optional[Callable],
    ) -> Optional[_Move]:
        """
        Move the single piece that is blocking the most stuck pieces.

        For each stuck piece, any other piece whose current position falls
        within planning clearance of that piece's direct path to its goal
        is counted as a blocker vote.  The piece with the most votes is
        relocated to the best escape position -- scored by how many voters
        it actually unblocks plus proximity to its own goal (prevents cycling).
        """
        votes: Dict[int, int]  = {}
        bpos:  Dict[int, Vec2] = {}

        for pid in remaining:
            s, g = pos[pid], goals[pid]
            for oid, op in pos.items():
                if oid != pid and _pt_seg_dist(op, s, g) < _CLEARANCE:
                    votes[oid] = votes.get(oid, 0) + 1
                    bpos[oid]  = op

        if not votes:
            return None

        for bid in sorted(votes, key=lambda b: -votes[b]):
            bp     = bpos[bid]
            static = {pid: p for pid, p in pos.items() if pid != bid}
            voters = {
                pid for pid in remaining
                if _pt_seg_dist(bp, pos[pid], goals[pid]) < _CLEARANCE
            }
            esc = self._escape(bid, bp, static, validator,
                               voters=voters, pos=pos, goals=goals)
            if esc is not None:
                return (bid, bp, esc, f'detour votes={votes[bid]}')

        return None

    def _escape(
        self,
        pid:       int,
        start:     Vec2,
        static:    Dict[int, Vec2],
        validator: Optional[Callable],
        voters:    Optional[set]             = None,
        pos:       Optional[Dict[int, Vec2]] = None,
        goals:     Optional[Dict[int, Vec2]] = None,
    ) -> Optional[Vec2]:
        """
        Find the best escape position in 8 compass directions x 3 distances.

        Scoring (highest wins):
          primary   -- number of voting stuck pieces whose path is cleared
          secondary -- proximity to this piece's own goal (prevents cycling by
                       making the blocker converge rather than oscillate)
        """
        own_goal   = (goals.get(pid) if goals else None)
        best_cand:  Optional[Vec2]      = None
        best_score: Tuple[int, float]   = (-1, float('inf'))  # (unblocked, dist_to_goal)

        for deg in range(0, 360, 45):
            rad = math.radians(deg)
            cx, cy = math.cos(rad), math.sin(rad)
            for f in (1.5, 2.5, 3.5):
                cand = (start[0] + cx * _CLEARANCE * f, start[1] + cy * _CLEARANCE * f)
                if not _in_bounds(cand):
                    continue
                if validator and not validator(pid, cand[0], cand[1]):
                    continue
                if not self._safe(start, cand, static, []):
                    continue

                # How many voters does this candidate actually unblock?
                if voters and pos and goals:
                    unblocked = sum(
                        1 for vid in voters
                        if _pt_seg_dist(cand, pos[vid], goals[vid]) >= _CLEARANCE
                    )
                else:
                    unblocked = 0

                dist_to_goal = _dist(cand, own_goal) if own_goal else 0.0
                score: Tuple[int, float] = (unblocked, -dist_to_goal)  # max unblocked, min dist

                if score > best_score:
                    best_score = score
                    best_cand  = cand

        return best_cand

    # -- Sea-parting deadlock relief -------------------------------------------

    def _part_the_sea(
        self,
        remaining: List[int],
        pos:       Dict[int, Vec2],
        goals:     Dict[int, Vec2],
        validator: Optional[Callable],
    ) -> Optional[List[List[_Move]]]:
        """
        Multi-wave corridor clearing for the "walled-in back-rank" deadlock.

        Returns a list of waves (each wave is a List[_Move]) that, when
        executed in order, clear the direct path for the most-stuck remaining
        piece.  Returns None if no clearing plan can be made.

        Why multiple waves?
        -------------------
        The old single-wave approach required EVERY corridor blocker to find an
        escape simultaneously.  When blockers are packed tightly (e.g. 8 pawns
        on a rank) each one's escape is blocked by its neighbours, so the
        `all_moved` gate would fail and the algorithm gave up, leaving the
        stuck piece to oscillate via the single-blocker detour.

        This version uses a simulated position snapshot (`sim_pos`) to plan
        up to _MAX_SEA_ROUNDS clearing waves in advance:

          Round 1  Outermost blockers (farthest from stuck piece, most free
                   space) are moved first.  Partial progress is fine � pieces
                   that can't escape yet are skipped without aborting the wave.
          Round 2  With round-1 pieces out of the way, middle blockers now
                   have room and can be moved.
          ...      Continues until the corridor is fully clear or no further
                   progress is possible.

        All planned waves are returned at once so the caller can emit them
        back-to-back.  This prevents the main loop from returning displaced
        pieces to their goals (re-blocking the corridor) between clearing waves.

        Trigger condition: at least one corridor blocker is settled (within
        2 � _DOCK of its own goal).  Purely un-settled blockers are handled
        better by the normal _build_wave / _find_detour flow.
        """
        for pid in sorted(remaining, key=lambda p: -_dist(pos[p], goals[p])):
            s, g = pos[pid], goals[pid]
            if _dist(s, g) <= _ARRIVE:
                continue

            def _in_corridor(p: Vec2, cur_s: Vec2, cur_g: Vec2) -> bool:
                return _pt_seg_dist(p, cur_s, cur_g) < _CLEARANCE

            def _corridor_blockers(cur_pos: Dict[int, Vec2]) -> List[Tuple[int, Vec2]]:
                """Blockers in corridor sorted outermost (farthest from s) first."""
                return sorted(
                    [
                        (oid, cur_pos[oid])
                        for oid in cur_pos
                        if oid != pid and _in_corridor(cur_pos[oid], s, g)
                    ],
                    key=lambda t: -_dist(s, t[1]),  # outermost first
                )

            initial_blockers = _corridor_blockers(pos)
            if not initial_blockers:
                continue

            # Trigger only when at least one blocker is settled near its goal.
            if not any(
                (oid not in goals) or (_dist(op, goals[oid]) <= _DOCK * 2)
                for oid, op in initial_blockers
            ):
                continue

            # ---- Plan the clearing cascade using simulated positions --------
            # sim_pos tracks where each piece *will* be after planned waves.
            # The stuck piece (pid) is excluded from static checks � we are
            # clearing a lane FOR it, so its current position must not block
            # the escape candidates we're evaluating.
            waves:   List[List[_Move]] = []
            sim_pos: Dict[int, Vec2]   = dict(pos)

            for _round in range(_MAX_SEA_ROUNDS):
                cb = _corridor_blockers(sim_pos)
                if not cb:
                    break   # corridor fully clear � done

                wave:    List[_Move]     = []
                wave_ep: Dict[int, Vec2] = {}
                moved_any = False

                for oid, op in cb:              # outermost first
                    moved = False
                    for deg in range(0, 360, 45):
                        if moved:
                            break
                        rad = math.radians(deg)
                        cx, cy = math.cos(rad), math.sin(rad)
                        for f in (1.5, 2.5, 3.5, 4.5, 5.5):
                            cand = (
                                op[0] + cx * _CLEARANCE * f,
                                op[1] + cy * _CLEARANCE * f,
                            )
                            if not _in_bounds(cand):
                                continue
                            if validator and not validator(oid, cand[0], cand[1]):
                                continue
                            # Candidate must lie outside the corridor
                            if _in_corridor(cand, s, g):
                                continue
                            # Static: all pieces except this mover, pid, and
                            # pieces already committed to this wave
                            static_pts: Dict[int, Vec2] = {
                                xid: sim_pos[xid]
                                for xid in sim_pos
                                if xid != oid and xid != pid
                                and xid not in wave_ep
                            }
                            if self._safe(op, cand, static_pts, wave):
                                wave.append((oid, op, cand, 'part_sea'))
                                wave_ep[oid] = cand
                                moved = True
                                moved_any = True
                                break
                    # Piece that can't move yet is simply skipped �
                    # a later round will handle it once neighbours have moved.

                if not moved_any:
                    break   # no progress possible � stop planning

                waves.append(wave)
                for oid, _, cand, _ in wave:
                    sim_pos[oid] = cand   # advance sim for next round

            if waves:
                log.debug(
                    '[ECP] part_sea: %d clearing waves for pid=0x%02X'
                    ' path=(%.0f,%.0f)->(%.0f,%.0f)',
                    len(waves), pid, s[0], s[1], g[0], g[1],
                )
                return waves

        return None

    # -- Make-way deadlock resolution -----------------------------------------

    def _make_way(
        self,
        remaining: List[int],
        pos:       Dict[int, Vec2],
        goals:     Dict[int, Vec2],
        validator: Optional[Callable],
    ) -> Optional[List[List[_Move]]]:
        """Give one stuck piece sole priority to reach its goal.

        Selects the piece in ``remaining`` that is furthest from its goal as
        the priority piece.  All pieces blocking its direct path are parked in
        safe positions outside that corridor.  Cascade-blocked pieces are
        handled in successive rounds (up to _MW_PARK_ROUNDS).  Once the
        corridor is clear the priority piece is sent directly to its goal.

        Returns a list of clearing waves (each a List[_Move]) followed by the
        priority move wave, or None if clearing is not possible.
        """
        # Pick the stuck piece furthest from its goal as priority
        priority_pid = max(remaining, key=lambda p: _dist(pos[p], goals[p]))
        p_start      = pos[priority_pid]
        p_goal       = goals[priority_pid]

        log.debug('[ECP] _make_way: priority=0x%02X path=(%.0f,%.0f)->(%.0f,%.0f)',
                  priority_pid, p_start[0], p_start[1], p_goal[0], p_goal[1])

        sim_pos:    Dict[int, Vec2]   = dict(pos)
        all_waves:  List[List[_Move]] = []
        yield_used: int               = 0

        for _round in range(_MW_PARK_ROUNDS):
            blockers = [
                oid for oid in sim_pos
                if oid != priority_pid
                and _pt_seg_dist(sim_pos[oid], p_start, p_goal) < _CLEARANCE
            ]
            if not blockers:
                break  # corridor is clear

            wave:    List[_Move]     = []
            wave_ep: Dict[int, Vec2] = {}
            moved_any = False

            # Sort: nearest blocker first (clear the entry to the path first)
            for bid in sorted(blockers, key=lambda b: _dist(sim_pos[b], p_start)):
                # Static = all pieces except this blocker.
                # NOTE: priority piece IS included so its position blocks escape
                # paths that would physically collide with it.
                # Pieces already committed to this wave contribute their endpoints.
                static: Dict[int, Vec2] = {
                    xid: (wave_ep[xid] if xid in wave_ep else sim_pos[xid])
                    for xid in sim_pos
                    if xid != bid
                }
                park = self._park_escape(
                    bid, sim_pos[bid], p_start, p_goal,
                    static, validator, wave, goals,
                )
                if park is not None:
                    wave.append((bid, sim_pos[bid], park, f'mw_park r={_round}'))
                    wave_ep[bid] = park
                    moved_any = True
                # Piece that cannot park yet is skipped; a later round handles it
                # once its neighbours have moved out of the way.

            if not moved_any:
                # The priority piece itself may be blocking blockers' escape routes.
                # Allow it to yield � move further from its goal to create space.
                if yield_used >= _MW_YIELD_MAX:
                    log.warning('[ECP] _make_way: no blocker could park at round %d '
                                '(yield budget exhausted)', _round)
                    return None
                yield_pos = self._yield_priority_piece(
                    priority_pid, p_start, p_goal, blockers, sim_pos, goals, validator,
                )
                if yield_pos is None:
                    log.warning('[ECP] _make_way: no blocker could park at round %d '
                                'and priority piece cannot yield', _round)
                    return None
                yield_wave = [(priority_pid, sim_pos[priority_pid], yield_pos,
                               f'mw_yield r={_round}')]
                all_waves.append(yield_wave)
                sim_pos[priority_pid] = yield_pos
                p_start = yield_pos   # corridor now starts from new position
                yield_used += 1
                log.debug('[ECP] _make_way: priority 0x%02X yielded to (%.0f,%.0f) '
                          '(yield %d/%d)',
                          priority_pid, yield_pos[0], yield_pos[1],
                          yield_used, _MW_YIELD_MAX)
                continue  # retry blocker clearing with priority at new position

            all_waves.append(wave)
            for bid, _, park, _ in wave:
                sim_pos[bid] = park

        # Verify the corridor is now fully clear
        still_blocking = [
            oid for oid in sim_pos
            if oid != priority_pid
            and _pt_seg_dist(sim_pos[oid], p_start, p_goal) < _CLEARANCE
        ]
        if still_blocking:
            log.warning('[ECP] _make_way: %d blocker(s) remain after %d rounds',
                        len(still_blocking), _MW_PARK_ROUNDS)
            return None

        # Append the priority piece's direct move to its goal
        static_final: Dict[int, Vec2] = {
            xid: sim_pos[xid] for xid in sim_pos if xid != priority_pid
        }
        if self._safe(p_start, p_goal, static_final, []):
            all_waves.append([(priority_pid, p_start, p_goal, 'mw_move')])
        else:
            # Path is geometrically clear but endpoint conflicts remain;
            # still return the clearing waves � the main loop will handle the rest
            log.debug('[ECP] _make_way: corridor clear but endpoint blocked; '
                      'returning clearing waves only')

        return all_waves or None

    def _yield_priority_piece(
        self,
        priority_pid: int,
        p_start:      Vec2,
        p_goal:       Vec2,
        blockers:     List[int],
        sim_pos:      Dict[int, Vec2],
        goals:        Dict[int, Vec2],
        validator:    Optional[Callable],
    ) -> Optional[Vec2]:
        """Find a position for the priority piece that unblocks corridor blockers.

        The priority piece is willing to move *further* from its goal in order
        to create space for blockers to escape.  We search 8 compass directions
        � 4 distances and, for each candidate, run a quick lookahead: simulate
        the priority piece at that position and count how many blockers can now
        find a valid park spot.

        Scoring (highest wins):
          primary   � number of blockers now able to park
          secondary � proximity to own goal (minimise regression)

        Returns the best candidate, or None if none unblocks at least one blocker.
        """
        static_for_yield: Dict[int, Vec2] = {
            xid: sim_pos[xid] for xid in sim_pos if xid != priority_pid
        }

        best_cand:  Optional[Vec2]      = None
        best_score: Tuple[int, float]   = (0, float('inf'))  # (unblocked, dist_to_goal)

        for deg in range(0, 360, 45):
            rad = math.radians(deg)
            cx, cy = math.cos(rad), math.sin(rad)
            for f in (1.5, 2.5, 3.5, 4.5):
                cand = (
                    p_start[0] + cx * _CLEARANCE * f,
                    p_start[1] + cy * _CLEARANCE * f,
                )
                if not _in_bounds(cand):
                    continue
                if validator and not validator(priority_pid, cand[0], cand[1]):
                    continue
                if not self._safe(p_start, cand, static_for_yield, []):
                    continue

                # Lookahead: how many blockers can park if priority is at cand?
                sim_yield = dict(sim_pos)
                sim_yield[priority_pid] = cand
                unblocked = 0
                for bid in blockers:
                    static_b: Dict[int, Vec2] = {
                        xid: sim_yield[xid]
                        for xid in sim_yield
                        if xid != bid
                    }
                    # Corridor is now cand ? p_goal
                    if self._park_escape(
                        bid, sim_pos[bid], cand, p_goal,
                        static_b, validator, [], goals,
                    ) is not None:
                        unblocked += 1

                dist_to_goal = _dist(cand, p_goal)
                score: Tuple[int, float] = (unblocked, -dist_to_goal)
                if score > best_score:
                    best_score = score
                    best_cand  = cand

        if best_score[0] == 0:
            return None   # no candidate unblocked even one blocker � pointless to yield
        return best_cand

    def _park_escape(
        self,
        pid:        int,
        start:      Vec2,
        path_start: Vec2,
        path_goal:  Vec2,
        static:     Dict[int, Vec2],
        validator:  Optional[Callable],
        wave:       List[_Move],
        goals:      Optional[Dict[int, Vec2]] = None,
    ) -> Optional[Vec2]:
        """Find a safe parking position outside the priority path.

        Searches 8 compass directions � 5 distances.  The candidate must lie
        outside the priority corridor (centre-to-corridor >= _CLEARANCE) and
        pass the standard _safe() check.  Among valid candidates the one
        closest to the piece's own goal is preferred (prevents oscillation).
        """
        own_goal:   Optional[Vec2] = goals.get(pid) if goals else None
        best_cand:  Optional[Vec2] = None
        best_score: float          = float('inf')  # minimise dist to own goal

        for deg in range(0, 360, 45):
            rad = math.radians(deg)
            cx, cy = math.cos(rad), math.sin(rad)
            for f in (1.5, 2.5, 3.5, 4.5, 5.5):
                cand = (
                    start[0] + cx * _CLEARANCE * f,
                    start[1] + cy * _CLEARANCE * f,
                )
                if not _in_bounds(cand):
                    continue
                if validator and not validator(pid, cand[0], cand[1]):
                    continue
                # Must lie *outside* the priority corridor
                if _pt_seg_dist(cand, path_start, path_goal) < _CLEARANCE:
                    continue
                if not self._safe(start, cand, static, wave):
                    continue
                score = _dist(cand, own_goal) if own_goal else 0.0
                if score < best_score:
                    best_score = score
                    best_cand  = cand

        return best_cand

    # -- Chain merge (priority 4) ----------------------------------------------

    def _chain_merge(
        self,
        commands:  List[MoveCommand],
        wstarts:   Dict[Tuple[int, int], Vec2],
        wpos_snap: Dict[int, Dict[int, Vec2]],
    ) -> List[MoveCommand]:
        """
        Merge consecutive waves into a single command when safe to do so.

        Two adjacent commands A (wave N) and B (wave N+1) for the same piece P
        are merged when ALL of the following hold:

          1. Wave N+1 contains only piece P -- no other piece is moving at N+1,
             so there are no new dynamic obstacles introduced by merging.
          2. The bearing change between segment A and segment B is <=30 degrees --
             the two segments are roughly collinear.
          3. The merged straight path (P's wave-N start -> B's target) clears
             all other pieces at their pre-wave-N snapshot positions.

        The merged command keeps wave N's sequence number; wave N+1 is dropped.
        Duration is the sum of both original durations.
        """
        if not commands:
            return commands

        # Group by wave sequence number
        waves: Dict[int, List[MoveCommand]] = {}
        for cmd in commands:
            waves.setdefault(cmd.sequence_num, []).append(cmd)

        seq_nums = sorted(waves.keys())
        absorbed: set = set()   # (seq, pid) pairs consumed by a merge
        result:   List[MoveCommand] = []

        for seq in seq_nums:
            for cmd in sorted(waves[seq], key=lambda c: c.piece_id):
                if (seq, cmd.piece_id) in absorbed:
                    continue

                merged  = cmd
                cur_seq = seq

                # Try to extend the chain forward
                while True:
                    nxt = cur_seq + 1
                    if nxt not in waves:
                        break
                    # Condition 1: wave nxt must contain only this piece
                    if len(waves[nxt]) != 1:
                        break
                    nxt_cmd = waves[nxt][0]
                    if nxt_cmd.piece_id != merged.piece_id:
                        break
                    if (nxt, nxt_cmd.piece_id) in absorbed:
                        break

                    pid = merged.piece_id

                    # Start of the original chain (position before wave seq)
                    chain_start = wstarts.get((seq, pid))
                    if chain_start is None:
                        break

                    end_a = (merged.target_x_mm, merged.target_y_mm)
                    end_b = (nxt_cmd.target_x_mm, nxt_cmd.target_y_mm)

                    # Condition 2: direction similarity check
                    ax, ay = end_a[0] - chain_start[0], end_a[1] - chain_start[1]
                    bx, by = end_b[0] - end_a[0],       end_b[1] - end_a[1]
                    ma, mb = math.hypot(ax, ay), math.hypot(bx, by)
                    if ma < 1e-6 or mb < 1e-6:
                        break
                    dot = max(-1.0, min(1.0, (ax * bx + ay * by) / (ma * mb)))
                    if math.degrees(math.acos(dot)) > 30.0:
                        break

                    # Condition 3: safety of the merged straight path.
                    # Check against:
                    #   a) pieces NOT moving in wave seq  -> use pre-wave snapshot
                    #   b) segments of OTHER pieces moving in wave seq
                    #      (the merged command executes simultaneously with them)
                    snap          = wpos_snap.get(seq, {})
                    moving_in_seq = {c.piece_id for c in waves[seq] if c.piece_id != pid}
                    static        = {
                        oid: p for oid, p in snap.items()
                        if oid != pid and oid not in moving_in_seq
                    }
                    # Build wave-seq segments for other concurrent pieces
                    wave_segs: List[_Move] = [
                        (
                            c.piece_id,
                            wstarts.get(
                                (seq, c.piece_id),
                                snap.get(c.piece_id, (c.target_x_mm, c.target_y_mm)),
                            ),
                            (c.target_x_mm, c.target_y_mm),
                            '',
                        )
                        for c in waves[seq]
                        if c.piece_id != pid
                    ]
                    if not self._safe(chain_start, end_b, static, wave_segs):
                        break

                    # All conditions pass -- merge
                    absorbed.add((nxt, pid))
                    merged = MoveCommand(
                        piece_id     = pid,
                        target_x_mm  = end_b[0],
                        target_y_mm  = end_b[1],
                        duration_ms  = merged.duration_ms + nxt_cmd.duration_ms,
                        sequence_num = merged.sequence_num,
                        uci_move     = merged.uci_move or nxt_cmd.uci_move,
                        planner_debug= (merged.planner_debug or '') + '+chain',
                    )
                    cur_seq = nxt

                result.append(merged)

        return sorted(result, key=lambda c: (c.sequence_num, c.piece_id))
