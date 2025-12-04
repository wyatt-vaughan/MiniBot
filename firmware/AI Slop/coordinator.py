import math
import random
import time
import tkinter as tk
from abc import ABC, abstractmethod
from typing import List, Dict, Tuple, Optional
import threading

# --- Constants ---
BOARD_SIZE = 440
SIDE_ZONE = 100
TOTAL_X_MIN = -SIDE_ZONE
TOTAL_X_MAX = BOARD_SIZE + SIDE_ZONE
TOTAL_Y_MIN = 0
TOTAL_Y_MAX = BOARD_SIZE
PIECE_RADIUS = 15
SAFETY_RADIUS = 20
MOVE_SPEED = 200
ROTATE_SPEED = 1.2
ROTATE_MAX_ERROR = 0.05
UPDATE_INTERVAL = 0.05  # 20 FPS for smoother UI responsiveness

# --- Placeholder ESP-NOW send ---
def send_espnow_command(cmd, piece_id, *args):
    """Send command to piece via ESP-NOW. Only used in real mode."""
    print(f"[ESPNow] {cmd} -> piece_{piece_id} {args}")

# --- Data classes ---
class Piece:
    """Represents a chess piece with ID, type, and color."""
    _next_index = 0

    def __init__(self, name: str, color: str):
        self.name = name
        self.color = color
        self.index = Piece._next_index
        Piece._next_index += 1

    @staticmethod
    def reset_index():
        """Reset the global piece index counter."""
        Piece._next_index = 0


class PieceState:
    """Runtime state of a piece (position, angle, goals)."""

    def __init__(self, piece: Piece, position: Tuple[float, float] = (0, 0), angle: float = 0.0):
        self.piece = piece
        self.position = position
        self.angle = angle
        self.at_target = False
        self.reverse_mode = False


class Motion:
    """A single motion command for a piece."""

    def __init__(self, kind: str, params: Tuple, duration: float):
        self.kind = kind  # "rotate", "line", "arc"
        self.params = params
        self.duration = duration


class MotionPlan:
    """Complete plan for moving a piece to a target."""

    def __init__(self, piece: Piece, motions: List[Motion], target_pos: Tuple[float, float]):
        self.piece = piece
        self.motions = motions
        self.target_pos = target_pos
        self.total_time = sum(m.duration for m in motions)


# --- Motion Planner Interface ---
class MotionPlanner(ABC):
    """Abstract base class for motion planning strategies."""

    @abstractmethod
    def plan_moves(
        self, piece_states: Dict[Piece, PieceState], targets: Dict[Piece, Tuple[float, float]]
    ) -> Dict[Piece, MotionPlan]:
        """
        Generate motion plans for all pieces.

        Args:
            piece_states: Current state of each piece
            targets: Target positions for each piece

        Returns:
            Dictionary mapping pieces to their motion plans
        """
        pass


class BasicMotionPlanner(MotionPlanner):
    """
    Basic motion planner with collision avoidance.
    Moves each piece to its target using straight lines and detours.
    """

    def __init__(self, board: "ChessBoard"):
        self.board = board
        self.piece_order = []  # Store the order pieces are planned

    def plan_moves(
        self, piece_states: Dict[Piece, PieceState], targets: Dict[Piece, Tuple[float, float]]
    ) -> Dict[Piece, MotionPlan]:
        """Generate motion plans for all pieces to reach their targets."""
        plans = {}
        for piece, state in piece_states.items():
            target = targets[piece]
            motions = self._plan_piece_path(state, target)
            plans[piece] = MotionPlan(piece, motions, target)
        return plans

    def _plan_piece_path(self, state: PieceState, target: Tuple[float, float]) -> List[Motion]:
        """Plan motion sequence for a single piece."""
        motions = []
        current_pos = state.position
        current_angle = state.angle
        max_attempts = 5

        for attempt in range(max_attempts):
            dx, dy = target[0] - current_pos[0], target[1] - current_pos[1]
            dist = math.hypot(dx, dy)
            if dist < 1e-3:
                break

            # Determine target angle
            target_angle = math.atan2(dy, dx)
            diff = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
            reverse = abs(diff) > math.pi / 2
            if reverse:
                target_angle = (target_angle + math.pi) % (2 * math.pi)
                diff = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

            # Add rotation
            rotate_time = abs(diff) / ROTATE_SPEED
            motions.append(Motion("rotate", (diff,), rotate_time))
            current_angle = target_angle

            # Try straight path
            if self._try_path_clear(current_pos, current_angle, dist, state.piece, reverse):
                line_time = dist / MOVE_SPEED
                motions.append(Motion("line", (dist,), line_time))
                current_pos = target
                break
            else:
                # Try detour
                detour_success, detour_time, new_pos, new_angle = self._simulate_detour(
                    state.piece, current_pos, current_angle
                )
                if detour_success:
                    motions.append(Motion("arc", (SAFETY_RADIUS * 3, math.pi / 3), detour_time))
                    current_pos, current_angle = new_pos, new_angle
                else:
                    # Try to move blockers
                    blockers = self._detect_nearby_blockers(current_pos, state.piece)
                    if not any(self._simulate_move_blocker(b, state.piece) for b in blockers):
                        break

        return motions

    def _try_path_clear(self, pos: Tuple[float, float], angle: float, distance: float, piece: Piece, reverse: bool = False) -> bool:
        """Check if a straight path is clear."""
        dx, dy = math.cos(angle), math.sin(angle)
        if reverse:
            distance *= -1
        steps = max(1, int(abs(distance) / SAFETY_RADIUS))
        for i in range(steps + 1):
            check_pos = (pos[0] + dx * distance * i / steps, pos[1] + dy * distance * i / steps)
            if self.board.is_collision(check_pos, piece):
                return False
        return True

    def _simulate_detour(self, piece: Piece, pos: Tuple[float, float], angle: float) -> Tuple[bool, float, Tuple[float, float], float]:
        """Try to find a detour path around obstacles."""
        for side in [+1, -1]:
            r, arc_angle, steps = SAFETY_RADIUS * 3, side * math.pi / 3, 10
            valid = True
            for i in range(steps + 1):
                ang = angle + arc_angle * i / steps
                x = pos[0] - math.sin(angle) * r + math.sin(ang) * r
                y = pos[1] + math.cos(angle) * r - math.cos(ang) * r
                if self.board.is_collision((x, y), piece):
                    valid = False
                    break
            if valid:
                arc_len = r * abs(arc_angle)
                arc_time = arc_len / MOVE_SPEED
                new_pos = (
                    pos[0] - math.sin(angle) * r + math.sin(angle + arc_angle) * r,
                    pos[1] + math.cos(angle) * r - math.cos(angle + arc_angle) * r,
                )
                new_ang = angle + arc_angle
                return True, arc_time, new_pos, new_ang
        return False, 0.0, pos, angle

    def _detect_nearby_blockers(self, pos: Tuple[float, float], piece: Piece) -> List[Piece]:
        """Find pieces that are blocking the current piece."""
        return [p for p in self.board.pieces if p != piece and self.board.distance(pos, self.board.piece_states[p].position) < 2 * SAFETY_RADIUS]

    def _simulate_move_blocker(self, blocker: Piece, mover: Piece) -> bool:
        """Attempt to move a blocking piece out of the way."""
        blocker_state = self.board.piece_states[blocker]
        mover_state = self.board.piece_states[mover]
        angle = math.atan2(blocker_state.position[1] - mover_state.position[1], blocker_state.position[0] - mover_state.position[0])
        for side in [+1, -1]:
            offset = angle + side * math.pi / 2
            new_pos = (
                blocker_state.position[0] + math.cos(offset) * SAFETY_RADIUS * 2,
                blocker_state.position[1] + math.sin(offset) * SAFETY_RADIUS * 2,
            )
            if not self.board.is_collision(new_pos, blocker):
                return True
        return False


class IntelligentMotionPlanner(MotionPlanner):
    """
    Advanced motion planner with strategic piece ordering and collision resolution.
    
    Strategy:
    1. Move back rank pieces first (sorted by move distance)
    2. Move pawns to the side to clear center paths
    3. Use curved paths around obstacles
    4. Try different collision resolution strategies
    5. Optimize move order for speed
    """

    def __init__(self, board: "ChessBoard"):
        self.board = board
        self.piece_order = []
        self.best_plan = None
        self.best_time = float('inf')

    def plan_moves(
        self, piece_states: Dict[Piece, PieceState], targets: Dict[Piece, Tuple[float, float]]
    ) -> Dict[Piece, MotionPlan]:
        """Generate optimized motion plans using intelligent ordering and strategies."""
        # Calculate strategic piece order
        piece_order = self._calculate_piece_order(piece_states, targets)
        
        # Try multiple ordering strategies and collision resolution approaches
        strategies = [
            (piece_order, "straight"),  # Original order, prefer straight paths
            (piece_order, "curve"),     # Original order, prefer curved detouring
            (piece_order, "combined"),  # Original order, try all strategies
        ]
        
        # Try slight randomization of order for optimization
        for _ in range(2):
            randomized = piece_order.copy()
            random.shuffle(randomized)
            strategies.append((randomized, "combined"))
        
        best_plans = None
        best_time = float('inf')
        
        # Evaluate each strategy
        for order, strategy in strategies:
            plans = self._plan_with_strategy(order, piece_states, targets, strategy)
            total_time = sum(plan.total_time for plan in plans.values())
            
            if total_time < best_time:
                best_time = total_time
                best_plans = plans
                print(f"[PLANNER] Found better plan with strategy '{strategy}': {total_time:.2f}s")
        
        return best_plans

    def _calculate_piece_order(self, piece_states: Dict[Piece, PieceState], targets: Dict[Piece, Tuple[float, float]]) -> List[Piece]:
        """Calculate optimal piece planning order."""
        # Separate pieces by type
        back_rank = []
        pawns = []
        
        for piece in self.board.pieces:
            if piece.name == 'P':
                pawns.append(piece)
            else:
                back_rank.append(piece)
        
        # Sort back rank by move distance (shortest first)
        back_rank_with_dist = [
            (piece, self.board.distance(piece_states[piece].position, targets[piece]))
            for piece in back_rank
        ]
        back_rank_with_dist.sort(key=lambda x: x[1])
        back_rank_sorted = [p[0] for p in back_rank_with_dist]
        
        # Sort pawns by distance to side (moving them to edges first)
        pawns_with_dist = [
            (piece, min(piece_states[piece].position[0], BOARD_SIZE - piece_states[piece].position[0]))
            for piece in pawns
        ]
        pawns_with_dist.sort(key=lambda x: -x[1])  # Move innermost pawns first
        pawns_sorted = [p[0] for p in pawns_with_dist]
        
        # Return order: back rank pieces first, then pawns
        return back_rank_sorted + pawns_sorted

    def _plan_with_strategy(self, piece_order: List[Piece], piece_states: Dict[Piece, PieceState], 
                           targets: Dict[Piece, Tuple[float, float]], strategy: str) -> Dict[Piece, MotionPlan]:
        """Plan moves for all pieces using a specific strategy."""
        plans = {}
        
        # Create copies of piece states for planning (to avoid modifying originals during blocker movements)
        planning_states = {}
        for piece, state in piece_states.items():
            planning_states[piece] = PieceState(piece, state.position, state.angle)
        
        for piece in piece_order:
            state = planning_states[piece]
            target = targets[piece]
            motions = self._plan_piece_path_intelligent(state, target, strategy, plans, planning_states)
            plans[piece] = MotionPlan(piece, motions, target)
        
        return plans

    def _plan_piece_path_intelligent(self, state: PieceState, target: Tuple[float, float], 
                                     strategy: str, planned_pieces: Dict[Piece, MotionPlan],
                                     all_states: Dict[Piece, PieceState]) -> List[Motion]:
        """Plan motion using intelligent collision avoidance."""
        motions = []
        current_pos = state.position
        current_angle = state.angle
        max_attempts = 8  # More attempts for intelligent planning
        
        for attempt in range(max_attempts):
            dx, dy = target[0] - current_pos[0], target[1] - current_pos[1]
            dist = math.hypot(dx, dy)
            if dist < 1e-3:
                break
            
            # Determine target angle
            target_angle = math.atan2(dy, dx)
            diff = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
            reverse = abs(diff) > math.pi / 2
            if reverse:
                target_angle = (target_angle + math.pi) % (2 * math.pi)
                diff = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
            
            # Add rotation
            rotate_time = abs(diff) / ROTATE_SPEED
            motions.append(Motion("rotate", (diff,), rotate_time))
            current_angle = target_angle
            
            # Try strategies in order
            path_found = False
            
            if strategy in ["straight", "combined"]:
                if self._try_path_clear(current_pos, current_angle, dist, state.piece, reverse):
                    line_time = dist / MOVE_SPEED
                    motions.append(Motion("line", (dist,), line_time))
                    current_pos = target
                    path_found = True
                    print(f"[PLANNER] {state.piece.name} -> ({target[0]:.1f}, {target[1]:.1f}) using straight line")
            
            if not path_found and strategy in ["curve", "combined"]:
                # Try aggressive curved detouring
                detour_success, detour_time, new_pos, new_angle = self._aggressive_detour(
                    state.piece, current_pos, current_angle, target
                )
                if detour_success:
                    arc_dist = detour_time * MOVE_SPEED
                    motions.append(Motion("arc", (arc_dist / 2, math.pi / 2.5), detour_time))
                    current_pos, current_angle = new_pos, new_angle
                    path_found = True
                    print(f"[PLANNER] {state.piece.name} -> ({target[0]:.1f}, {target[1]:.1f}) using arc detour to ({new_pos[0]:.1f}, {new_pos[1]:.1f})")
            
            if not path_found:
                # Try moving blockers out of the way
                blockers = self._detect_nearby_blockers(current_pos, state.piece)
                if blockers:
                    # Try to move blockers to the side
                    blocker_moved = False
                    for blocker in blockers:
                        if self._move_blocker_to_side(blocker, all_states):
                            blocker_moved = True
                            break
                    
                    if blocker_moved:
                        continue  # Retry path now that blocker moved
                
                # Last resort: small curve around obstacle
                detour_success, detour_time, new_pos, new_angle = self._gentle_curve_detour(
                    state.piece, current_pos, current_angle
                )
                if detour_success:
                    motions.append(Motion("arc", (SAFETY_RADIUS * 2, math.pi / 4), detour_time))
                    current_pos, current_angle = new_pos, new_angle
                else:
                    # Cannot find path, give up on this attempt
                    break
        
        return motions

    def _aggressive_detour(self, piece: Piece, pos: Tuple[float, float], angle: float, 
                          target: Tuple[float, float]) -> Tuple[bool, float, Tuple[float, float], float]:
        """Try larger curves to navigate around obstacles."""
        for side in [+1, -1]:
            for radius_mult in [2.0, 2.5, 3.0]:
                r = SAFETY_RADIUS * radius_mult
                arc_angle = side * math.pi / 2.5
                
                # Validate arc path
                valid = True
                steps = 15
                for i in range(steps + 1):
                    ang = angle + arc_angle * i / steps
                    x = pos[0] - math.sin(angle) * r + math.sin(ang) * r
                    y = pos[1] + math.cos(angle) * r - math.cos(ang) * r
                    if self.board.is_collision((x, y), piece):
                        valid = False
                        break
                
                if valid:
                    arc_len = r * abs(arc_angle)
                    arc_time = arc_len / MOVE_SPEED
                    new_pos = (
                        pos[0] - math.sin(angle) * r + math.sin(angle + arc_angle) * r,
                        pos[1] + math.cos(angle) * r - math.cos(angle + arc_angle) * r,
                    )
                    new_ang = (angle + arc_angle) % (2 * math.pi)
                    return True, arc_time, new_pos, new_ang
        
        return False, 0.0, pos, angle

    def _gentle_curve_detour(self, piece: Piece, pos: Tuple[float, float], 
                            angle: float) -> Tuple[bool, float, Tuple[float, float], float]:
        """Try gentle curves for minor obstacle avoidance."""
        for side in [+1, -1]:
            r = SAFETY_RADIUS * 1.5
            arc_angle = side * math.pi / 6
            
            valid = True
            steps = 8
            for i in range(steps + 1):
                ang = angle + arc_angle * i / steps
                x = pos[0] - math.sin(angle) * r + math.sin(ang) * r
                y = pos[1] + math.cos(angle) * r - math.cos(ang) * r
                if self.board.is_collision((x, y), piece):
                    valid = False
                    break
            
            if valid:
                arc_len = r * abs(arc_angle)
                arc_time = arc_len / MOVE_SPEED
                new_pos = (
                    pos[0] - math.sin(angle) * r + math.sin(angle + arc_angle) * r,
                    pos[1] + math.cos(angle) * r - math.cos(angle + arc_angle) * r,
                )
                new_ang = (angle + arc_angle) % (2 * math.pi)
                return True, arc_time, new_pos, new_ang
        
        return False, 0.0, pos, angle

    def _move_blocker_to_side(self, blocker: Piece, all_states: Dict[Piece, PieceState]) -> bool:
        """Move a blocking piece to the side to clear a path."""
        blocker_state = all_states[blocker]
        
        # Try moving left or right to clear center path
        for side in [-1, +1]:
            offset_x = side * SAFETY_RADIUS * 3
            new_pos = (blocker_state.position[0] + offset_x, blocker_state.position[1])
            
            # Check if new position is valid
            if (TOTAL_X_MIN + SAFETY_RADIUS < new_pos[0] < TOTAL_X_MAX - SAFETY_RADIUS and
                TOTAL_Y_MIN + SAFETY_RADIUS < new_pos[1] < TOTAL_Y_MAX - SAFETY_RADIUS and
                not self.board.is_collision(new_pos, blocker)):
                # Actually move the blocker to the side
                blocker_state.position = new_pos
                return True
        
        return False

    def _try_path_clear(self, pos: Tuple[float, float], angle: float, distance: float, 
                       piece: Piece, reverse: bool = False) -> bool:
        """Check if a straight path is clear."""
        dx, dy = math.cos(angle), math.sin(angle)
        if reverse:
            distance *= -1
        steps = max(1, int(abs(distance) / SAFETY_RADIUS))
        for i in range(steps + 1):
            check_pos = (pos[0] + dx * distance * i / steps, pos[1] + dy * distance * i / steps)
            if self.board.is_collision(check_pos, piece):
                return False
        return True

    def _detect_nearby_blockers(self, pos: Tuple[float, float], piece: Piece) -> List[Piece]:
        """Find pieces that are blocking the current piece."""
        return [p for p in self.board.pieces if p != piece and 
                self.board.distance(pos, self.board.piece_states[p].position) < 3 * SAFETY_RADIUS]


# --- Main ChessBoard ---
class ChessBoard:
    """Manages board state, pieces, and simulation."""

    def __init__(self):
        self.pieces: List[Piece] = []
        self.piece_states: Dict[Piece, PieceState] = {}
        self.starting_positions: List[Tuple[float, float]] = []
        self.motion_planner: MotionPlanner = IntelligentMotionPlanner(self)
        self.current_plan: Optional[Dict[Piece, MotionPlan]] = None
        self.simulation_mode = False
        self.ui = None

        self.create_pieces()
        self.starting_positions = self.get_starting_positions()
        self.initialize_piece_states()

    def create_pieces(self):
        """Create all 32 chess pieces."""
        Piece.reset_index()
        self.pieces = []
        back_row = ['R', 'N', 'B', 'Q', 'K', 'B', 'N', 'R']
        for name in back_row:
            self.pieces.append(Piece(name, 'white'))
        for _ in range(8):
            self.pieces.append(Piece('P', 'white'))
        for name in back_row:
            self.pieces.append(Piece(name, 'black'))
        for _ in range(8):
            self.pieces.append(Piece('P', 'black'))

    def initialize_piece_states(self):
        """Initialize piece states at starting positions."""
        self.piece_states = {}
        for piece in self.pieces:
            pos = self.starting_positions[piece.index]
            self.piece_states[piece] = PieceState(piece, position=pos, angle=0.0)

    def get_starting_positions(self) -> List[Tuple[float, float]]:
        """Get standard chess starting positions."""
        sq = BOARD_SIZE / 8
        pos = []
        # White back row
        pos += [(i * sq + sq / 2, 0.5 * sq) for i in range(8)]
        # White pawns
        pos += [(i * sq + sq / 2, 1.5 * sq) for i in range(8)]
        # Black back row
        pos += [(i * sq + sq / 2, 7.5 * sq) for i in range(8)]
        # Black pawns
        pos += [(i * sq + sq / 2, 6.5 * sq) for i in range(8)]
        return pos

    def randomize_positions(self):
        """Randomize piece positions without overlap (simulation mode only)."""
        if not self.simulation_mode:
            print("[BOARD] Cannot randomize positions outside simulation mode.")
            return

        placed = []
        for piece in self.pieces:
            attempts = 0
            while attempts < 100:
                x = random.uniform(TOTAL_X_MIN + SAFETY_RADIUS, TOTAL_X_MAX - SAFETY_RADIUS)
                y = random.uniform(TOTAL_Y_MIN + SAFETY_RADIUS, TOTAL_Y_MAX - SAFETY_RADIUS)
                if all(math.hypot(x - px, y - py) > 2 * SAFETY_RADIUS for px, py in placed):
                    self.piece_states[piece].position = (x, y)
                    self.piece_states[piece].angle = random.uniform(0, 2 * math.pi)
                    self.piece_states[piece].at_target = False
                    placed.append((x, y))
                    break
                attempts += 1
            else:
                print(f"[BOARD] Could not place {piece.name} after 100 attempts")

        print(f"[BOARD] Randomized positions for {len(placed)} pieces")
        if self.ui:
            self.ui.refresh_pieces()

    def distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two points."""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def is_collision(self, pos: Tuple[float, float], exclude_piece: Optional[Piece] = None) -> Optional[Piece]:
        """Check if a position collides with any piece."""
        for piece in self.pieces:
            if piece == exclude_piece:
                continue
            if self.distance(pos, self.piece_states[piece].position) < 2 * SAFETY_RADIUS:
                return piece
        return None

    def set_motion_planner(self, planner: MotionPlanner):
        """Swap motion planner strategy."""
        self.motion_planner = planner
        print(f"[BOARD] Motion planner changed to {planner.__class__.__name__}")

    def plan_all_moves(self):
        """Generate motion plan to return all pieces to starting positions."""
        if not self.simulation_mode:
            print("[BOARD] Cannot plan moves outside simulation mode.")
            return None

        targets = {piece: self.starting_positions[piece.index] for piece in self.pieces}
        self.current_plan = self.motion_planner.plan_moves(self.piece_states, targets)
        total_time = sum(plan.total_time for plan in self.current_plan.values())
        print(f"[BOARD] Generated motion plan. Total time: {total_time:.2f}s")
        return self.current_plan

    def execute_plan(self):
        """Execute the current motion plan in simulation mode."""
        if not self.simulation_mode:
            print("[BOARD] Cannot execute plan outside simulation mode.")
            return

        if not self.current_plan:
            print("[BOARD] No motion plan to execute.")
            return

        print("[BOARD] Executing motion plan...")
        # Run execution in a separate thread to avoid freezing UI
        execution_thread = threading.Thread(target=self._execute_plan_thread, daemon=True)
        execution_thread.start()

    def _execute_plan_thread(self):
        """Execute the motion plan in a background thread."""
        for piece, plan in self.current_plan.items():
            state = self.piece_states[piece]
            for motion in plan.motions:
                if motion.kind == "rotate":
                    self._execute_rotate(state, motion)
                elif motion.kind == "line":
                    self._execute_line(state, motion)
                elif motion.kind == "arc":
                    self._execute_arc(state, motion)
            state.at_target = True

        print("[BOARD] Motion plan execution complete.")
        if self.ui:
            self.ui.refresh_pieces()

    def _execute_rotate(self, state: PieceState, motion: Motion):
        """Execute a rotation motion."""
        diff = motion.params[0]
        steps = max(1, int(motion.duration / UPDATE_INTERVAL))
        for _ in range(steps):
            state.angle = state.angle + diff / steps
            if self.ui:
                self.ui.refresh_pieces_threadsafe()
            time.sleep(UPDATE_INTERVAL)

    def _execute_line(self, state: PieceState, motion: Motion):
        """Execute a linear motion."""
        distance = motion.params[0]
        dx, dy = math.cos(state.angle), math.sin(state.angle)
        steps = max(1, int(motion.duration / UPDATE_INTERVAL))
        step_dist = distance / steps
        for _ in range(steps):
            x, y = state.position
            new_pos = (x + dx * step_dist, y + dy * step_dist)
            if self.is_collision(new_pos, state.piece):
                print(f"[BOARD] {state.piece.name} blocked during execution")
                return
            state.position = new_pos
            if self.ui:
                self.ui.refresh_pieces_threadsafe()
            time.sleep(UPDATE_INTERVAL)

    def _execute_arc(self, state: PieceState, motion: Motion):
        """Execute an arc motion."""
        r, arc_angle = motion.params
        cx = state.position[0] - math.sin(state.angle) * r
        cy = state.position[1] + math.cos(state.angle) * r
        steps = max(1, int(motion.duration / UPDATE_INTERVAL))
        for _ in range(steps):
            state.angle = state.angle + arc_angle / steps
            x = cx + math.sin(state.angle) * r
            y = cy - math.cos(state.angle) * r
            if self.is_collision((x, y), state.piece):
                print(f"[BOARD] {state.piece.name} blocked during arc")
                return
            state.position = (x, y)
            if self.ui:
                self.ui.refresh_pieces_threadsafe()
            time.sleep(UPDATE_INTERVAL)


# --- UI ---
class ChessBoardUI:
    """Main UI for the robotic chessboard coordinator."""

    def __init__(self, root, board):
        self.root = root
        self.board = board
        board.ui = self
        self.canvas_size = 600
        
        # Main canvas
        self.canvas = tk.Canvas(root, width=self.canvas_size, height=self.canvas_size, bg="white")
        self.canvas.pack()

        # Debug panel frame
        debug_frame = tk.Frame(root, relief=tk.SUNKEN, borderwidth=2)
        debug_frame.pack(fill=tk.X, padx=5, pady=5)

        # Simulation mode toggle
        mode_frame = tk.Frame(debug_frame)
        mode_frame.pack(anchor=tk.W, padx=5, pady=5)
        tk.Label(mode_frame, text="Mode:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.sim_var = tk.BooleanVar(value=False)
        self.sim_switch = tk.Checkbutton(mode_frame, text="Simulation Mode", variable=self.sim_var, command=self.toggle_simulation_mode)
        self.sim_switch.pack(side=tk.LEFT, padx=10)
        self.mode_label = tk.Label(mode_frame, text="[Real Mode]", fg="red", font=("Arial", 9))
        self.mode_label.pack(side=tk.LEFT, padx=10)

        # Control buttons
        button_frame = tk.Frame(debug_frame)
        button_frame.pack(anchor=tk.W, padx=5, pady=5)
        tk.Label(button_frame, text="Controls:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        tk.Button(button_frame, text="Randomize", command=self.randomize_pieces, state=tk.DISABLED).pack(side=tk.LEFT, padx=3)
        tk.Button(button_frame, text="Plan Moves", command=self.plan_moves, state=tk.DISABLED).pack(side=tk.LEFT, padx=3)
        tk.Button(button_frame, text="Execute", command=self.execute_moves, state=tk.DISABLED).pack(side=tk.LEFT, padx=3)
        
        self.randomize_btn = button_frame.winfo_children()[1]
        self.plan_btn = button_frame.winfo_children()[2]
        self.execute_btn = button_frame.winfo_children()[3]

        # Motion Planner selection
        planner_frame = tk.Frame(debug_frame)
        planner_frame.pack(anchor=tk.W, padx=5, pady=5)
        tk.Label(planner_frame, text="Planner:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.planner_var = tk.StringVar(value="intelligent")
        tk.Radiobutton(planner_frame, text="Intelligent", variable=self.planner_var, value="intelligent", 
                      command=self.switch_planner, state=tk.DISABLED).pack(side=tk.LEFT, padx=5)
        tk.Radiobutton(planner_frame, text="Basic", variable=self.planner_var, value="basic", 
                      command=self.switch_planner, state=tk.DISABLED).pack(side=tk.LEFT, padx=5)
        self.planner_buttons = planner_frame.winfo_children()[1:]

        self.current_plan = None
        self.draw_grid()
        self.refresh_pieces()

    def toggle_simulation_mode(self):
        """Toggle between simulation and real mode."""
        self.board.simulation_mode = self.sim_var.get()
        if self.board.simulation_mode:
            self.mode_label.config(text="[Simulation Mode]", fg="green")
            self.randomize_btn.config(state=tk.NORMAL)
            self.plan_btn.config(state=tk.NORMAL)
            self.execute_btn.config(state=tk.NORMAL)
            for btn in self.planner_buttons:
                btn.config(state=tk.NORMAL)
            print("[UI] Entered simulation mode")
        else:
            self.mode_label.config(text="[Real Mode]", fg="red")
            self.randomize_btn.config(state=tk.DISABLED)
            self.plan_btn.config(state=tk.DISABLED)
            self.execute_btn.config(state=tk.DISABLED)
            for btn in self.planner_buttons:
                btn.config(state=tk.DISABLED)
            # Reset all pieces to starting positions
            self.board.initialize_piece_states()
            self.refresh_pieces()
            print("[UI] Entered real mode")

    def randomize_pieces(self):
        """Randomize piece positions in simulation mode."""
        if self.board.simulation_mode:
            self.board.randomize_positions()

    def switch_planner(self):
        """Switch between motion planning strategies."""
        planner_type = self.planner_var.get()
        if planner_type == "intelligent":
            self.board.set_motion_planner(IntelligentMotionPlanner(self.board))
            print("[UI] Switched to Intelligent Motion Planner")
        else:
            self.board.set_motion_planner(BasicMotionPlanner(self.board))
            print("[UI] Switched to Basic Motion Planner")

    def plan_moves(self):
        """Generate motion plan in simulation mode."""
        if self.board.simulation_mode:
            self.current_plan = self.board.plan_all_moves()
            if self.current_plan:
                print(f"[UI] Generated plan for {len(self.current_plan)} pieces")
                self.show_planned_paths()

    def show_planned_paths(self):
        """Visualize planned paths as dotted lines."""
        if not self.current_plan:
            return
        
        self.canvas.delete("path")
        scale = self.canvas_size / (BOARD_SIZE + 2 * SIDE_ZONE)
        
        for piece, plan in self.current_plan.items():
            state = self.board.piece_states[piece]
            current_pos = state.position
            current_angle = state.angle
            
            # Draw path segments for each motion
            for motion in plan.motions:
                if motion.kind == "rotate":
                    # Update angle for next segment
                    current_angle = (current_angle + motion.params[0]) % (2 * math.pi)
                    continue
                elif motion.kind == "line":
                    # Draw straight line segment
                    distance = motion.params[0]
                    dx = math.cos(current_angle)
                    dy = math.sin(current_angle)
                    end_pos = (current_pos[0] + dx * distance, current_pos[1] + dy * distance)
                    
                    x1 = (current_pos[0] + SIDE_ZONE) * scale
                    y1 = (BOARD_SIZE - current_pos[1]) * scale
                    x2 = (end_pos[0] + SIDE_ZONE) * scale
                    y2 = (BOARD_SIZE - end_pos[1]) * scale
                    
                    self.canvas.create_line(x1, y1, x2, y2, fill="orange", dash=(3, 3), width=2, tags="path")
                    current_pos = end_pos
                    
                elif motion.kind == "arc":
                    # Draw arc segment (approximated with line segments)
                    r, arc_angle = motion.params
                    cx = current_pos[0] - math.sin(current_angle) * r
                    cy = current_pos[1] + math.cos(current_angle) * r
                    
                    arc_steps = max(5, int(abs(arc_angle) / 0.1))
                    for i in range(arc_steps):
                        t1 = i / arc_steps
                        t2 = (i + 1) / arc_steps
                        
                        angle1 = current_angle + arc_angle * t1
                        angle2 = current_angle + arc_angle * t2
                        
                        x1_world = cx + math.sin(angle1) * r
                        y1_world = cy - math.cos(angle1) * r
                        x2_world = cx + math.sin(angle2) * r
                        y2_world = cy - math.cos(angle2) * r
                        
                        x1 = (x1_world + SIDE_ZONE) * scale
                        y1 = (BOARD_SIZE - y1_world) * scale
                        x2 = (x2_world + SIDE_ZONE) * scale
                        y2 = (BOARD_SIZE - y2_world) * scale
                        
                        self.canvas.create_line(x1, y1, x2, y2, fill="orange", dash=(3, 3), width=2, tags="path")
                    
                    # Update current position and angle
                    current_angle = (current_angle + arc_angle) % (2 * math.pi)
                    current_pos = (cx + math.sin(current_angle) * r, cy - math.cos(current_angle) * r)
        
        # Refresh pieces on top of paths
        self.refresh_pieces()

    def execute_moves(self):
        """Execute motion plan in simulation mode."""
        if self.board.simulation_mode:
            if self.current_plan:
                # Disable buttons during execution
                self.plan_btn.config(state=tk.DISABLED)
                self.execute_btn.config(state=tk.DISABLED)
                self.randomize_btn.config(state=tk.DISABLED)
                
                # Execute plan
                self.board.execute_plan()
                
                # Re-enable buttons after a delay (plan execution time)
                # Schedule re-enable after 1 second to let execution start
                self.root.after(1000, self._re_enable_buttons)
                
                # Clear paths after execution
                self.canvas.delete("path")
            else:
                print("[UI] No plan to execute. Run 'Plan Moves' first.")

    def _re_enable_buttons(self):
        """Re-enable control buttons after execution completes."""
        if self.board.simulation_mode:
            self.plan_btn.config(state=tk.NORMAL)
            self.execute_btn.config(state=tk.NORMAL)
            self.randomize_btn.config(state=tk.NORMAL)

    def draw_grid(self):
        """Draw the chess board grid."""
        self.canvas.delete("grid")
        sq = self.canvas_size / (BOARD_SIZE + 2 * SIDE_ZONE)
        for i in range(9):
            y = i * (BOARD_SIZE / 8) * sq
            self.canvas.create_line(SIDE_ZONE * sq, y, (BOARD_SIZE + SIDE_ZONE) * sq, y, fill="gray", tags="grid")
            self.canvas.create_line(i * (BOARD_SIZE / 8) * sq + SIDE_ZONE * sq, 0, i * (BOARD_SIZE / 8) * sq + SIDE_ZONE * sq, BOARD_SIZE * sq, fill="gray", tags="grid")

    def refresh_pieces(self):
        """Redraw all pieces on the canvas."""
        self.canvas.delete("piece")
        scale = self.canvas_size / (BOARD_SIZE + 2 * SIDE_ZONE)
        for piece in self.board.pieces:
            state = self.board.piece_states[piece]
            x = (state.position[0] + SIDE_ZONE) * scale
            y = (BOARD_SIZE - state.position[1]) * scale
            r = PIECE_RADIUS * scale
            fill = "white" if piece.color == "white" else "black"
            outline = "green" if state.at_target else "blue"
            
            # Normalize angle for display (sin/cos work on any angle, but normalize for clarity)
            display_angle = state.angle % (2 * math.pi)
            
            # Draw piece circle
            self.canvas.create_oval(x - r, y - r, x + r, y + r, fill=fill, outline=outline, width=2, tags="piece")
            # Draw safety radius
            self.canvas.create_oval(x - r * 1.3, y - r * 1.3, x + r * 1.3, y + r * 1.3, outline="gray", dash=(2, 2), tags="piece")
            # Draw orientation indicator using normalized angle
            self.canvas.create_line(x, y, x + r * math.cos(display_angle), y - r * math.sin(display_angle), fill="red", width=2, tags="piece")
            # Draw piece label
            self.canvas.create_text(x, y, text=piece.name, fill="red" if piece.color == "white" else "white", tags="piece")

    def refresh_pieces_threadsafe(self):
        """Thread-safe refresh that schedules canvas updates on the main thread."""
        self.root.after(0, self.refresh_pieces)


# --- Run app ---
if __name__ == "__main__":
    root = tk.Tk()
    root.title("Robotic Chessboard Coordinator")
    board = ChessBoard()
    ui = ChessBoardUI(root, board)
    root.mainloop()
