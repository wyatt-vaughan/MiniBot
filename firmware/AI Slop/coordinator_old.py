import math
import random
import time
import tkinter as tk
from abc import ABC, abstractmethod
from typing import List, Dict, Tuple, Optional

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
UPDATE_INTERVAL = 0.02

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

class ChessBoard:
    def __init__(self):
        self.pieces = []
        self.create_pieces()
        self.starting_positions = self.get_starting_positions()
        self.ui = None

    def create_pieces(self):
        Piece._next_index = 0
        self.pieces = []
        back_row = ['R', 'N', 'B', 'Q', 'K', 'B', 'N', 'R']
        for name in back_row: self.pieces.append(Piece(name, 'white'))
        for _ in range(8): self.pieces.append(Piece('P', 'white'))
        for name in back_row: self.pieces.append(Piece(name, 'black'))
        for _ in range(8): self.pieces.append(Piece('P', 'black'))

    def get_starting_positions(self):
        sq = BOARD_SIZE / 8
        pos = []
        pos += [(i*sq + sq/2, 0.5*sq) for i in range(8)]
        pos += [(i*sq + sq/2, 1.5*sq) for i in range(8)]
        pos += [(i*sq + sq/2, 7.5*sq) for i in range(8)]
        pos += [(i*sq + sq/2, 6.5*sq) for i in range(8)]
        return pos

    def randomize_positions(self):
        placed = []
        for piece in self.pieces:
            while True:
                x = random.uniform(TOTAL_X_MIN + SAFETY_RADIUS, TOTAL_X_MAX - SAFETY_RADIUS)
                y = random.uniform(TOTAL_Y_MIN + SAFETY_RADIUS, TOTAL_Y_MAX - SAFETY_RADIUS)
                if all(math.hypot(x - px, y - py) > 2 * SAFETY_RADIUS for px, py in placed):
                    piece.position = (x, y)
                    placed.append((x, y))
                    break
        if self.ui: self.ui.refresh_pieces()

    def distance(self, a, b): return math.hypot(a[0]-b[0], a[1]-b[1])

    def is_collision(self, pos, exclude=None):
        for piece in self.pieces:
            if piece == exclude: continue
            if self.distance(pos, piece.position) < 2*SAFETY_RADIUS:
                return piece
        return None

    # --- Simulation-based path planner ---
    def plan_all_moves(self):
        plans = {}
        for piece in self.pieces:
            piece.goal = self.starting_positions[piece.index]
            plans[piece] = self.simulate_piece_path(piece)
        return plans

    def simulate_piece_path(self, piece):
        path = []
        current_pos, current_angle = piece.position, piece.angle
        total_time = 0.0
        max_attempts = 5

        for attempt in range(max_attempts):
            dx, dy = piece.goal[0] - current_pos[0], piece.goal[1] - current_pos[1]
            dist = math.hypot(dx, dy)
            if dist < 1e-3: break

            target_angle = math.atan2(dy, dx)
            diff = (target_angle - current_angle + math.pi) % (2*math.pi) - math.pi
            reverse = abs(diff) > math.pi/2
            if reverse: target_angle = (target_angle + math.pi) % (2*math.pi)

            rotate_time = abs(diff) / ROTATE_SPEED
            path.append(PlannedMove("rotate", (diff,), rotate_time))
            total_time += rotate_time
            current_angle = target_angle

            if self.try_path_clear_sim(current_pos, current_angle, dist, piece, reverse):
                line_time = dist / MOVE_SPEED
                path.append(PlannedMove("line", (dist,), line_time))
                total_time += line_time
                current_pos = piece.goal
                break
            else:
                detour_success, detour_time, new_pos, new_angle = self.simulate_detour(piece, current_pos, current_angle)
                if detour_success:
                    path.append(PlannedMove("arc", (SAFETY_RADIUS*3, math.pi/3), detour_time))
                    total_time += detour_time
                    current_pos, current_angle = new_pos, new_angle
                else:
                    blockers = self.detect_nearby_blockers_sim(current_pos, piece)
                    if not any(self.simulate_move_blocker(b, piece) for b in blockers):
                        break
        return {"moves": path, "total_time": total_time}

    def try_path_clear_sim(self, pos, angle, distance, piece, reverse=False):
        dx, dy = math.cos(angle), math.sin(angle)
        if reverse: distance *= -1
        steps = max(1, int(abs(distance) / SAFETY_RADIUS))
        for i in range(steps+1):
            check_pos = (pos[0] + dx * distance * i / steps,
                         pos[1] + dy * distance * i / steps)
            if self.is_collision(check_pos, piece): return False
        return True

    def simulate_detour(self, piece, pos, angle):
        for side in [+1, -1]:
            r, arc_angle, steps = SAFETY_RADIUS*3, side*math.pi/3, 10
            valid = True
            for i in range(steps+1):
                ang = angle + arc_angle*i/steps
                x = pos[0] - math.sin(angle)*r + math.sin(ang)*r
                y = pos[1] + math.cos(angle)*r - math.cos(ang)*r
                if self.is_collision((x, y), piece):
                    valid = False; break
            if valid:
                arc_len = r * abs(arc_angle)
                arc_time = arc_len / MOVE_SPEED
                new_pos = (pos[0] - math.sin(angle)*r + math.sin(angle+arc_angle)*r,
                           pos[1] + math.cos(angle)*r - math.cos(angle+arc_angle)*r)
                new_ang = angle + arc_angle
                return True, arc_time, new_pos, new_ang
        return False, 0.0, pos, angle

    def detect_nearby_blockers_sim(self, pos, piece):
        return [p for p in self.pieces if p != piece and self.distance(pos, p.position) < 2*SAFETY_RADIUS]

    def simulate_move_blocker(self, blocker, mover):
        angle = math.atan2(blocker.position[1]-mover.position[1],
                           blocker.position[0]-mover.position[0])
        for side in [+1, -1]:
            offset = angle + side*math.pi/2
            new_pos = (blocker.position[0] + math.cos(offset)*SAFETY_RADIUS*2,
                       blocker.position[1] + math.sin(offset)*SAFETY_RADIUS*2)
            if not self.is_collision(new_pos, blocker):
                move_t = SAFETY_RADIUS*2 / MOVE_SPEED
                rotate_diff = (offset - blocker.angle + math.pi) % (2*math.pi) - math.pi
                rot_t = abs(rotate_diff) / ROTATE_SPEED
                return True
        return False

    def paths_conflict(self, a, b):
        """Return True if two pieces’ paths might collide."""
        # Ensure both are Piece instances
        if not isinstance(a, Piece) or not isinstance(b, Piece):
            print(f"[WARNING] paths_conflict received non-Piece objects: {a}, {b}")
            return False  # or True if you want to be conservative
        return self.distance(a.position, b.position) < 3 * SAFETY_RADIUS

    def optimize_paths(self, plans):
        """Group pieces for parallel movement based on non-conflicting paths."""
        groups = []
        for piece in self.pieces:  # explicitly iterate over Piece objects
            if piece not in plans:
                continue
            placed = False
            for g in groups:
                # Only check conflicts with actual Piece objects
                if all(isinstance(other, Piece) for other in g):
                    if not any(self.paths_conflict(piece, other) for other in g):
                        g.append(piece)
                        placed = True
                        break
                else:
                    # clean invalid entries
                    g[:] = [p for p in g if isinstance(p, Piece)]
            if not placed:
                groups.append([piece])

        total_time = 0.0
        for g in groups:
            group_time = max(plans[p]["total_time"] for p in g if isinstance(p, Piece))
            total_time += group_time

        print(f"[OPTIMIZER] {len(groups)} parallel groups, est total time {total_time:.2f}s")
        return {"groups": groups, "plans": plans, "total_time": total_time}


    def execute_plan(self, plan_bundle, ui=None):
        if not plan_bundle or "plans" not in plan_bundle:
            print("[EXEC] No valid plan to execute.")
            return
        self.ui = ui
        plans = plan_bundle["plans"]
        groups = plan_bundle["groups"]

        print("[EXEC] Executing optimized motion plan...")

        for group_idx, group in enumerate(groups):
            print(f"[EXEC] Group {group_idx+1}/{len(groups)} ({len(group)} pieces in parallel)")
            active = []
            for piece in group:
                active.append(self._execute_piece_plan(piece, plans[piece]))
            # Wait for all group pieces to finish before next
            for t in active:
                t.join() if hasattr(t, "join") else None
        print("[EXEC] All motion complete.")

    def _execute_piece_plan(self, piece, plan):
        # Sequentially execute the piece’s precomputed plan
        for move in plan["moves"]:
            if move.kind == "rotate":
                self.rotate_in_place(piece, piece.angle + move.params[0])
            elif move.kind == "line":
                self.move_straight(piece, move.params[0])
            elif move.kind == "arc":
                self.follow_arc(piece, *move.params)
        piece.at_target = True
        if self.ui: self.ui.refresh_pieces()
        return piece
    
    def move_straight(self, piece, distance):
        dx, dy = math.cos(piece.angle), math.sin(piece.angle)
        if getattr(piece, "reverse_mode", False):
            distance *= -1
        steps = max(1, int(abs(distance)/(MOVE_SPEED*UPDATE_INTERVAL)))
        step_dist = distance / steps
        for _ in range(steps):
            x, y = piece.position
            new_pos = (x + dx*step_dist, y + dy*step_dist)
            blocker = self.is_collision(new_pos, piece)
            if blocker:
                print(f"[BLOCKED] {piece.name} blocked by {blocker.name}")
                return False
            piece.position = new_pos
            if self.ui: self.ui.refresh_pieces()
            time.sleep(UPDATE_INTERVAL)
        return True

    def distance(self, a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])
    
    def follow_arc(self, piece, r, arc_angle):
        cx = piece.position[0] - math.sin(piece.angle) * r
        cy = piece.position[1] + math.cos(piece.angle) * r
        steps = max(1, int(abs(arc_angle) / 0.05))
        for _ in range(steps):
            piece.angle += arc_angle / steps
            x = cx + math.sin(piece.angle) * r
            y = cy - math.cos(piece.angle) * r
            if self.is_collision((x, y), piece):
                return False
            piece.position = (x, y)
            if self.ui: self.ui.refresh_pieces()
            time.sleep(UPDATE_INTERVAL)
        return True

    def rotate_in_place(self, piece, target_angle):
        diff = (target_angle - piece.angle + math.pi) % (2 * math.pi) - math.pi
        if abs(diff) > math.pi / 2:
            target_angle = (target_angle + math.pi) % (2 * math.pi)
            diff = (target_angle - piece.angle + math.pi) % (2 * math.pi) - math.pi
            piece.reverse_mode = True
        else:
            piece.reverse_mode = False

        while abs(diff) >= ROTATE_MAX_ERROR:
            step = ROTATE_SPEED * UPDATE_INTERVAL * (1 if diff > 0 else -1)
            piece.angle = (piece.angle + step) % (2 * math.pi)
            diff = (target_angle - piece.angle + math.pi) % (2 * math.pi) - math.pi
            if self.ui:
                self.ui.refresh_pieces()
            time.sleep(UPDATE_INTERVAL)



# --- UI ---
class ChessBoardUI:
    def __init__(self, root, board):
        self.root = root
        self.board = board
        board.ui = self
        self.canvas_size = 600
        self.canvas = tk.Canvas(root, width=self.canvas_size, height=self.canvas_size, bg="white")
        self.canvas.pack()

        tk.Button(root, text="Randomize Pieces", command=self.board.randomize_positions).pack()
        tk.Button(root, text="Precompute Moves", command=self.precompute_moves).pack()
        tk.Button(root, text="Optimize Moves", command=self.optimize_moves).pack()
        tk.Button(root, text="Start Motion", command=self.start_motion).pack()


        self.plans = None
        self.optimized_plans = None
        self.draw_grid()

    def draw_grid(self):
        self.canvas.delete("grid")
        sq = self.canvas_size / (BOARD_SIZE + 2*SIDE_ZONE)
        for i in range(9):
            y = i * (BOARD_SIZE/8) * sq
            self.canvas.create_line(SIDE_ZONE*sq, y, (BOARD_SIZE+SIDE_ZONE)*sq, y, fill="gray", tags="grid")
            self.canvas.create_line(i*(BOARD_SIZE/8)*sq+SIDE_ZONE*sq, 0, i*(BOARD_SIZE/8)*sq+SIDE_ZONE*sq, BOARD_SIZE*sq, fill="gray", tags="grid")

    def refresh_pieces(self):
        self.canvas.delete("piece")
        scale = self.canvas_size / (BOARD_SIZE + 2 * SIDE_ZONE)
        for piece in self.board.pieces:
            x = (piece.position[0] + SIDE_ZONE) * scale
            y = (BOARD_SIZE - piece.position[1]) * scale
            r = PIECE_RADIUS * scale
            fill = "white" if piece.color == "white" else "black"
            outline = "green" if piece.at_target else "blue"
            self.canvas.create_oval(x - r, y - r, x + r, y + r, fill=fill, outline=outline, width=2, tags="piece")
            self.canvas.create_oval(x - r*1.3, y - r*1.3, x + r*1.3, y + r*1.3, outline="gray", dash=(2,2), tags="piece")
            self.canvas.create_line(x, y, x + r*math.cos(piece.angle), y - r*math.sin(piece.angle), fill="red", width=2, tags="piece")
            self.canvas.create_text(x, y, text=piece.name, fill="red" if piece.color == "white" else "white", tags="piece")

    def precompute_moves(self):
        print("[UI] Precomputing paths...")
        self.plans = self.board.plan_all_moves()
        self.show_paths(self.plans)

    def optimize_moves(self):
        if not self.plans:
            print("[UI] No plans to optimize.")
            return
        self.optimized_plans = self.board.optimize_paths(self.plans)
        self.show_paths(self.optimized_plans.get("plans"))

    def show_paths(self, plans):
        self.canvas.delete("path")
        scale = self.canvas_size / (BOARD_SIZE + 2 * SIDE_ZONE)
        for piece, plan in plans.items():
            start_x = (piece.position[0] + SIDE_ZONE) * scale
            start_y = (BOARD_SIZE - piece.position[1]) * scale
            end_x = (piece.goal[0] + SIDE_ZONE) * scale
            end_y = (BOARD_SIZE - piece.goal[1]) * scale
            self.canvas.create_line(start_x, start_y, end_x, end_y, fill="orange", dash=(3, 3), width=2, tags="path")
        self.refresh_pieces()
        self.show_timeline(plans)

    def show_timeline(self, plans):
        self.canvas.delete("timeline")
        total_time = sum(p["total_time"] for p in plans.values())
        bar_height, start_y = 10, self.canvas_size - 150
        for i, (piece, plan) in enumerate(plans.items()):
            x0, y0 = 50, start_y + i * (bar_height + 5)
            x1 = 50 + (plan["total_time"] / total_time) * 500
            color = "lightblue" if piece.color == "white" else "lightgray"
            self.canvas.create_rectangle(x0, y0, x1, y0 + bar_height, fill=color, outline="black", tags="timeline")
            self.canvas.create_text(x0 - 20, y0 + 5, text=piece.name, anchor="e", tags="timeline")

    def start_motion(self):
        if not self.optimized_plans:
            print("[UI] No optimized plan — run Optimize Moves first.")
            return
        print("[UI] Starting motion...")
        self.board.execute_plan(self.optimized_plans, ui=self)
        print("[UI] Motion complete.")


# --- Run app ---
if __name__ == "__main__":
    root = tk.Tk()
    root.title("Robotic Chessboard Coordinator")
    board = ChessBoard()
    ui = ChessBoardUI(root, board)
    ui.refresh_pieces()
    root.mainloop()
