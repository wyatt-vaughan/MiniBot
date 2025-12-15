"""
A* Path Planner - Uses A* algorithm with intelligent prioritization
"""

import math
import heapq
from typing import Dict, List, Optional, Tuple, Set
from .path_planner import PathPlanner
from .data_types import Piece, Position, ExecutionPlan, PieceCommandSequence, CommandType
from .constants import PIECE_START_POSITIONS, PIECE_INTERMEDIATE_POSITIONS, PIECE_RADIUS, BOARD_SQUARE_SIZE, BOARD_EXTRA_SIDE


class AStarPlanner(PathPlanner):
    """Path planner using A* algorithm with back-rank and distance prioritization"""
    
    def __init__(self, grid_resolution: int = 10):
        """
        Initialize A* planner
        
        Args:
            grid_resolution: Grid cell size in mm (smaller = more precise but slower)
        """
        self.grid_resolution = grid_resolution
        # Board includes 100mm margins on left/right, 20mm on top/bottom
        self.board_width = 8 * BOARD_SQUARE_SIZE + 200  # +100mm each side
        self.board_height = 8 * BOARD_SQUARE_SIZE + 40  # +20mm each side
        self.margin_left = 100  # mm
        self.margin_top = 20  # mm
    
    def plan_movements(self, pieces: Dict[str, Piece], 
                       target_positions: Dict[str, Position]) -> ExecutionPlan:
        """
        Plan movements using A* algorithm with iterative prioritization
        """
        print(f"\n=== A* Planner Starting ===")
        print(f"Total pieces to move: {len(target_positions)}")
        
        plan = ExecutionPlan()
        
        # Track simulated positions for collision checking
        simulated_positions = {pid: piece.position.copy() for pid, piece in pieces.items()}
        
        # Create intermediate targets for pawns (zigzag pattern to clear space)
        intermediate_targets = self._create_intermediate_pawn_targets(target_positions)
        
        remaining_pieces = set(intermediate_targets.keys())
        current_time = 0.0
        max_iterations = 100  # Prevent infinite loops
        
        for iteration in range(max_iterations):
            if not remaining_pieces:
                break
            
            print(f"\nIteration {iteration + 1}: {len(remaining_pieces)} pieces remaining")
            
            # Reassign targets for identical pieces based on current distances
            optimized_targets = self._optimize_target_assignments(
                remaining_pieces, simulated_positions, intermediate_targets
            )
            print(f"  Target assignments optimized")
            
            # Calculate priority for each remaining piece
            priorities = self._calculate_priorities(remaining_pieces, simulated_positions, optimized_targets)
            
            if not priorities:
                break
            
            print(f"  Prioritized {len(priorities)} pieces")
            moved_any = False
            
            # Process pieces in priority order
            for piece_id in priorities:
                if piece_id not in remaining_pieces:
                    continue
                
                current_pos = simulated_positions[piece_id]
                target_pos = optimized_targets[piece_id]
                
                # Check if already at target
                if current_pos.distance_to(target_pos) < 1.0:
                    remaining_pieces.discard(piece_id)
                    continue
                
                # Try to find A* path
                path = self._find_astar_path(piece_id, current_pos, target_pos, simulated_positions)
                
                if path and len(path) > 1:
                    print(f"  -> {piece_id}: Found A* path with {len(path)} waypoints, distance={current_pos.distance_to(target_pos):.1f}mm")
                    # Found a complete path - create command sequence
                    sequence = self._create_sequence_from_path(piece_id, current_pos, path)
                    
                    if sequence and len(sequence.commands) > 0:
                        plan.add_sequence_at_time(sequence, current_time)
                        current_time += sequence.total_duration
                        
                        # Update simulated position to end of path
                        self._update_position_from_path(simulated_positions[piece_id], path)
                        
                        # Check if reached target
                        if simulated_positions[piece_id].distance_to(target_pos) < 1.0:
                            remaining_pieces.discard(piece_id)
                        
                        moved_any = True
                        break  # Move to next iteration
                
                else:
                    # No complete path - try to move halfway to first collision
                    print(f"  -> {piece_id}: No complete path, attempting partial movement")
                    partial_sequence = self._move_toward_target_partial(
                        piece_id, current_pos, target_pos, simulated_positions
                    )
                    
                    if partial_sequence and len(partial_sequence.commands) > 0:
                        plan.add_sequence_at_time(partial_sequence, current_time)
                        current_time += partial_sequence.total_duration
                        
                        # Update simulated position
                        self._update_simulated_position(simulated_positions[piece_id], partial_sequence)
                        
                        moved_any = True
                        break  # Move to next iteration
            
            if not moved_any:
                # No piece could move this iteration
                print(f"  *** Iteration {iteration + 1}: STUCK - no pieces could move ***")
                break
        
        total_time = plan.get_total_duration()
        
        pieces_moved = len(target_positions) - len(remaining_pieces)
        print(f"\n=== A* Planner Complete ===")
        print(f"Total time: {total_time:.1f}s")
        print(f"Pieces moved: {pieces_moved}/{len(target_positions)}")
        print(f"Success rate: {pieces_moved/len(target_positions)*100:.1f}%")
        
        # Final parallel alignment: Move all pieces to their closest valid target
        print(f"\n=== Final Parallel Alignment ===")
        
        # Optimize final target assignments based on current positions
        final_optimized_targets = self._optimize_target_assignments(
            set(target_positions.keys()), simulated_positions, target_positions
        )
        
        pieces_to_align = []
        for piece_id in target_positions.keys():
            current_pos = simulated_positions[piece_id]
            target_pos = final_optimized_targets[piece_id]  # Use optimized closest target
            distance = current_pos.distance_to(target_pos)
            
            if distance >= 0.5:  # Only align if more than 0.5mm away
                pieces_to_align.append((piece_id, distance))
        
        print(f"Aligning {len(pieces_to_align)} pieces in parallel")
        
        if pieces_to_align:
            for piece_id, distance in pieces_to_align:
                current_pos = simulated_positions[piece_id]
                target_pos = final_optimized_targets[piece_id]  # Use optimized target
                
                # Create direct path to target (convert Position objects to tuples)
                sequence = self._create_sequence_from_path(
                    piece_id, current_pos, [(current_pos.x, current_pos.y), (target_pos.x, target_pos.y)]
                )
                
                if sequence:
                    print(f"  -> {piece_id}: Final alignment {distance:.1f}mm")
                    plan.add_sequence_at_time(sequence, total_time)
                    # Update simulated position
                    simulated_positions[piece_id].x = target_pos.x
                    simulated_positions[piece_id].y = target_pos.y
            
            # Update total time
            total_time = plan.get_total_duration()
            print(f"Alignment complete, total time now: {total_time:.1f}s")
        
        # Final orientation correction: Rotate all pieces to face forward
        print(f"\n=== Final Orientation Alignment ===")
        pieces_to_rotate = []
        for piece_id in target_positions.keys():
            # White pieces (P1-P8, R1, N1, B1, Q, K, B2, N2, R2) face up (90 degrees)
            # Black pieces (p1-p8, r1, n1, b1, q, k, b2, n2, r2) face down (270 degrees)
            target_orientation = 90.0 if piece_id[0].isupper() else 270.0
            current_orientation = simulated_positions[piece_id].orientation
            
            angle_diff = abs((target_orientation - current_orientation + 180) % 360 - 180)
            if angle_diff > 1.0:  # Only rotate if more than 1 degree off
                pieces_to_rotate.append((piece_id, angle_diff))
        
        print(f"Rotating {len(pieces_to_rotate)} pieces to correct orientation")
        
        if pieces_to_rotate:
            for piece_id, angle_diff in pieces_to_rotate:
                target_orientation = 90.0 if piece_id[0].isupper() else 270.0
                current_pos = simulated_positions[piece_id]
                
                # Create rotation command
                sequence = PieceCommandSequence(piece_id=piece_id)
                rotate_cmd = self._create_rotate_command(current_pos.orientation, target_orientation)
                sequence.add_command(rotate_cmd)
                
                print(f"  -> {piece_id}: Rotating {angle_diff:.1f}° to {target_orientation}°")
                plan.add_sequence_at_time(sequence, total_time)
                simulated_positions[piece_id].orientation = target_orientation
            
            # Update total time
            total_time = plan.get_total_duration()
            print(f"Orientation alignment complete, total time now: {total_time:.1f}s")
        
        return plan
    
    def _calculate_priorities(self, remaining_pieces: Set[str], 
                              simulated_positions: Dict[str, Position],
                              target_positions: Dict[str, Position]) -> List[str]:
        """Calculate and return pieces sorted by priority (highest first)"""
        priorities = []
        
        for piece_id in remaining_pieces:
            current_pos = simulated_positions[piece_id]
            target_pos = target_positions[piece_id]
            
            distance = current_pos.distance_to(target_pos)
            
            # Skip if already at target
            if distance < 1.0:
                continue
            
            # Check if back rank piece (row 0 or 7)
            is_back_rank = False
            for pid, (col, row) in PIECE_START_POSITIONS.items():
                if pid == piece_id and row in [0, 7]:
                    is_back_rank = True
                    break
            
            # Priority calculation: back rank gets high priority, then by distance (closer = higher priority)
            # Negate distance so closer pieces have higher priority
            priority_score = (1000.0 if is_back_rank else 0.0) - distance
            
            priorities.append((priority_score, piece_id))
        
        # Sort by priority score (highest first)
        priorities.sort(reverse=True)
        
        return [piece_id for _, piece_id in priorities]
    
    def _create_intermediate_pawn_targets(self, target_positions: Dict[str, Position]) -> Dict[str, Position]:
        """Create intermediate targets for pawns in zigzag pattern to clear space for back rank"""
        from .utils import board_coords_to_world
        intermediate = {}
        
        for piece_id, target_pos in target_positions.items():
            if piece_id in PIECE_INTERMEDIATE_POSITIONS:
                # Get intermediate position from constants
                col, row = PIECE_INTERMEDIATE_POSITIONS[piece_id]
                x, y = board_coords_to_world(col, row)
                intermediate[piece_id] = Position(x, y, target_pos.orientation)
            else:
                # Piece not in intermediate positions, use original target
                intermediate[piece_id] = target_pos.copy()
        
        return intermediate
    
    def _optimize_target_assignments(self, remaining_pieces: Set[str],
                                     simulated_positions: Dict[str, Position],
                                     target_positions: Dict[str, Position]) -> Dict[str, Position]:
        """Optimize target assignments for identical pieces based on distance"""
        import itertools
        
        optimized = target_positions.copy()
        
        # Group pieces by type
        piece_groups = {
            'white_pawns': [],
            'black_pawns': [],
            'white_rooks': [],
            'black_rooks': [],
            'white_knights': [],
            'black_knights': [],
            'white_bishops': [],
            'black_bishops': [],
        }
        
        target_groups = {
            'white_pawns': [],
            'black_pawns': [],
            'white_rooks': [],
            'black_rooks': [],
            'white_knights': [],
            'black_knights': [],
            'white_bishops': [],
            'black_bishops': [],
        }
        
        # Classify pieces that still need to move
        for piece_id in remaining_pieces:
            if piece_id not in target_positions:
                continue
            
            # White pawns: p1-p8
            if piece_id.startswith('p') and piece_id[1:].isdigit():
                piece_groups['white_pawns'].append(piece_id)
                target_groups['white_pawns'].append(target_positions[piece_id])
            # Black pawns: P1-P8
            elif piece_id.startswith('P') and piece_id[1:].isdigit():
                piece_groups['black_pawns'].append(piece_id)
                target_groups['black_pawns'].append(target_positions[piece_id])
            # White rooks: r1, r2
            elif piece_id in ['r1', 'r2']:
                piece_groups['white_rooks'].append(piece_id)
                target_groups['white_rooks'].append(target_positions[piece_id])
            # Black rooks: R1, R2
            elif piece_id in ['R1', 'R2']:
                piece_groups['black_rooks'].append(piece_id)
                target_groups['black_rooks'].append(target_positions[piece_id])
            # White knights: n1, n2
            elif piece_id in ['n1', 'n2']:
                piece_groups['white_knights'].append(piece_id)
                target_groups['white_knights'].append(target_positions[piece_id])
            # Black knights: N1, N2
            elif piece_id in ['N1', 'N2']:
                piece_groups['black_knights'].append(piece_id)
                target_groups['black_knights'].append(target_positions[piece_id])
            # White bishops: b1, b2
            elif piece_id in ['b1', 'b2']:
                piece_groups['white_bishops'].append(piece_id)
                target_groups['white_bishops'].append(target_positions[piece_id])
            # Black bishops: B1, B2
            elif piece_id in ['B1', 'B2']:
                piece_groups['black_bishops'].append(piece_id)
                target_groups['black_bishops'].append(target_positions[piece_id])
        
        # Optimize each group
        for group_name in piece_groups.keys():
            piece_ids = piece_groups[group_name]
            targets = target_groups[group_name]
            
            if len(piece_ids) <= 1:
                continue
            
            # Find optimal assignment to minimize total distance
            best_assignment = self._find_optimal_assignment(piece_ids, targets, simulated_positions)
            
            for piece_id, target_pos in best_assignment.items():
                optimized[piece_id] = target_pos
        
        return optimized
    
    def _find_optimal_assignment(self, piece_ids: List[str], targets: List[Position],
                                simulated_positions: Dict[str, Position]) -> Dict[str, Position]:
        """Find optimal target assignment to minimize total distance"""
        import itertools
        
        if len(piece_ids) != len(targets):
            return {pid: targets[i] for i, pid in enumerate(piece_ids)}
        
        best_assignment = {}
        best_total_distance = float('inf')
        
        # Try all permutations to find minimum total distance
        for target_permutation in itertools.permutations(targets):
            total_distance = 0
            for i, piece_id in enumerate(piece_ids):
                distance = simulated_positions[piece_id].distance_to(target_permutation[i])
                total_distance += distance
            
            if total_distance < best_total_distance:
                best_total_distance = total_distance
                best_assignment = {piece_ids[i]: target_permutation[i] for i in range(len(piece_ids))}
        
        return best_assignment
    
    def _find_astar_path(self, piece_id: str, start_pos: Position, 
                        goal_pos: Position, 
                        simulated_positions: Dict[str, Position]) -> Optional[List[Tuple[float, float]]]:
        """
        Find path using A* algorithm
        
        Returns:
            List of (x, y) waypoints from start to goal, or None if no path found
        """
        # Convert positions to grid coordinates
        start_grid = self._world_to_grid(start_pos.x, start_pos.y)
        goal_grid = self._world_to_grid(goal_pos.x, goal_pos.y)
        
        # A* data structures
        open_set = []  # Priority queue: (f_score, counter, grid_pos)
        counter = 0  # Tie-breaker for heap
        heapq.heappush(open_set, (0, counter, start_grid))
        
        came_from = {}  # Reconstruction path
        g_score = {start_grid: 0}  # Cost from start to node
        f_score = {start_grid: self._heuristic(start_grid, goal_grid)}  # Estimated total cost
        
        max_nodes = 5000  # Limit search to prevent slowdown
        nodes_explored = 0
        
        while open_set and nodes_explored < max_nodes:
            _, _, current = heapq.heappop(open_set)
            nodes_explored += 1
            
            # Check if reached goal
            if current == goal_grid:
                # Reconstruct path
                path = self._reconstruct_path(came_from, current)
                # Convert grid coordinates back to world coordinates
                world_path = [self._grid_to_world(gx, gy) for gx, gy in path]
                return world_path
            
            # Explore neighbors
            for neighbor in self._get_neighbors(current):
                # Check if neighbor is valid (on board and not colliding)
                world_x, world_y = self._grid_to_world(neighbor[0], neighbor[1])
                
                if not self._is_grid_position_valid(world_x, world_y, piece_id, simulated_positions):
                    continue
                
                # Calculate tentative g_score
                tentative_g = g_score[current] + self._grid_distance(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # This path to neighbor is better
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal_grid)
                    
                    # Add to open set if not already there
                    counter += 1
                    heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
        
        # No path found
        return None
    
    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x - BOARD_EXTRA_SIDE) / self.grid_resolution)
        grid_y = int(y / self.grid_resolution)
        return (grid_x, grid_y)
    
    def _grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates (center of grid cell)"""
        world_x = BOARD_EXTRA_SIDE + grid_x * self.grid_resolution + self.grid_resolution / 2
        world_y = grid_y * self.grid_resolution + self.grid_resolution / 2
        return (world_x, world_y)
    
    def _heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Euclidean distance heuristic"""
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        return math.sqrt(dx * dx + dy * dy)
    
    def _grid_distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Distance between adjacent grid cells"""
        dx = abs(pos2[0] - pos1[0])
        dy = abs(pos2[1] - pos1[1])
        # Diagonal movement costs more
        if dx == 1 and dy == 1:
            return 1.414  # sqrt(2)
        return 1.0
    
    def _get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get 8-connected neighbors of a grid position"""
        x, y = pos
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbors.append((x + dx, y + dy))
        return neighbors
    
    def _reconstruct_path(self, came_from: Dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstruct path from came_from map"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def _is_grid_position_valid(self, x: float, y: float, piece_id: str,
                                simulated_positions: Dict[str, Position]) -> bool:
        """Check if a world position is valid (on board and collision-free)"""
        # Check board bounds including margins (100mm left/right, 20mm top/bottom)
        if x < BOARD_EXTRA_SIDE - self.margin_left or x > BOARD_EXTRA_SIDE + 8 * BOARD_SQUARE_SIZE + self.margin_left:
            return False
        if y < -self.margin_top or y > 8 * BOARD_SQUARE_SIZE + self.margin_top:
            return False
        
        # Check collisions with other pieces
        for other_id, other_pos in simulated_positions.items():
            if other_id == piece_id:
                continue
            
            dx = x - other_pos.x
            dy = y - other_pos.y
            dist = math.sqrt(dx * dx + dy * dy)
            
            if dist < PIECE_RADIUS * 2 + 5:  # 5mm safety margin
                return False
        
        return True
    
    def _create_sequence_from_path(self, piece_id: str, start_pos: Position, 
                                   path: List[Tuple[float, float]]) -> Optional[PieceCommandSequence]:
        """Create a command sequence from a path of waypoints"""
        if len(path) < 2:
            return None
        
        sequence = PieceCommandSequence(piece_id=piece_id)
        current_orientation = start_pos.orientation
        current_x, current_y = start_pos.x, start_pos.y
        
        # Validate that start position matches first path point
        if len(path) > 0:
            path_start_x, path_start_y = path[0]
            dist_error = math.sqrt((current_x - path_start_x)**2 + (current_y - path_start_y)**2)
            if dist_error > 1.0:
                print(f"WARNING: {piece_id} start position mismatch: {dist_error:.1f}mm")
        
        # Create commands for each segment
        for i in range(1, len(path)):
            target_x, target_y = path[i]
            
            # Calculate angle to next waypoint
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.sqrt(dx * dx + dy * dy)
            
            if distance < 0.1:
                continue
            
            target_angle = math.degrees(math.atan2(dy, dx)) % 360
            
            # Add rotation command if needed
            angle_diff = abs((target_angle - current_orientation + 180) % 360 - 180)
            if angle_diff > 1.0:  # Only rotate if significant difference
                rotate_cmd = self._create_rotate_command(current_orientation, target_angle)
                sequence.add_command(rotate_cmd)
                current_orientation = target_angle
            
            # Add movement command
            move_cmd = self._create_move_command(distance)
            sequence.add_command(move_cmd)
            
            current_x, current_y = target_x, target_y
        
        return sequence if len(sequence.commands) > 0 else None
    
    def _update_position_from_path(self, pos: Position, path: List[Tuple[float, float]]):
        """Update position to end of path"""
        if len(path) > 0:
            pos.x, pos.y = path[-1]
            
            # Update orientation to face the last direction traveled
            if len(path) > 1:
                dx = path[-1][0] - path[-2][0]
                dy = path[-1][1] - path[-2][1]
                if abs(dx) > 0.1 or abs(dy) > 0.1:
                    pos.orientation = math.degrees(math.atan2(dy, dx)) % 360
    
    def _move_toward_target_partial(self, piece_id: str, current_pos: Position,
                                    target_pos: Position,
                                    simulated_positions: Dict[str, Position]) -> Optional[PieceCommandSequence]:
        """Move halfway toward target until first collision"""
        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y
        distance = math.sqrt(dx * dx + dy * dy)
        
        if distance < 1.0:
            return None
        
        target_angle = math.degrees(math.atan2(dy, dx)) % 360
        
        # Find maximum safe distance (up to halfway)
        max_attempt_distance = distance * 0.5
        safe_distance = 0.0
        
        # Check distances along the path
        for test_dist in [d for d in range(int(max_attempt_distance), 0, -5)]:
            test_x = current_pos.x + test_dist * math.cos(math.radians(target_angle))
            test_y = current_pos.y + test_dist * math.sin(math.radians(target_angle))
            
            if self._is_grid_position_valid(test_x, test_y, piece_id, simulated_positions):
                safe_distance = test_dist
                break
        
        if safe_distance > 1.0:
            sequence = PieceCommandSequence(piece_id=piece_id)
            
            # Rotate to face target
            rotate_cmd = self._create_rotate_command(current_pos.orientation, target_angle)
            sequence.add_command(rotate_cmd)
            
            # Move safe distance
            move_cmd = self._create_move_command(safe_distance)
            sequence.add_command(move_cmd)
            
            return sequence
        
        return None
    
    def _update_simulated_position(self, pos: Position, sequence: PieceCommandSequence):
        """Update simulated position based on command sequence"""
        for command in sequence.commands:
            if command.command_type == CommandType.ROTATE:
                pos.orientation = command.target_orientation
            elif command.command_type == CommandType.MOVE_STRAIGHT:
                angle_rad = math.radians(pos.orientation)
                pos.x += command.distance * math.cos(angle_rad)
                pos.y += command.distance * math.sin(angle_rad)
    
    def get_name(self) -> str:
        return "A* Planner"
