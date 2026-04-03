"""
AI Path Planner - Advanced collision-aware planning with intelligent routing
"""

import math
import random
import numpy as np
import itertools
from typing import Dict, List, Optional
from .path_planner import PathPlanner
from .data_types import Piece, Position, ExecutionPlan, PieceCommandSequence, CommandType
from .constants import PIECE_START_POSITIONS, PIECE_RADIUS, BOARD_SQUARE_SIZE, BOARD_EXTRA_SIDE


class AI_Planner(PathPlanner):
    """Iterative planner that tries multiple random orderings to find optimal collision-free paths"""
    
    def plan_movements(self, pieces: Dict[str, Piece], 
                       target_positions: Dict[str, Position]) -> ExecutionPlan:
        """
        Plan movements using iterative optimization with random priority.
        Pieces move one at a time, avoiding collisions with all other pieces.
        """
        best_plan = None
        best_score = float('inf')
        best_time = float('inf')
        
        # Try 100 different random orderings
        for attempt in range(20):
            plan = ExecutionPlan()
            
            # Track simulated positions for collision checking
            simulated_positions = {pid: piece.position.copy() for pid, piece in pieces.items()}
            
            # Optimize target assignments for identical pieces (with random strategy)
            optimize_chance = random.random()
            if optimize_chance < 0.7:  # 70% use optimization
                optimized_targets = self._optimize_target_assignments(pieces, target_positions, simulated_positions)
            elif optimize_chance < 0.85:  # 15% use partial random swaps
                optimized_targets = self._random_swap_targets(pieces, target_positions, simulated_positions)
            else:  # 15% use original assignments
                optimized_targets = target_positions.copy()
            
            # Calculate distances to target for prioritization
            distances = {}
            for piece_id in pieces.keys():
                if piece_id in optimized_targets:
                    dist = simulated_positions[piece_id].distance_to(optimized_targets[piece_id])
                    distances[piece_id] = dist
            
            remaining_pieces = set(distances.keys())
            current_time = 0.0
            max_iterations = 50  # Prevent infinite loops
            
            for iteration in range(max_iterations):
                if not remaining_pieces:
                    break
                
                # Calculate priority: distance + random + back rank bonus
                priorities = []
                for piece_id in remaining_pieces:
                    dist = simulated_positions[piece_id].distance_to(target_positions[piece_id])
                    
                    # Skip if already at target
                    if dist < 1.0:
                        continue
                    
                    # Back rank pieces (row 0 or 7) get priority
                    target_col, target_row = None, None
                    for pid, (col, row) in PIECE_START_POSITIONS.items():
                        if pid == piece_id:
                            target_col, target_row = col, row
                            break
                    
                    back_rank_bonus = 100.0 if target_row in [0, 7] else 0.0
                    # Vary the random factor and weighting per attempt
                    random_factor = random.uniform(0, 100)
                    distance_weight = random.uniform(0.5, 2.0)  # Vary importance of distance
                    priority = dist * distance_weight + back_rank_bonus + random_factor
                    
                    priorities.append((priority, piece_id))
                
                if not priorities:
                    break
                
                # Sort by priority (highest first)
                priorities.sort(reverse=True)
                
                # Try to move each piece in priority order
                moved_any = False
                for _, piece_id in priorities:
                    target_pos = optimized_targets[piece_id]
                    current_pos = simulated_positions[piece_id]
                    
                    # Check if already at target
                    if current_pos.distance_to(target_pos) < 1.0:
                        remaining_pieces.discard(piece_id)
                        continue
                    
                    # Try to find a collision-free path
                    sequence = self._plan_piece_movement(
                        piece_id, current_pos, target_pos, simulated_positions,
                        plan, current_time, optimized_targets, remaining_pieces
                    )
                    
                    if sequence and len(sequence.commands) > 0:
                        # Add sequence to plan at current time
                        plan.add_sequence_at_time(sequence, current_time)
                        current_time += sequence.total_duration
                        
                        # Update simulated position
                        self._update_simulated_position(simulated_positions[piece_id], sequence)
                        
                        # Check if reached target
                        if simulated_positions[piece_id].distance_to(target_pos) < 1.0:
                            remaining_pieces.discard(piece_id)
                        
                        moved_any = True
                        break  # Move to next iteration
                
                if not moved_any:
                    # No piece could move, try moving pieces out of the way
                    moved_blocking = self._try_move_blocking_pieces(
                        plan, simulated_positions, optimized_targets, 
                        remaining_pieces, current_time
                    )
                    
                    if moved_blocking:
                        # Update current time and continue
                        current_time = plan.get_total_duration()
                    else:
                        # Still stuck, break out
                        break
            
            # Score this attempt
            total_time = plan.get_total_duration()
            accuracy_score = sum(
                simulated_positions[pid].distance_to(optimized_targets[pid])
                for pid in optimized_targets.keys()
            )
            
            # Perfect accuracy = pieces at target, otherwise prefer better accuracy
            if accuracy_score < 10.0:  # Near perfect
                score = total_time
            else:
                score = accuracy_score * 1000 + total_time
            
            if attempt % 10 == 0:
                print(f"  Attempt {attempt}: time={total_time:.1f}s, accuracy={accuracy_score:.1f}mm, score={score:.1f}")
            
            # Keep best solution
            if score < best_score:
                best_score = score
                best_time = total_time
                best_plan = plan
        
        print(f"Best solution: time={best_time:.1f}s, score={best_score:.1f}")
        return best_plan if best_plan else ExecutionPlan()
    
    def _optimize_target_assignments(self, pieces: Dict[str, Piece], 
                                     target_positions: Dict[str, Position],
                                     simulated_positions: Dict[str, Position]) -> Dict[str, Position]:
        """Optimize target assignments for interchangeable pieces"""
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
        
        # Classify pieces and their targets
        for piece_id in pieces.keys():
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
            
            # Find optimal assignment using greedy matching
            assignments = self._find_optimal_assignment(piece_ids, targets, simulated_positions)
            
            for piece_id, target_pos in assignments.items():
                optimized[piece_id] = target_pos
        
        return optimized
    
    def _find_optimal_assignment(self, piece_ids: List[str], targets: List[Position],
                                simulated_positions: Dict[str, Position]) -> Dict[str, Position]:
        """Find optimal target assignment to minimize total distance"""
        import itertools
        
        if len(piece_ids) != len(targets):
            # Shouldn't happen, but handle gracefully
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
    
    def _random_swap_targets(self, pieces: Dict[str, Piece],
                            target_positions: Dict[str, Position],
                            simulated_positions: Dict[str, Position]) -> Dict[str, Position]:
        """Randomly swap some targets for identical pieces (exploratory strategy)"""
        optimized = target_positions.copy()
        
        # Get piece groups
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
        
        # Classify pieces
        for piece_id in pieces.keys():
            if piece_id not in target_positions:
                continue
            
            if piece_id.startswith('p') and piece_id[1:].isdigit():
                piece_groups['white_pawns'].append(piece_id)
                target_groups['white_pawns'].append(target_positions[piece_id])
            elif piece_id.startswith('P') and piece_id[1:].isdigit():
                piece_groups['black_pawns'].append(piece_id)
                target_groups['black_pawns'].append(target_positions[piece_id])
            elif piece_id in ['r1', 'r2']:
                piece_groups['white_rooks'].append(piece_id)
                target_groups['white_rooks'].append(target_positions[piece_id])
            elif piece_id in ['R1', 'R2']:
                piece_groups['black_rooks'].append(piece_id)
                target_groups['black_rooks'].append(target_positions[piece_id])
            elif piece_id in ['n1', 'n2']:
                piece_groups['white_knights'].append(piece_id)
                target_groups['white_knights'].append(target_positions[piece_id])
            elif piece_id in ['N1', 'N2']:
                piece_groups['black_knights'].append(piece_id)
                target_groups['black_knights'].append(target_positions[piece_id])
            elif piece_id in ['b1', 'b2']:
                piece_groups['white_bishops'].append(piece_id)
                target_groups['white_bishops'].append(target_positions[piece_id])
            elif piece_id in ['B1', 'B2']:
                piece_groups['black_bishops'].append(piece_id)
                target_groups['black_bishops'].append(target_positions[piece_id])
        
        # Randomly shuffle targets within each group
        for group_name in piece_groups.keys():
            piece_ids = piece_groups[group_name]
            targets = target_groups[group_name]
            
            if len(piece_ids) <= 1:
                continue
            
            # Shuffle targets
            shuffled_targets = targets.copy()
            random.shuffle(shuffled_targets)
            
            for i, piece_id in enumerate(piece_ids):
                optimized[piece_id] = shuffled_targets[i]
        
        return optimized
    
    def _plan_piece_movement(self, piece_id: str, current_pos: Position, 
                            target_pos: Position, 
                            simulated_positions: Dict[str, Position],
                            plan: ExecutionPlan,
                            current_time: float,
                            target_positions: Dict[str, Position],
                            remaining_pieces: set) -> Optional[PieceCommandSequence]:
        """Plan movement for a single piece, checking for collisions"""
        
        # Randomly choose strategy order (50% try direct first, 50% try route-around first)
        try_route_first = random.random() < 0.3
        
        if not try_route_first:
            # Try direct path first
            sequence = self._try_direct_path(piece_id, current_pos, target_pos, simulated_positions)
            if sequence:
                return sequence
        else:
            # Try routing around first (more exploratory)
            sequence = self._try_route_around(piece_id, current_pos, target_pos, simulated_positions)
            if sequence:
                return sequence
            # Fall back to direct
            sequence = self._try_direct_path(piece_id, current_pos, target_pos, simulated_positions)
            if sequence:
                return sequence
            return None
        
        # Direct path is blocked - identify what's blocking
        blocker_id = self._find_blocking_piece(piece_id, current_pos, target_pos, simulated_positions)
        
        if blocker_id and blocker_id not in remaining_pieces:
            # The blocker has already reached its target, try to move it out of the way
            if self._try_move_single_blocker(blocker_id, piece_id, plan, simulated_positions, 
                                            target_positions, current_time):
                # Try direct path again after moving blocker
                sequence = self._try_direct_path(piece_id, current_pos, target_pos, simulated_positions)
                if sequence:
                    return sequence
        
        elif blocker_id and blocker_id in remaining_pieces:
            # The blocker also needs to move - try to move it toward its target first
            blocker_target = target_positions.get(blocker_id)
            if blocker_target:
                blocker_pos = simulated_positions[blocker_id]
                blocker_sequence = self._try_direct_path(blocker_id, blocker_pos, blocker_target, simulated_positions)
                
                if blocker_sequence and len(blocker_sequence.commands) > 0:
                    # Move the blocker first
                    plan.add_sequence_at_time(blocker_sequence, current_time)
                    self._update_simulated_position(simulated_positions[blocker_id], blocker_sequence)
                    
                    # Check if blocker reached its target
                    if simulated_positions[blocker_id].distance_to(blocker_target) < 1.0:
                        remaining_pieces.discard(blocker_id)
                    
                    # Try our path again
                    sequence = self._try_direct_path(piece_id, current_pos, target_pos, simulated_positions)
                    if sequence:
                        return sequence
        
        # If still blocked, try routing around obstacles
        sequence = self._try_route_around(piece_id, current_pos, target_pos, simulated_positions)
        if sequence:
            return sequence
        
        return None
    
    def _try_direct_path(self, piece_id: str, current_pos: Position,
                        target_pos: Position,
                        simulated_positions: Dict[str, Position]) -> Optional[PieceCommandSequence]:
        """Try a direct path to target"""
        sequence = PieceCommandSequence(piece_id=piece_id)
        
        # Calculate angle to target
        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 1.0:
            return None
        
        target_angle = math.degrees(math.atan2(dy, dx)) % 360
        
        # Add rotation command
        rotate_cmd = self._create_rotate_command(current_pos.orientation, target_angle)
        sequence.add_command(rotate_cmd)
        
        # Check for collisions along the path
        max_safe_distance = self._find_max_safe_distance(
            current_pos, target_angle, distance, piece_id, simulated_positions
        )
        
        if max_safe_distance > 1.0:
            # Move as far as safely possible
            move_cmd = self._create_move_command(max_safe_distance)
            sequence.add_command(move_cmd)
            return sequence
        
        return None
    
    def _try_route_around(self, piece_id: str, current_pos: Position,
                         target_pos: Position,
                         simulated_positions: Dict[str, Position]) -> Optional[PieceCommandSequence]:
        """Try to route around obstacles by going to an intermediate waypoint"""
        sequence = PieceCommandSequence(piece_id=piece_id)
        
        # Try several intermediate waypoints around the obstacle
        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y
        direct_distance = math.sqrt(dx*dx + dy*dy)
        
        if direct_distance < 1.0:
            return None
        
        # Try waypoints at different angles offset from the direct path
        best_waypoint_sequence = None
        best_waypoint_progress = 0.0
        
        # Randomize which angles to try and in what order
        angle_options = [45, -45, 90, -90, 135, -135]
        random.shuffle(angle_options)
        
        for angle_offset in angle_options:
            # Calculate waypoint perpendicular to direct path
            mid_x = current_pos.x + dx * 0.5
            mid_y = current_pos.y + dy * 0.5
            
            # Offset perpendicular to path
            perp_angle = math.atan2(dy, dx) + math.radians(angle_offset)
            offset_distance = min(100, direct_distance * 0.5)  # 100mm or half distance
            
            waypoint_x = mid_x + offset_distance * math.cos(perp_angle)
            waypoint_y = mid_y + offset_distance * math.sin(perp_angle)
            
            # Check if waypoint is valid
            if not self._is_position_valid(waypoint_x, waypoint_y, piece_id, simulated_positions):
                continue
            
            # Try path to waypoint
            waypoint_pos = Position(waypoint_x, waypoint_y, 0)
            
            # Calculate path to waypoint
            wp_dx = waypoint_x - current_pos.x
            wp_dy = waypoint_y - current_pos.y
            wp_distance = math.sqrt(wp_dx*wp_dx + wp_dy*wp_dy)
            wp_angle = math.degrees(math.atan2(wp_dy, wp_dx)) % 360
            
            # Check if we can reach waypoint
            safe_dist_to_wp = self._find_max_safe_distance(
                current_pos, wp_angle, wp_distance, piece_id, simulated_positions
            )
            
            if safe_dist_to_wp > wp_distance * 0.9:  # Can reach at least 90% of the way
                # Calculate progress toward goal this waypoint gives us
                progress = safe_dist_to_wp / direct_distance
                
                if progress > best_waypoint_progress:
                    test_sequence = PieceCommandSequence(piece_id=piece_id)
                    rotate_cmd = self._create_rotate_command(current_pos.orientation, wp_angle)
                    test_sequence.add_command(rotate_cmd)
                    move_cmd = self._create_move_command(safe_dist_to_wp)
                    test_sequence.add_command(move_cmd)
                    
                    best_waypoint_sequence = test_sequence
                    best_waypoint_progress = progress
        
        return best_waypoint_sequence
    
    def _find_max_safe_distance(self, start_pos: Position, angle: float, 
                               max_distance: float, moving_piece_id: str,
                               simulated_positions: Dict[str, Position]) -> float:
        """Find maximum distance piece can move without collision"""
        angle_rad = math.radians(angle)
        safe_distance = max_distance
        
        # Check collision with every other piece
        for other_id, other_pos in simulated_positions.items():
            if other_id == moving_piece_id:
                continue
            
            # Check multiple points along the path
            for test_dist in np.linspace(0, max_distance, 20):
                test_x = start_pos.x + test_dist * math.cos(angle_rad)
                test_y = start_pos.y + test_dist * math.sin(angle_rad)
                
                dx = test_x - other_pos.x
                dy = test_y - other_pos.y
                dist_to_other = math.sqrt(dx*dx + dy*dy)
                
                if dist_to_other < PIECE_RADIUS * 2 + 5:  # 5mm safety margin
                    safe_distance = min(safe_distance, test_dist - 10)  # Stop before collision
                    break
        
        return max(0, safe_distance)
    
    def _update_simulated_position(self, pos: Position, sequence: PieceCommandSequence):
        """Update a simulated position based on command sequence"""
        for command in sequence.commands:
            if command.command_type == CommandType.ROTATE:
                pos.orientation = command.target_orientation
            elif command.command_type == CommandType.MOVE_STRAIGHT:
                angle_rad = math.radians(pos.orientation)
                pos.x += command.distance * math.cos(angle_rad)
                pos.y += command.distance * math.sin(angle_rad)
    
    def _find_blocking_piece(self, piece_id: str, start_pos: Position, target_pos: Position,
                            simulated_positions: Dict[str, Position]) -> Optional[str]:
        """Find which piece is blocking the direct path"""
        dx = target_pos.x - start_pos.x
        dy = target_pos.y - start_pos.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 1.0:
            return None
        
        angle = math.degrees(math.atan2(dy, dx))
        angle_rad = math.radians(angle)
        
        # Check for pieces along the path
        closest_blocker = None
        closest_dist = float('inf')
        
        for other_id, other_pos in simulated_positions.items():
            if other_id == piece_id:
                continue
            
            # Check multiple points along the path
            for test_t in np.linspace(0.05, 1.0, 20):
                test_x = start_pos.x + distance * test_t * math.cos(angle_rad)
                test_y = start_pos.y + distance * test_t * math.sin(angle_rad)
                
                dx_to_other = test_x - other_pos.x
                dy_to_other = test_y - other_pos.y
                dist_to_other = math.sqrt(dx_to_other*dx_to_other + dy_to_other*dy_to_other)
                
                if dist_to_other < PIECE_RADIUS * 2 + 5:
                    # This piece is blocking
                    path_dist = distance * test_t
                    if path_dist < closest_dist:
                        closest_dist = path_dist
                        closest_blocker = other_id
                    break
        
        return closest_blocker
    
    def _try_move_single_blocker(self, blocker_id: str, blocked_id: str, plan: ExecutionPlan,
                                 simulated_positions: Dict[str, Position],
                                 target_positions: Dict[str, Position],
                                 current_time: float) -> bool:
        """Try to move a single blocking piece out of the way"""
        blocker_pos = simulated_positions[blocker_id]
        blocked_pos = simulated_positions[blocked_id]
        blocked_target = target_positions[blocked_id]
        
        # Calculate perpendicular direction to move blocker
        path_angle = math.atan2(blocked_target.y - blocked_pos.y, blocked_target.x - blocked_pos.x)
        
        # Randomize escape angles
        perp_offsets = [90, -90, 45, -45, 135, -135]
        random.shuffle(perp_offsets)
        
        for perp_offset in perp_offsets:
            move_angle = path_angle + math.radians(perp_offset)
            move_distance = random.uniform(80, 120)
            
            temp_x = blocker_pos.x + move_distance * math.cos(move_angle)
            temp_y = blocker_pos.y + move_distance * math.sin(move_angle)
            
            if self._is_position_valid(temp_x, temp_y, blocker_id, simulated_positions):
                temp_pos = Position(temp_x, temp_y, 0)
                sequence = self._try_direct_path(
                    blocker_id, blocker_pos, temp_pos, simulated_positions
                )
                
                if sequence and len(sequence.commands) > 0:
                    plan.add_sequence_at_time(sequence, current_time)
                    self._update_simulated_position(simulated_positions[blocker_id], sequence)
                    print(f"  Moved blocker {blocker_id} out of way for {blocked_id}")
                    return True
        
        return False
    
    def _try_move_blocking_pieces(self, plan: ExecutionPlan, 
                                  simulated_positions: Dict[str, Position],
                                  target_positions: Dict[str, Position],
                                  remaining_pieces: set, 
                                  current_time: float) -> bool:
        """Try to move pieces that are blocking others out of the way"""
        
        # Find pieces that are blocking the path of remaining pieces
        blocking_info = []
        
        for blocked_id in remaining_pieces:
            blocked_pos = simulated_positions[blocked_id]
            target_pos = target_positions[blocked_id]
            
            # Calculate direct path
            dx = target_pos.x - blocked_pos.x
            dy = target_pos.y - blocked_pos.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 1.0:
                continue
            
            target_angle = math.degrees(math.atan2(dy, dx))
            angle_rad = math.radians(target_angle)
            
            # Check which pieces are in the way
            for other_id, other_pos in simulated_positions.items():
                if other_id == blocked_id or other_id in remaining_pieces:
                    continue
                
                # Check if this piece is near the path
                for test_t in np.linspace(0.1, 0.9, 9):
                    test_x = blocked_pos.x + distance * test_t * math.cos(angle_rad)
                    test_y = blocked_pos.y + distance * test_t * math.sin(angle_rad)
                    
                    dist_to_blocker = math.sqrt((test_x - other_pos.x)**2 + (test_y - other_pos.y)**2)
                    
                    if dist_to_blocker < PIECE_RADIUS * 2 + 20:  # Within blocking distance
                        blocking_info.append((other_id, blocked_id, dist_to_blocker))
                        break
        
        # Try to move blocking pieces
        if blocking_info:
            # Sort by how much they're blocking (closest blockers first)
            blocking_info.sort(key=lambda x: x[2])
            
            for blocker_id, blocked_id, _ in blocking_info[:3]:  # Try top 3 blockers
                blocker_pos = simulated_positions[blocker_id]
                
                # Try to move blocker to a safe position away from the path
                blocked_pos = simulated_positions[blocked_id]
                target_pos = target_positions[blocked_id]
                
                # Calculate perpendicular direction to move blocker
                path_angle = math.atan2(target_pos.y - blocked_pos.y, target_pos.x - blocked_pos.x)
                
                for perp_offset in [90, -90, 45, -45, 135, -135]:
                    move_angle = path_angle + math.radians(perp_offset)
                    move_distance = random.uniform(80, 120)
                    
                    temp_x = blocker_pos.x + move_distance * math.cos(move_angle)
                    temp_y = blocker_pos.y + move_distance * math.sin(move_angle)
                    
                    if self._is_position_valid(temp_x, temp_y, blocker_id, simulated_positions):
                        temp_pos = Position(temp_x, temp_y, 0)
                        sequence = self._try_direct_path(
                            blocker_id, blocker_pos, temp_pos, simulated_positions
                        )
                        
                        if sequence and len(sequence.commands) > 0:
                            plan.add_sequence_at_time(sequence, current_time)
                            self._update_simulated_position(simulated_positions[blocker_id], sequence)
                            print(f"  Moved blocker {blocker_id} out of way for {blocked_id}")
                            return True
        
        return False
    
    def _is_position_valid(self, x: float, y: float, piece_id: str,
                          simulated_positions: Dict[str, Position]) -> bool:
        """Check if a position is on the board and collision-free"""
        # Check board bounds
        board_width = 8 * BOARD_SQUARE_SIZE
        board_height = 8 * BOARD_SQUARE_SIZE
        
        if x < BOARD_EXTRA_SIDE or x > BOARD_EXTRA_SIDE + board_width:
            return False
        if y < 0 or y > board_height:
            return False
        
        # Check collisions
        for other_id, other_pos in simulated_positions.items():
            if other_id == piece_id:
                continue
            
            dx = x - other_pos.x
            dy = y - other_pos.y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < PIECE_RADIUS * 2 + 5:
                return False
        
        return True
    
    def get_name(self) -> str:
        return "AI Planner V1"