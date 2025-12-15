"""
Visual Simulation-Based Path Planner
Uses potential fields and incremental simulation for collision-free parallel movement
"""

import math
import numpy as np
from typing import Dict, List, Optional, Tuple, Set
from .path_planner import PathPlanner
from .data_types import Piece, Position, ExecutionPlan, PieceCommandSequence, CommandType, PieceCommand
from .constants import (PIECE_START_POSITIONS, PIECE_INTERMEDIATE_POSITIONS, 
                       PIECE_RADIUS, BOARD_SQUARE_SIZE, BOARD_EXTRA_SIDE,
                       LINEAR_VELOCITY, ANGULAR_VELOCITY)


class VisualSimulationPlanner(PathPlanner):
    """Visual simulation-based planner with real-time obstacle avoidance"""
    
    def __init__(self, time_step: float = 0.2, max_simulation_time: float = 120.0):
        """
        Initialize visual simulation planner
        
        Args:
            time_step: Simulation time increment in seconds (default 0.2s)
            max_simulation_time: Maximum simulation time in seconds
        """
        self.time_step = time_step
        self.max_simulation_time = max_simulation_time
        self.repulsion_distance = 80.0  # mm - distance at which pieces repel
        self.goal_attraction = 2.0  # Strength of goal attraction
        self.obstacle_repulsion = 150.0  # Strength of obstacle repulsion
        self.stuck_threshold = 3  # Iterations without progress before considering stuck
        
    def plan_movements(self, pieces: Dict[str, Piece], 
                       target_positions: Dict[str, Position]) -> ExecutionPlan:
        """
        Plan movements using visual simulation with potential fields
        """
        print(f"\n=== Visual Simulation Planner Starting ===")
        print(f"Time step: {self.time_step}s, Max time: {self.max_simulation_time}s")
        
        # Create intermediate targets for pawns
        intermediate_targets = self._create_intermediate_targets(target_positions)
        
        # Initialize simulation state
        sim_positions = {pid: piece.position.copy() for pid, piece in pieces.items()}
        sim_orientations = {pid: piece.position.orientation for pid, piece in pieces.items()}
        
        # Track which phase each piece is in (intermediate vs final)
        piece_phase = {pid: "intermediate" for pid in target_positions.keys()}
        
        # Track stuck pieces
        stuck_counter = {pid: 0 for pid in target_positions.keys()}
        last_positions = {pid: sim_positions[pid].copy() for pid in target_positions.keys()}
        
        # Recording of movements for plan creation
        movement_history = {pid: [(0.0, sim_positions[pid].copy(), sim_orientations[pid])] 
                           for pid in target_positions.keys()}
        
        current_time = 0.0
        iteration = 0
        
        print(f"\nStarting simulation with {len(target_positions)} pieces...")
        print(f"Time step: {self.time_step}s, Max time: {self.max_simulation_time}s")
        print(f"\n[DEBUG] Sample starting positions vs targets:")
        sample_pieces = [pid for pid in ['wp1', 'wp2', 'wR1', 'bR1'] if pid in target_positions][:4]
        if not sample_pieces:
            sample_pieces = list(target_positions.keys())[:4]
        for piece_id in sample_pieces:
            start = sim_positions[piece_id]
            inter = intermediate_targets[piece_id]
            final = target_positions[piece_id]
            dist_to_inter = start.distance_to(inter)
            dist_to_final = start.distance_to(final)
            print(f"  {piece_id}: start({start.x:.0f},{start.y:.0f}) dist_to_inter={dist_to_inter:.1f}mm dist_to_final={dist_to_final:.1f}mm")
        
        while current_time < self.max_simulation_time:
            if iteration < 5 or iteration % 25 == 0:
                print(f"\n{'='*60}")
                print(f"Iteration {iteration} at t={current_time:.1f}s")
                print(f"{'='*60}")
            iteration += 1
            current_time += self.time_step
            
            # Determine current targets (intermediate or final)
            current_targets = {}
            for piece_id in target_positions.keys():
                if piece_phase[piece_id] == "intermediate":
                    current_targets[piece_id] = intermediate_targets[piece_id]
                else:
                    current_targets[piece_id] = target_positions[piece_id]
            
            # Optimize target assignments for identical pieces
            optimized_targets = self._optimize_target_assignments(
                set(target_positions.keys()), sim_positions, current_targets
            )
            
            # Calculate desired movements for all pieces
            movements = {}
            pieces_at_final_target = 0
            pieces_at_intermediate_target = 0
            
            # Debug first iteration
            if iteration == 0:
                print(f"\n[DEBUG] First iteration - checking initial distances:")
                sample_pieces = [pid for pid in list(target_positions.keys())[:4]]
                for piece_id in sample_pieces:
                    curr = sim_positions[piece_id]
                    inter = intermediate_targets[piece_id]
                    final = target_positions[piece_id]
                    dist_to_inter = curr.distance_to(inter)
                    dist_to_final = curr.distance_to(final)
                    print(f"  {piece_id}: phase={piece_phase[piece_id]}, dist_to_inter={dist_to_inter:.1f}mm, dist_to_final={dist_to_final:.1f}mm")
            
            for piece_id in target_positions.keys():
                current_pos = sim_positions[piece_id]
                target_pos = optimized_targets[piece_id]
                
                # Check if at target
                distance_to_target = current_pos.distance_to(target_pos)
                
                # Debug phase transitions
                old_phase = piece_phase[piece_id]
                
                # Debug first few iterations
                sample_debug_pieces = [pid for pid in ['wp1', 'wR1'] if pid in target_positions]
                if iteration <= 2 and piece_id in sample_debug_pieces:
                    print(f"\n[DEBUG] {piece_id} (iter {iteration}):")
                    print(f"  Current pos: ({current_pos.x:.1f}, {current_pos.y:.1f})")
                    print(f"  Target pos: ({target_pos.x:.1f}, {target_pos.y:.1f})")
                    print(f"  Distance: {distance_to_target:.2f}mm")
                    print(f"  Current phase: {piece_phase[piece_id]}")
                
                if distance_to_target < 2.0:  # Within 2mm tolerance
                    # Check if should transition to final phase
                    if piece_phase[piece_id] == "intermediate":
                        piece_phase[piece_id] = "final"
                        print(f"  [TRANSITION] {piece_id}: intermediate->final at t={current_time:.1f}s, dist={distance_to_target:.2f}mm")
                        # Don't continue - recalculate target for final phase
                    elif piece_phase[piece_id] == "final":
                        # Already at final target
                        pieces_at_final_target += 1
                        movements[piece_id] = (0.0, 0.0)  # No movement needed
                        continue
                else:
                    # Not at target, need to move
                    if piece_phase[piece_id] == "intermediate":
                        pieces_at_intermediate_target += 1
                
                # Recalculate target in case phase just changed
                if piece_phase[piece_id] == "intermediate":
                    target_pos = optimized_targets[piece_id]  # Should be intermediate
                else:
                    # Get final target
                    final_targets_temp = {piece_id: target_positions[piece_id]}
                    target_pos = target_positions[piece_id]
                    if old_phase != piece_phase[piece_id]:
                        print(f"    -> New target for {piece_id}: ({target_pos.x:.0f},{target_pos.y:.0f})")
                
                # Calculate desired movement using potential fields
                desired_movement = self._calculate_potential_field_movement(
                    piece_id, current_pos, target_pos, sim_positions, optimized_targets, stuck_counter.get(piece_id, 0)
                )
                
                # Debug first few iterations for sample pieces
                sample_move_pieces = [pid for pid in ['wp1', 'wp2', 'wR1', 'bR1'] if pid in target_positions]
                if iteration < 3 and piece_id in sample_move_pieces:
                    move_mag = (desired_movement[0]**2 + desired_movement[1]**2)**0.5
                    print(f"  [MOVE] {piece_id}: phase={piece_phase[piece_id]}, dist={distance_to_target:.1f}mm, move={move_mag:.1f}mm")
                
                movements[piece_id] = desired_movement
            
            # Check if all pieces reached final targets
            if pieces_at_final_target == len(target_positions):
                print(f"\n✓ All pieces reached final targets at {current_time:.1f}s")
                break
            
            # Execute movements and update positions
            moves_applied = 0
            collisions_detected = 0
            for piece_id, (dx, dy) in movements.items():
                old_pos = sim_positions[piece_id].copy()
                
                # Calculate new position
                new_x = old_pos.x + dx
                new_y = old_pos.y + dy
                
                move_dist = math.sqrt(dx*dx + dy*dy)
                
                # Check if movement is safe
                if self._is_position_safe(piece_id, new_x, new_y, sim_positions):
                    sim_positions[piece_id].x = new_x
                    sim_positions[piece_id].y = new_y
                    moves_applied += 1
                    
                    # Update orientation to face movement direction
                    if abs(dx) > 0.1 or abs(dy) > 0.1:
                        new_orientation = math.degrees(math.atan2(dy, dx)) % 360
                        sim_orientations[piece_id] = new_orientation
                        sim_positions[piece_id].orientation = new_orientation
                    
                    # Reset stuck counter if moved significantly
                    move_dist = math.sqrt(dx*dx + dy*dy)
                    if move_dist > 1.0:
                        stuck_counter[piece_id] = 0
                    else:
                        stuck_counter[piece_id] = stuck_counter.get(piece_id, 0) + 1
                else:
                    # Couldn't move - increment stuck counter
                    stuck_counter[piece_id] = stuck_counter.get(piece_id, 0) + 1
                    collisions_detected += 1
                    if iteration <= 5 or stuck_counter[piece_id] % 15 == 0:
                        print(f"  [BLOCKED] {piece_id}: can't move to ({new_x:.0f},{new_y:.0f}), stuck={stuck_counter[piece_id]}")
                
                # Always record position in history (whether moved or stayed in place)
                movement_history[piece_id].append(
                    (current_time, sim_positions[piece_id].copy(), sim_orientations[piece_id])
                )
            
            # Detailed progress reporting
            if iteration <= 5 or iteration % 25 == 0:
                phase_inter = sum(1 for p in piece_phase.values() if p == 'intermediate')
                phase_final = sum(1 for p in piece_phase.values() if p == 'final')
                print(f"\n[PROGRESS] After iteration {iteration}:")
                print(f"  Pieces at final target: {pieces_at_final_target}/{len(target_positions)}")
                print(f"  Pieces at intermediate: {pieces_at_intermediate_target}")
                print(f"  Pieces in intermediate phase: {phase_inter}")
                print(f"  Pieces in final phase: {phase_final}")
                print(f"  Moves attempted: {len(movements)}")
                print(f"  Moves applied: {moves_applied}")
                stuck_count = sum(1 for c in stuck_counter.values() if c > 10)
                print(f"  Stuck pieces (>10): {stuck_count}")
        
        # Check if reached max time
        if current_time >= self.max_simulation_time:
            print(f"\n{'='*60}")
            print(f"⚠ TIMEOUT after {self.max_simulation_time}s")
            print(f"{'='*60}")
            print(f"Pieces at final: {pieces_at_final_target}/{len(target_positions)}")
            print(f"\n[DEBUG] Phase distribution:")
            inter_phases = [pid for pid, phase in piece_phase.items() if phase == 'intermediate']
            final_phases = [pid for pid, phase in piece_phase.items() if phase == 'final']
            print(f"  In intermediate phase: {len(inter_phases)}")
            if len(inter_phases) <= 10:
                print(f"    {inter_phases}")
            print(f"  In final phase: {len(final_phases)}")
            if len(final_phases) <= 10:
                print(f"    {final_phases}")
            print(f"\n[DEBUG] Most stuck pieces:")
            stuck_list = sorted(stuck_counter.items(), key=lambda x: x[1], reverse=True)[:6]
            for pid, cnt in stuck_list:
                curr = sim_positions[pid]
                inter = intermediate_targets[pid]
                final = target_positions[pid]
                dist_inter = curr.distance_to(inter)
                dist_final = curr.distance_to(final)
                print(f"  {pid}: stuck={cnt}, phase={piece_phase[pid]}, dist_inter={dist_inter:.1f}mm, dist_final={dist_final:.1f}mm")
        
        # Convert movement history to execution plan
        print(f"\n=== Creating Execution Plan ===")
        plan = self._create_plan_from_history(movement_history)
        
        total_time = plan.get_total_duration()
        print(f"Total execution time: {total_time:.1f}s")
        
        return plan
    
    def _create_intermediate_targets(self, target_positions: Dict[str, Position]) -> Dict[str, Position]:
        """Create intermediate targets using PIECE_INTERMEDIATE_POSITIONS"""
        from .utils import board_coords_to_world
        intermediate = {}
        
        for piece_id, target_pos in target_positions.items():
            if piece_id in PIECE_INTERMEDIATE_POSITIONS:
                col, row = PIECE_INTERMEDIATE_POSITIONS[piece_id]
                x, y = board_coords_to_world(col, row)
                intermediate[piece_id] = Position(x, y, target_pos.orientation)
            else:
                intermediate[piece_id] = target_pos.copy()
        
        return intermediate
    
    def _calculate_potential_field_movement(self, piece_id: str, current_pos: Position,
                                           target_pos: Position, all_positions: Dict[str, Position],
                                           all_targets: Dict[str, Position],
                                           stuck_count: int) -> Tuple[float, float]:
        """Calculate movement using potential field approach"""
        # Attractive force toward goal
        dx_goal = target_pos.x - current_pos.x
        dy_goal = target_pos.y - current_pos.y
        dist_to_goal = math.sqrt(dx_goal*dx_goal + dy_goal*dy_goal)
        
        if dist_to_goal < 0.1:
            return (0.0, 0.0)
        
        # Normalize goal direction
        goal_fx = (dx_goal / dist_to_goal) * self.goal_attraction
        goal_fy = (dy_goal / dist_to_goal) * self.goal_attraction
        
        # Repulsive forces from other pieces
        repulsion_fx = 0.0
        repulsion_fy = 0.0
        
        for other_id, other_pos in all_positions.items():
            if other_id == piece_id:
                continue
            
            dx_other = current_pos.x - other_pos.x
            dy_other = current_pos.y - other_pos.y
            dist_to_other = math.sqrt(dx_other*dx_other + dy_other*dy_other)
            
            if dist_to_other < self.repulsion_distance and dist_to_other > 0.1:
                # Stronger repulsion when closer
                repulsion_strength = self.obstacle_repulsion * (1.0 - dist_to_other / self.repulsion_distance)
                repulsion_fx += (dx_other / dist_to_other) * repulsion_strength
                repulsion_fy += (dy_other / dist_to_other) * repulsion_strength
                
                # Extra strong repulsion if very close
                if dist_to_other < PIECE_RADIUS * 2 + 10:
                    repulsion_fx += (dx_other / dist_to_other) * self.obstacle_repulsion * 2
                    repulsion_fy += (dy_other / dist_to_other) * self.obstacle_repulsion * 2
        
        # If stuck, add random perturbation to escape local minimum
        random_fx = 0.0
        random_fy = 0.0
        if stuck_count > self.stuck_threshold:
            import random
            angle = random.random() * 2 * math.pi
            random_fx = math.cos(angle) * 0.5
            random_fy = math.sin(angle) * 0.5
        
        # Combine forces
        total_fx = goal_fx + repulsion_fx + random_fx
        total_fy = goal_fy + repulsion_fy + random_fy
        
        # Convert to actual movement (limit by velocity)
        max_distance = LINEAR_VELOCITY * self.time_step / 1000.0  # Convert to mm
        force_magnitude = math.sqrt(total_fx*total_fx + total_fy*total_fy)
        
        if force_magnitude > 0.01:
            # Normalize and scale by max distance
            move_x = (total_fx / force_magnitude) * min(max_distance, force_magnitude * 10)
            move_y = (total_fy / force_magnitude) * min(max_distance, force_magnitude * 10)
        else:
            move_x = 0.0
            move_y = 0.0
        
        return (move_x, move_y)
    
    def _is_position_safe(self, piece_id: str, x: float, y: float,
                         all_positions: Dict[str, Position]) -> bool:
        """Check if a position is safe (no collisions)"""
        # Check board bounds
        if x < BOARD_EXTRA_SIDE or x > BOARD_EXTRA_SIDE + 8 * BOARD_SQUARE_SIZE:
            return False
        if y < 0 or y > 8 * BOARD_SQUARE_SIZE:
            return False
        
        # Check collisions with other pieces
        for other_id, other_pos in all_positions.items():
            if other_id == piece_id:
                continue
            
            dx = x - other_pos.x
            dy = y - other_pos.y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < PIECE_RADIUS * 2 + 2:  # 2mm safety margin
                return False
        
        return True
    
    def _optimize_target_assignments(self, remaining_pieces: Set[str],
                                     simulated_positions: Dict[str, Position],
                                     target_positions: Dict[str, Position]) -> Dict[str, Position]:
        """Optimize target assignments for identical pieces"""
        import itertools
        
        optimized = target_positions.copy()
        
        # Group pieces by type (same logic as A* planner)
        piece_groups = {
            'white_pawns': [], 'black_pawns': [],
            'white_rooks': [], 'black_rooks': [],
            'white_knights': [], 'black_knights': [],
            'white_bishops': [], 'black_bishops': [],
        }
        
        target_groups = {
            'white_pawns': [], 'black_pawns': [],
            'white_rooks': [], 'black_rooks': [],
            'white_knights': [], 'black_knights': [],
            'white_bishops': [], 'black_bishops': [],
        }
        
        for piece_id in remaining_pieces:
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
        
        # Optimize each group
        for group_name in piece_groups.keys():
            piece_ids = piece_groups[group_name]
            targets = target_groups[group_name]
            
            if len(piece_ids) <= 1:
                continue
            
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
        
        for target_permutation in itertools.permutations(targets):
            total_distance = 0
            for i, piece_id in enumerate(piece_ids):
                distance = simulated_positions[piece_id].distance_to(target_permutation[i])
                total_distance += distance
            
            if total_distance < best_total_distance:
                best_total_distance = total_distance
                best_assignment = {piece_ids[i]: target_permutation[i] for i in range(len(piece_ids))}
        
        return best_assignment
    
    def _create_plan_from_history(self, movement_history: Dict[str, List[Tuple[float, Position, float]]]) -> ExecutionPlan:
        """Convert movement history into execution plan"""
        plan = ExecutionPlan()
        
        print(f"\n[DEBUG] Converting movement history to execution plan...")
        
        for piece_id, history in movement_history.items():
            if len(history) < 2:
                print(f"  {piece_id}: Skipping (history too short: {len(history)} entries)")
                continue
            
            sequence = PieceCommandSequence(piece_id=piece_id)
            
            # Get start and end positions
            start_time, start_pos, start_orient = history[0]
            end_time, end_pos, end_orient = history[-1]
            
            # Calculate total distance traveled
            total_distance = 0.0
            for i in range(1, len(history)):
                prev_pos = history[i-1][1]
                curr_pos = history[i][1]
                dx = curr_pos.x - prev_pos.x
                dy = curr_pos.y - prev_pos.y
                total_distance += math.sqrt(dx*dx + dy*dy)
            
            # Initial rotation to face first movement direction if needed
            if len(history) > 1:
                first_pos = history[0][1]
                second_pos = history[1][1]
                dx = second_pos.x - first_pos.x
                dy = second_pos.y - first_pos.y
                if abs(dx) > 0.1 or abs(dy) > 0.1:
                    target_orientation = math.degrees(math.atan2(dy, dx)) % 360
                    angle_diff = abs((target_orientation - start_orient + 180) % 360 - 180)
                    if angle_diff > 5.0:
                        rotate_cmd = self._create_rotate_command(start_orient, target_orientation)
                        sequence.add_command(rotate_cmd)
            
            # Create movement commands by grouping consecutive movements in same direction
            if total_distance > 1.0:  # Only if piece moved significantly overall
                i = 0
                while i < len(history) - 1:
                    segment_start_pos = history[i][1]
                    segment_start_orient = history[i][2]
                    
                    # Find consecutive movements in roughly the same direction
                    segment_distance = 0.0
                    j = i + 1
                    while j < len(history):
                        prev_pos = history[j-1][1]
                        curr_pos = history[j][1]
                        dx = curr_pos.x - prev_pos.x
                        dy = curr_pos.y - prev_pos.y
                        step_dist = math.sqrt(dx*dx + dy*dy)
                        
                        if step_dist > 0.1:  # Piece moved
                            segment_distance += step_dist
                            j += 1
                        else:
                            # Piece didn't move this step
                            break
                    
                    # Create command for this segment if significant
                    if segment_distance > 1.0:
                        # Rotate to face segment direction
                        segment_end_pos = history[j-1][1]
                        dx_seg = segment_end_pos.x - segment_start_pos.x
                        dy_seg = segment_end_pos.y - segment_start_pos.y
                        if abs(dx_seg) > 0.1 or abs(dy_seg) > 0.1:
                            target_orient = math.degrees(math.atan2(dy_seg, dx_seg)) % 360
                            angle_diff = abs((target_orient - segment_start_orient + 180) % 360 - 180)
                            if angle_diff > 5.0:
                                rotate_cmd = self._create_rotate_command(segment_start_orient, target_orient)
                                sequence.add_command(rotate_cmd)
                        
                        # Move forward
                        move_cmd = self._create_move_command(segment_distance)
                        sequence.add_command(move_cmd)
                    
                    i = max(i + 1, j)
            
            # Final orientation correction
            angle_diff = abs((end_orient - start_orient + 180) % 360 - 180)
            if angle_diff > 5.0 and total_distance > 1.0:
                # Get orientation from second-to-last position
                if len(sequence.commands) > 0:
                    second_last_orient = history[-2][2] if len(history) > 1 else start_orient
                    final_angle_diff = abs((end_orient - second_last_orient + 180) % 360 - 180)
                    if final_angle_diff > 5.0:
                        rotate_cmd = self._create_rotate_command(second_last_orient, end_orient)
                        sequence.add_command(rotate_cmd)
            
            if len(sequence.commands) > 0:
                plan.add_sequence_at_time(sequence, 0.0)
                print(f"  {piece_id}: {len(sequence.commands)} commands, total_dist={total_distance:.1f}mm")
            else:
                print(f"  {piece_id}: No commands (total_dist={total_distance:.1f}mm)")
        
        return plan
    
    def _create_rotate_command(self, start_angle: float, target_angle: float) -> PieceCommand:
        """Create a rotation command"""
        # Calculate shortest rotation direction
        angle_diff = (target_angle - start_angle + 180) % 360 - 180
        duration = abs(angle_diff) / ANGULAR_VELOCITY
        
        return PieceCommand(
            command_type=CommandType.ROTATE,
            target_angle=target_angle,
            duration=duration
        )
    
    def _create_move_command(self, distance: float) -> PieceCommand:
        """Create a movement command"""
        duration = distance / LINEAR_VELOCITY
        
        return PieceCommand(
            command_type=CommandType.MOVE_STRAIGHT,
            distance=distance,
            duration=duration
        )
    
    def get_name(self) -> str:
        return "Visual Simulation Planner"
