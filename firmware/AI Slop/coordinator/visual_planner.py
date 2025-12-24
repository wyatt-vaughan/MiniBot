"""
Visual Simulation-Based Path Planner
Uses phased straight-line movement with collision avoidance
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
    """Visual simulation-based planner with phased straight-line movement"""
    
    def __init__(self, time_step: float = 0.2, max_simulation_time: float = 120.0):
        """Initialize visual simulation planner"""
        self.time_step = time_step
        self.max_simulation_time = max_simulation_time
        
    def plan_movements(self, pieces: Dict[str, Piece], 
                       target_positions: Dict[str, Position]) -> ExecutionPlan:
        """
        Plan movements using a phased approach:
        1. Move back-rank pieces to final positions, pawns to side staging areas
        2. Move pawns from staging to final positions
        3. Use A* for any remaining blocked pieces
        """
        print(f"\n{'='*70}")
        print(f"=== Visual Simulation Planner - Phased Movement Strategy ===")
        print(f"{'='*70}")
        print(f"Total pieces to move: {len(target_positions)}")
        
        # Create intermediate targets (side staging areas for pawns)
        intermediate_targets = self._create_side_staging_targets(pieces, target_positions)
        
        # Initialize simulation state
        sim_positions = {pid: piece.position.copy() for pid, piece in pieces.items()}
        sim_orientations = {pid: piece.position.orientation for pid, piece in pieces.items()}
        
        # Track movement history
        movement_history = {pid: [(0.0, sim_positions[pid].copy(), sim_orientations[pid])] 
                           for pid in target_positions.keys()}
        
        current_time = 0.0
        time_step = 0.2  # seconds
        max_time = 60.0  # Max time for each phase
        
        # PHASE 1: Move to intermediate positions
        print(f"\n{'='*70}")
        print(f"PHASE 1: Moving back-rank pieces to finals, pawns to staging areas")
        print(f"{'='*70}")
        
        phase1_targets = {}
        for pid in target_positions.keys():
            if self._is_pawn(pid):
                phase1_targets[pid] = intermediate_targets[pid]
                print(f"  {pid} -> Staging at ({intermediate_targets[pid].x:.0f}, {intermediate_targets[pid].y:.0f})")
            else:
                phase1_targets[pid] = target_positions[pid]
                print(f"  {pid} -> Final at ({target_positions[pid].x:.0f}, {target_positions[pid].y:.0f})")
        
        sim_positions, movement_history, current_time = self._simulate_straight_line_movement(
            sim_positions, sim_orientations, phase1_targets, movement_history, 
            current_time, time_step, max_time, "PHASE 1"
        )
        
        # PHASE 2: Move pawns to final positions
        print(f"\n{'='*70}")
        print(f"PHASE 2: Moving pawns from staging to final positions")
        print(f"{'='*70}")
        
        phase2_targets = {}
        for pid in target_positions.keys():
            if self._is_pawn(pid):
                phase2_targets[pid] = target_positions[pid]
                curr = sim_positions[pid]
                print(f"  {pid}: ({curr.x:.0f},{curr.y:.0f}) -> ({target_positions[pid].x:.0f},{target_positions[pid].y:.0f})")
        
        if phase2_targets:
            sim_positions, movement_history, current_time = self._simulate_straight_line_movement(
                sim_positions, sim_orientations, phase2_targets, movement_history,
                current_time, time_step, max_time, "PHASE 2"
            )
        
        # PHASE 3: Check for pieces that didn't reach goal, use A* for them
        print(f"\n{'='*70}")
        print(f"PHASE 3: Checking for blocked pieces")
        print(f"{'='*70}")
        
        blocked_pieces = {}
        for pid, target in target_positions.items():
            curr = sim_positions[pid]
            distance = curr.distance_to(target)
            if distance > 5.0:  # Not at target (5mm tolerance)
                blocked_pieces[pid] = pieces[pid]
                print(f"  {pid}: Blocked, {distance:.1f}mm from target")
        
        astar_plan = None
        if blocked_pieces:
            print(f"\nUsing A* planner for {len(blocked_pieces)} blocked pieces...")
            # Use A* planner as fallback
            from .astar_planner import AStarPlanner
            astar = AStarPlanner()
            
            # Update piece positions to simulated positions
            for pid in blocked_pieces:
                blocked_pieces[pid].position = sim_positions[pid].copy()
            
            blocked_targets = {pid: target_positions[pid] for pid in blocked_pieces}
            astar_plan = astar.plan_movements(blocked_pieces, blocked_targets)
            print(f"  A* generated plan with {len(astar_plan.sequences)} sequences")
        
        # Convert movement history to execution plan
        print(f"\n{'='*70}")
        print(f"Creating execution plan from movement history...")
        print(f"{'='*70}")
        plan = self._create_plan_from_history(movement_history)
        
        # Add A* plan for blocked pieces
        if astar_plan and astar_plan.sequences:
            print(f"Merging A* plan for blocked pieces...")
            for pid, sequences in astar_plan.sequences.items():
                for seq in sequences:
                    seq.start_time = current_time  # Start after simulation
                    plan.add_sequence_at_time(seq, current_time)
        
        total_time = plan.get_total_duration()
        print(f"\n{'='*70}")
        print(f"Planning complete! Total execution time: {total_time:.1f}s")
        print(f"Total sequences: {len(plan.sequences)}")
        print(f"{'='*70}\n")
        
        return plan
    
    def _is_pawn(self, piece_id: str) -> bool:
        """Check if piece is a pawn"""
        return piece_id.startswith('p') or piece_id.startswith('P')
    
    def _create_side_staging_targets(self, pieces: Dict[str, Piece], 
                                     target_positions: Dict[str, Position]) -> Dict[str, Position]:
        """Create staging targets on sides of board for pawns"""
        from .utils import board_coords_to_world
        staging = {}
        
        white_pawn_count = 0
        black_pawn_count = 0
        
        for pid in target_positions.keys():
            if self._is_pawn(pid):
                if pid.startswith('p'):  # White pawn - left side
                    # Position on left side (column -0.5, varying rows)
                    y = 50 + white_pawn_count * 60  # Spread out vertically
                    x = BOARD_EXTRA_SIDE - 40  # 40mm from board edge
                    staging[pid] = Position(x, y, 0)
                    white_pawn_count += 1
                else:  # Black pawn - right side
                    y = 50 + black_pawn_count * 60
                    x = BOARD_EXTRA_SIDE + 8 * BOARD_SQUARE_SIZE + 40
                    staging[pid] = Position(x, y, 180)
                    black_pawn_count += 1
            else:
                # Non-pawns use their final position as staging
                staging[pid] = target_positions[pid]
        
        return staging
    
    def _simulate_straight_line_movement(self, sim_positions: Dict[str, Position],
                                         sim_orientations: Dict[str, float],
                                         targets: Dict[str, Position],
                                         movement_history: Dict[str, List],
                                         start_time: float, time_step: float,
                                         max_time: float, phase_name: str) -> Tuple:
        """
        Simulate straight-line movement toward targets with collision avoidance
        Returns updated positions, history, and time
        """
        print(f"\nStarting {phase_name} simulation...")
        
        current_time = start_time
        iteration = 0
        stuck_counter = {pid: 0 for pid in targets.keys()}
        
        while current_time - start_time < max_time:
            iteration += 1
            
            # Check how many pieces are at target
            pieces_at_target = 0
            pieces_blocked = 0
            
            for pid in targets.keys():
                dist = sim_positions[pid].distance_to(targets[pid])
                if dist < 2.0:
                    pieces_at_target += 1
                elif stuck_counter[pid] > 10:
                    pieces_blocked += 1
            
            # Progress report every 2 seconds
            if iteration % 10 == 0:
                print(f"  t={current_time:.1f}s: {pieces_at_target} at target, {pieces_blocked} blocked, {len(targets)-pieces_at_target-pieces_blocked} moving")
            
            # All pieces either at target or blocked - done with phase
            if pieces_at_target + pieces_blocked >= len(targets):
                print(f"  {phase_name} complete at t={current_time:.1f}s: {pieces_at_target} reached, {pieces_blocked} blocked")
                break
            
            # Calculate desired movements for each piece
            for pid in targets.keys():
                curr_pos = sim_positions[pid]
                target_pos = targets[pid]
                
                # Check if at target
                dist = curr_pos.distance_to(target_pos)
                if dist < 2.0:
                    continue  # Already at target
                
                # Calculate straight-line direction
                dx = target_pos.x - curr_pos.x
                dy = target_pos.y - curr_pos.y
                
                # Desired movement distance this step
                max_move = LINEAR_VELOCITY * time_step
                move_dist = min(max_move, dist)
                
                # Normalize direction
                move_x = (dx / dist) * move_dist
                move_y = (dy / dist) * move_dist
                
                # Try to move - check for collisions
                new_x = curr_pos.x + move_x
                new_y = curr_pos.y + move_y
                
                # Check if movement is safe
                if self._is_position_safe(pid, new_x, new_y, sim_positions):
                    # Move succeeded
                    sim_positions[pid].x = new_x
                    sim_positions[pid].y = new_y
                    stuck_counter[pid] = 0
                    
                    # Update orientation
                    if abs(dx) > 0.1 or abs(dy) > 0.1:
                        orient = math.degrees(math.atan2(dy, dx)) % 360
                        sim_orientations[pid] = orient
                        sim_positions[pid].orientation = orient
                else:
                    # Blocked - try smaller movement
                    for scale in [0.5, 0.25, 0.1]:
                        test_x = curr_pos.x + move_x * scale
                        test_y = curr_pos.y + move_y * scale
                        if self._is_position_safe(pid, test_x, test_y, sim_positions):
                            sim_positions[pid].x = test_x
                            sim_positions[pid].y = test_y
                            stuck_counter[pid] = 0
                            break
                    else:
                        # Completely blocked
                        stuck_counter[pid] += 1
                        if stuck_counter[pid] == 1 or stuck_counter[pid] % 20 == 0:
                            print(f"    {pid} blocked at ({curr_pos.x:.0f},{curr_pos.y:.0f}), stuck={stuck_counter[pid]}")
                
                # Record position in history
                movement_history[pid].append(
                    (current_time, sim_positions[pid].copy(), sim_orientations[pid])
                )
            
            current_time += time_step
        
        return sim_positions, movement_history, current_time
    
    def _is_position_safe(self, piece_id: str, x: float, y: float,
                         all_positions: Dict[str, Position]) -> bool:
        """Check if a position is safe (no collisions)"""
        # Check board bounds with margins
        if x < BOARD_EXTRA_SIDE - 50 or x > BOARD_EXTRA_SIDE + 8 * BOARD_SQUARE_SIZE + 50:
            return False
        if y < -20 or y > 8 * BOARD_SQUARE_SIZE + 20:
            return False
        
        # Check collisions with other pieces
        for other_id, other_pos in all_positions.items():
            if other_id == piece_id:
                continue
            
            dx = x - other_pos.x
            dy = y - other_pos.y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < PIECE_RADIUS * 2 + 5:  # 5mm safety margin
                return False
        
        return True
    
    def _create_plan_from_history(self, movement_history: Dict[str, List[Tuple[float, Position, float]]]) -> ExecutionPlan:
        """Convert movement history into execution plan"""
        plan = ExecutionPlan()
        
        print(f"\n[DEBUG] Converting movement history to execution plan...")
        print(f"  Total pieces: {len(movement_history)}")
        
        for piece_id, history in movement_history.items():
            if len(history) < 2:
                continue
            
            sequence = PieceCommandSequence(piece_id=piece_id)
            
            # Get start and end positions
            start_time, start_pos, start_orient = history[0]
            end_time, end_pos, end_orient = history[-1]
            
            # Calculate total distance
            total_distance = 0.0
            for i in range(1, len(history)):
                prev_pos = history[i-1][1]
                curr_pos = history[i][1]
                dx = curr_pos.x - prev_pos.x
                dy = curr_pos.y - prev_pos.y
                total_distance += math.sqrt(dx*dx + dy*dy)
            
            if total_distance < 0.5:
                # Minimal movement - skip
                continue
            
            # Sample waypoints from history (every 10 steps to reduce command count)
            sample_rate = max(1, len(history) // 10)
            waypoints = [history[0]]
            for i in range(sample_rate, len(history), sample_rate):
                waypoints.append(history[i])
            if waypoints[-1] != history[-1]:
                waypoints.append(history[-1])
            
            # Create move commands between waypoints
            current_orient = start_orient
            for i in range(1, len(waypoints)):
                prev_time, prev_pos, prev_orient = waypoints[i-1]
                curr_time, curr_pos, curr_orient = waypoints[i]
                
                dx = curr_pos.x - prev_pos.x
                dy = curr_pos.y - prev_pos.y
                segment_dist = math.sqrt(dx*dx + dy*dy)
                
                if segment_dist > 0.5:
                    # Rotate to face movement direction
                    target_orient = math.degrees(math.atan2(dy, dx)) % 360
                    angle_diff = abs((target_orient - current_orient + 180) % 360 - 180)
                    if angle_diff > 5.0:
                        rotate_cmd = self._create_rotate_command(current_orient, target_orient)
                        sequence.add_command(rotate_cmd)
                        current_orient = target_orient
                    
                    # Move forward
                    move_cmd = self._create_move_command(segment_dist)
                    sequence.add_command(move_cmd)
            
            if len(sequence.commands) > 0:
                plan.add_sequence_at_time(sequence, 0.0)
                print(f"  {piece_id}: {len(sequence.commands)} commands, total_dist={total_distance:.1f}mm")
        
        print(f"\n[RESULT] Plan contains {len(plan.sequences)} piece sequences")
        total_commands = sum(len(seq.commands) for seqs in plan.sequences.values() for seq in seqs)
        print(f"[RESULT] Total commands: {total_commands}")
        
        return plan
    
    def _create_rotate_command(self, start_angle: float, target_angle: float) -> PieceCommand:
        """Create a rotation command"""
        angle_diff = (target_angle - start_angle + 180) % 360 - 180
        duration = abs(angle_diff) / ANGULAR_VELOCITY
        
        return PieceCommand(
            command_type=CommandType.ROTATE,
            target_orientation=target_angle,
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
