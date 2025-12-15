"""
Chess Robot Coordinator and Master Controller
Manages piece communication, movement planning, and game rule enforcement
"""

import pygame
import numpy as np
import random
import time
import math
from typing import List, Tuple, Dict

# Import from coordinator package
from coordinator import (
    BOARD_SQUARE_SIZE, BOARD_EXTRA_SIDE, PIECE_RADIUS,
    ANGULAR_VELOCITY, LINEAR_VELOCITY, PIECE_START_POSITIONS,
    WINDOW_WIDTH, WINDOW_HEIGHT, BOARD_DISPLAY_OFFSET_X,
    BOARD_DISPLAY_OFFSET_Y, BOARD_DISPLAY_SQUARE_SIZE,
    CommandType, Position, Piece, PieceCommand,
    PieceCommandSequence, ExecutionPlan, SimulatorState,
    PathPlanner, SequentialPathPlanner, AI_Planner, AStarPlanner, AStarOptimized, VisualSimulationPlanner
)
from coordinator.utils import board_coords_to_world


# ============================================================================
# Simulator Engine and UI
# ============================================================================

class SimulatorEngine:
    """Manages piece simulation and movement execution"""
    
    def __init__(self):
        self.state = SimulatorState(pieces={})
        # Track starting positions for each command
        self.command_start_positions: Dict[str, Position] = {}
        
    def initialize_board(self):
        """Initialize chess pieces at starting positions"""
        self.state.pieces = {}
        for piece_id, (col, row) in PIECE_START_POSITIONS.items():
            # Use board_coords_to_world to center pieces on their squares
            x, y = board_coords_to_world(col, row)
            pos = Position(x, y, 0)
            self.state.pieces[piece_id] = Piece(piece_id, pos, pos.copy())
    
    def randomize_positions(self):
        """Randomize piece positions on the board ensuring no collisions"""
        board_width = 8 * BOARD_SQUARE_SIZE
        board_height = 8 * BOARD_SQUARE_SIZE
        min_distance = PIECE_RADIUS * 2 + 5
        
        positions = {}
        
        for piece_id in self.state.pieces.keys():
            max_attempts = 100
            placed = False
            
            for attempt in range(max_attempts):
                x = BOARD_EXTRA_SIDE + random.uniform(0, board_width)
                y = random.uniform(0, board_height)
                angle = random.uniform(0, 360)
                pos = Position(x, y, angle)
                
                # Check collision with all previously placed pieces
                collision = False
                for other_pos in positions.values():
                    if pos.distance_to(other_pos) < min_distance:
                        collision = True
                        break
                
                if not collision:
                    positions[piece_id] = pos
                    placed = True
                    break
            
            if not placed:
                # If we can't place this piece, replace a random existing piece
                if positions:
                    replaced_id = random.choice(list(positions.keys()))
                    del positions[replaced_id]
                    # Try again for this piece
                    for attempt in range(max_attempts):
                        x = BOARD_EXTRA_SIDE + random.uniform(0, board_width)
                        y = random.uniform(0, board_height)
                        angle = random.uniform(0, 360)
                        pos = Position(x, y, angle)
                        
                        collision = False
                        for other_pos in positions.values():
                            if pos.distance_to(other_pos) < min_distance:
                                collision = True
                                break
                        
                        if not collision:
                            positions[piece_id] = pos
                            break
        
        # Apply positions
        for piece_id, pos in positions.items():
            self.state.pieces[piece_id].position = pos
    
    def start_execution(self, plan: ExecutionPlan):
        """Start executing the execution plan"""
        self.state.execution_plan = plan
        self.state.executing = True
        self.state.execution_start_time = time.time()
        
        # Store start positions for visualization
        for piece_id, piece in self.state.pieces.items():
            piece.start_position = piece.position.copy()
        
        # Initialize execution tracking for each piece
        self.state.executing_pieces = {}
        self.command_start_positions = {}  # Clear old command tracking
        if plan and plan.sequences:
            for piece_id in plan.sequences.keys():
                self.state.executing_pieces[piece_id] = (0, 0)  # (sequence_index, command_index)
    
    def stop_execution(self):
        """Stop executing movements"""
        self.state.executing = False
        self.state.execution_plan = None
        self.state.executing_pieces = {}
    
    def update(self, dt: float) -> bool:
        """Update simulator state. Returns True if still executing"""
        if not self.state.executing or not self.state.execution_plan:
            return False
        
        plan = self.state.execution_plan
        elapsed = time.time() - self.state.execution_start_time
        
        any_executing = False
        
        # Update each piece
        for piece_id, sequences in plan.sequences.items():
            if piece_id not in self.state.executing_pieces:
                continue
            
            piece = self.state.pieces[piece_id]
            seq_idx, cmd_idx = self.state.executing_pieces[piece_id]
            
            # Check if this piece is done
            if seq_idx >= len(sequences):
                continue
            
            sequence = sequences[seq_idx]
            
            # Check if this sequence should have started yet
            if elapsed < sequence.start_time:
                any_executing = True
                continue
            
            # Check if done with all commands in this sequence
            if cmd_idx >= len(sequence.commands):
                # Move to next sequence
                seq_idx += 1
                cmd_idx = 0
                self.state.executing_pieces[piece_id] = (seq_idx, cmd_idx)
                
                if seq_idx < len(sequences):
                    any_executing = True
                continue
            
            command = sequence.commands[cmd_idx]
            time_in_sequence = elapsed - sequence.start_time
            command_start = command.start_time
            command_end = command.start_time + command.duration
            
            # Check if command should have started
            if time_in_sequence < command_start:
                any_executing = True
                continue
            
            # Check if command is complete
            if time_in_sequence >= command_end:
                # Apply final state with exact progress=1.0 to ensure we hit target precisely
                self._apply_command_progress(piece, command, 1.0, seq_idx, cmd_idx)
                
                # Clear the cached start position for this completed command
                cmd_key = f"{piece_id}_seq{seq_idx}_cmd{cmd_idx}"
                if cmd_key in self.command_start_positions:
                    del self.command_start_positions[cmd_key]
                
                # Move to next command
                cmd_idx += 1
                self.state.executing_pieces[piece_id] = (seq_idx, cmd_idx)
                any_executing = True
                continue
            
            # Command is in progress - interpolate
            progress = (time_in_sequence - command_start) / max(command.duration, 0.001)
            progress = min(1.0, max(0.0, progress))
            
            self._apply_command_progress(piece, command, progress, seq_idx, cmd_idx)
            any_executing = True
        
        if not any_executing:
            self.state.executing = False
            self.state.execution_plan = None  # Clear the plan to remove drawn paths
        
        return self.state.executing
    
    def _apply_command_final_state(self, piece: Piece, command: PieceCommand):
        """Apply the final state of a completed command to ensure exact positioning."""
        if command.command_type == CommandType.ROTATE:
            piece.position.orientation = command.target_orientation
        elif command.command_type == CommandType.MOVE_STRAIGHT:
            # Move to exact final position based on start position and command
            cmd_key = f"{piece.id}_seq{self.state.executing_pieces[piece.id][0]}_cmd{self.state.executing_pieces[piece.id][1]}"
            if cmd_key in self.command_start_positions:
                start_pos = self.command_start_positions[cmd_key]
                angle_rad = math.radians(start_pos.orientation)
                piece.position.x = start_pos.x + command.distance * math.cos(angle_rad)
                piece.position.y = start_pos.y + command.distance * math.sin(angle_rad)
    
    def _apply_command_progress(self, piece: Piece, command: PieceCommand, progress: float, seq_idx: int, cmd_idx: int):
        """Apply partial progress of a command"""
        # Get or store the start position for this command
        # Use sequence and command index to uniquely identify each command execution
        cmd_key = f"{piece.id}_seq{seq_idx}_cmd{cmd_idx}"
        
        if cmd_key not in self.command_start_positions:
            self.command_start_positions[cmd_key] = piece.position.copy()
        
        start_pos = self.command_start_positions[cmd_key]
        
        if command.command_type == CommandType.ROTATE:
            # Interpolate rotation
            start_orientation = start_pos.orientation
            target_orientation = command.target_orientation
            
            # Calculate shortest rotation path
            angle_diff = (target_orientation - start_orientation) % 360
            if angle_diff > 180:
                angle_diff -= 360
            
            piece.position.orientation = (start_orientation + angle_diff * progress) % 360
            
        elif command.command_type == CommandType.MOVE_STRAIGHT:
            # Move in the direction from the START of this command (not current orientation)
            # This prevents drift when rotation and movement commands overlap
            angle_rad = math.radians(start_pos.orientation)
            distance_traveled = command.distance * progress
            
            piece.position.x = start_pos.x + distance_traveled * math.cos(angle_rad)
            piece.position.y = start_pos.y + distance_traveled * math.sin(angle_rad)
        
        elif command.command_type == CommandType.MOVE_ARC:
            # Move along an arc from start to target position
            # Arc is tangent to start orientation and curves based on radius
            target = command.target_position
            radius = command.arc_radius
            
            # Calculate straight-line distance
            dx = target.x - start_pos.x
            dy = target.y - start_pos.y
            chord_length = math.sqrt(dx*dx + dy*dy)
            
            # Calculate arc parameters
            abs_radius = abs(radius)
            if chord_length > 2 * abs_radius:
                # Fallback to straight line
                piece.position.x = start_pos.x + dx * progress
                piece.position.y = start_pos.y + dy * progress
            else:
                # Calculate center of arc circle
                # The center is perpendicular to the start orientation
                sin_half_theta = chord_length / (2 * abs_radius)
                sin_half_theta = min(1.0, max(-1.0, sin_half_theta))
                half_theta = math.asin(sin_half_theta)
                theta = 2 * half_theta  # Total arc angle in radians
                
                # Perpendicular direction (90° from start orientation)
                # Positive radius = CW = perpendicular to the right
                # Negative radius = CCW = perpendicular to the left
                perp_angle = math.radians(start_pos.orientation) + (math.pi/2 if radius > 0 else -math.pi/2)
                
                # Distance from start to center along perpendicular
                # Using geometry: for arc tangent to start orientation,
                # center is at distance |radius| perpendicular to start direction
                center_x = start_pos.x + abs_radius * math.cos(perp_angle)
                center_y = start_pos.y + abs_radius * math.sin(perp_angle)
                
                # Angle from center to start position
                start_angle = math.atan2(start_pos.y - center_y, start_pos.x - center_x)
                
                # Current angle along arc
                current_angle = start_angle + (theta * progress if radius > 0 else -theta * progress)
                
                # Calculate current position on arc
                piece.position.x = center_x + abs_radius * math.cos(current_angle)
                piece.position.y = center_y + abs_radius * math.sin(current_angle)
                
                # Update orientation to be tangent to arc
                # Tangent is perpendicular to radius
                tangent_angle = current_angle + (math.pi/2 if radius > 0 else -math.pi/2)
                piece.position.orientation = math.degrees(tangent_angle) % 360
        
        # WAIT does nothing
    
    def check_collisions(self) -> List[Tuple[str, str]]:
        """Check for piece collisions. Returns list of colliding pairs"""
        collisions = []
        piece_ids = list(self.state.pieces.keys())
        
        for i, pid1 in enumerate(piece_ids):
            for pid2 in piece_ids[i+1:]:
                piece1 = self.state.pieces[pid1]
                piece2 = self.state.pieces[pid2]
                
                distance = piece1.position.distance_to(piece2.position)
                if distance < PIECE_RADIUS * 2:
                    collisions.append((pid1, pid2))
        
        return collisions
    
    def get_total_move_time(self) -> float:
        """Get estimated total move time for current execution plan"""
        if not self.state.execution_plan:
            return 0.0
        return self.state.execution_plan.get_total_duration()

# ============================================================================
# UI and Rendering
# ============================================================================

class ChessRobotUI:
    """Pygame UI for the chess robot coordinator"""
    
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Chess Robot Coordinator - Simulator")
        self.clock = pygame.time.Clock()
        self.font_small = pygame.font.Font(None, 24)
        self.font_medium = pygame.font.Font(None, 32)
        self.font_large = pygame.font.Font(None, 48)
        
        self.simulator = SimulatorEngine()
        self.path_planner: PathPlanner = SequentialPathPlanner()
        self.running = True
        
        self.simulator.initialize_board()
        
        # UI state
        self.show_paths = True
        self.planner_index = 0
        self.available_planners = [
            SequentialPathPlanner(),
            AI_Planner(),
            AStarPlanner(grid_resolution=10),
            AStarOptimized(grid_resolution=10),
            VisualSimulationPlanner(time_step=0.2),
        ]
        self.buttons = []  # Will be populated by draw_ui_buttons
        
    def get_display_position(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates (mm) to display coordinates (pixels)"""
        # Account for board offset in world space (pieces placed with offset)
        # x is absolute position in mm including BOARD_EXTRA_SIDE offset
        # We need to convert to display space
        board_x = x - BOARD_EXTRA_SIDE  # Remove world offset to get board-relative position
        display_x = BOARD_DISPLAY_OFFSET_X + (board_x / BOARD_SQUARE_SIZE) * BOARD_DISPLAY_SQUARE_SIZE
        display_y = BOARD_DISPLAY_OFFSET_Y + (y / BOARD_SQUARE_SIZE) * BOARD_DISPLAY_SQUARE_SIZE
        return int(display_x), int(display_y)
    
    def draw_board(self):
        """Draw the chess board"""
        # Draw navigation area (margins) first
        nav_area_color = (220, 220, 240)  # Light blue-gray
        nav_area_border = (150, 150, 180)  # Darker border
        
        # Calculate navigation area in display coordinates
        # Left margin: 100mm
        left_margin_display_x = BOARD_DISPLAY_OFFSET_X - int((100 / BOARD_SQUARE_SIZE) * BOARD_DISPLAY_SQUARE_SIZE)
        left_margin_width = int((100 / BOARD_SQUARE_SIZE) * BOARD_DISPLAY_SQUARE_SIZE)
        
        # Right margin: 100mm
        right_margin_display_x = BOARD_DISPLAY_OFFSET_X + 8 * BOARD_DISPLAY_SQUARE_SIZE
        right_margin_width = int((100 / BOARD_SQUARE_SIZE) * BOARD_DISPLAY_SQUARE_SIZE)
        
        # Top margin: 20mm
        top_margin_display_y = BOARD_DISPLAY_OFFSET_Y - int((20 / BOARD_SQUARE_SIZE) * BOARD_DISPLAY_SQUARE_SIZE)
        top_margin_height = int((20 / BOARD_SQUARE_SIZE) * BOARD_DISPLAY_SQUARE_SIZE)
        
        # Bottom margin: 20mm
        bottom_margin_display_y = BOARD_DISPLAY_OFFSET_Y + 8 * BOARD_DISPLAY_SQUARE_SIZE
        bottom_margin_height = int((20 / BOARD_SQUARE_SIZE) * BOARD_DISPLAY_SQUARE_SIZE)
        
        board_height = 8 * BOARD_DISPLAY_SQUARE_SIZE
        
        # Draw left margin
        pygame.draw.rect(self.screen, nav_area_color, 
                        (left_margin_display_x, BOARD_DISPLAY_OFFSET_Y, left_margin_width, board_height))
        pygame.draw.rect(self.screen, nav_area_border,
                        (left_margin_display_x, BOARD_DISPLAY_OFFSET_Y, left_margin_width, board_height), 2)
        
        # Draw right margin
        pygame.draw.rect(self.screen, nav_area_color,
                        (right_margin_display_x, BOARD_DISPLAY_OFFSET_Y, right_margin_width, board_height))
        pygame.draw.rect(self.screen, nav_area_border,
                        (right_margin_display_x, BOARD_DISPLAY_OFFSET_Y, right_margin_width, board_height), 2)
        
        # Draw top margin (full width including side margins)
        full_width = left_margin_width + 8 * BOARD_DISPLAY_SQUARE_SIZE + right_margin_width
        pygame.draw.rect(self.screen, nav_area_color,
                        (left_margin_display_x, top_margin_display_y, full_width, top_margin_height))
        pygame.draw.rect(self.screen, nav_area_border,
                        (left_margin_display_x, top_margin_display_y, full_width, top_margin_height), 2)
        
        # Draw bottom margin (full width including side margins)
        pygame.draw.rect(self.screen, nav_area_color,
                        (left_margin_display_x, bottom_margin_display_y, full_width, bottom_margin_height))
        pygame.draw.rect(self.screen, nav_area_border,
                        (left_margin_display_x, bottom_margin_display_y, full_width, bottom_margin_height), 2)
        
        # Draw board squares
        for row in range(8):
            for col in range(8):
                x = BOARD_DISPLAY_OFFSET_X + col * BOARD_DISPLAY_SQUARE_SIZE
                y = BOARD_DISPLAY_OFFSET_Y + row * BOARD_DISPLAY_SQUARE_SIZE
                
                color = (240, 217, 181) if (row + col) % 2 == 0 else (181, 136, 99)
                pygame.draw.rect(self.screen, color, (x, y, BOARD_DISPLAY_SQUARE_SIZE, BOARD_DISPLAY_SQUARE_SIZE))
                pygame.draw.rect(self.screen, (0, 0, 0), (x, y, BOARD_DISPLAY_SQUARE_SIZE, BOARD_DISPLAY_SQUARE_SIZE), 1)
        
        # Draw coordinate labels
        for i in range(8):
            label = chr(ord('a') + i)
            x = BOARD_DISPLAY_OFFSET_X + i * BOARD_DISPLAY_SQUARE_SIZE + BOARD_DISPLAY_SQUARE_SIZE // 2
            y = BOARD_DISPLAY_OFFSET_Y + 8 * BOARD_DISPLAY_SQUARE_SIZE + 5
            text = self.font_small.render(label, True, (0, 0, 0))
            self.screen.blit(text, (x - text.get_width() // 2, y))
            
            label = str(8 - i)
            x = BOARD_DISPLAY_OFFSET_X - 25
            y = BOARD_DISPLAY_OFFSET_Y + i * BOARD_DISPLAY_SQUARE_SIZE + BOARD_DISPLAY_SQUARE_SIZE // 2
            text = self.font_small.render(label, True, (0, 0, 0))
            self.screen.blit(text, (x, y - text.get_height() // 2))
    
    def draw_pieces(self):
        """Draw all pieces"""
        for piece in self.simulator.state.pieces.values():
            display_x, display_y = self.get_display_position(piece.position.x, piece.position.y)
            
            # Draw piece circle
            color = (100, 200, 255) if piece.id.isupper() else (255, 150, 100)
            pygame.draw.circle(self.screen, color, (display_x, display_y), 15, 0)
            pygame.draw.circle(self.screen, (0, 0, 0), (display_x, display_y), 15, 2)
            
            # Draw orientation indicator (line from center)
            angle_rad = math.radians(piece.position.orientation)
            end_x = display_x + 12 * math.cos(angle_rad)
            end_y = display_y + 12 * math.sin(angle_rad)
            pygame.draw.line(self.screen, (0, 0, 0), (display_x, display_y), (end_x, end_y), 2)
            
            # Draw piece label
            label_text = self.font_small.render(piece.id, True, (0, 0, 0))
            self.screen.blit(label_text, (display_x - label_text.get_width() // 2, display_y - label_text.get_height() // 2))
    
    def draw_paths(self):
        """Draw planned movement paths as dotted lines"""
        if not self.show_paths or not self.simulator.state.execution_plan:
            return
        
        plan = self.simulator.state.execution_plan
        for piece_id, sequences in plan.sequences.items():
            if piece_id not in self.simulator.state.pieces:
                continue
            
            piece = self.simulator.state.pieces[piece_id]
            # Start from current position when planning, or start_position during execution
            if self.simulator.state.executing and piece.start_position:
                current_pos = piece.start_position.copy()
            else:
                current_pos = piece.position.copy()
            
            for sequence in sequences:
                for command in sequence.commands:
                    if command.command_type == CommandType.ROTATE:
                        # Draw rotation arc indicator
                        display_pos = self.get_display_position(current_pos.x, current_pos.y)
                        pygame.draw.circle(self.screen, (200, 150, 100), display_pos, 10, 1)
                        current_pos.orientation = command.target_orientation
                        
                    elif command.command_type == CommandType.MOVE_STRAIGHT:
                        # Draw movement line
                        start_display = self.get_display_position(current_pos.x, current_pos.y)
                        
                        angle_rad = math.radians(current_pos.orientation)
                        end_x = current_pos.x + command.distance * math.cos(angle_rad)
                        end_y = current_pos.y + command.distance * math.sin(angle_rad)
                        end_display = self.get_display_position(end_x, end_y)
                        
                        self._draw_dotted_line(start_display, end_display, (150, 150, 150), 3, 5)
                        
                        current_pos.x = end_x
                        current_pos.y = end_y
                    
                    elif command.command_type == CommandType.MOVE_ARC:
                        # Draw arc path
                        target = command.target_position
                        radius = command.arc_radius
                        
                        # Calculate arc parameters
                        dx = target.x - current_pos.x
                        dy = target.y - current_pos.y
                        chord_length = math.sqrt(dx*dx + dy*dy)
                        abs_radius = abs(radius)
                        
                        if chord_length <= 2 * abs_radius:
                            # Calculate center of arc
                            sin_half_theta = chord_length / (2 * abs_radius)
                            sin_half_theta = min(1.0, max(-1.0, sin_half_theta))
                            half_theta = math.asin(sin_half_theta)
                            theta = 2 * half_theta
                            
                            perp_angle = math.radians(current_pos.orientation) + (math.pi/2 if radius > 0 else -math.pi/2)
                            center_x = current_pos.x + abs_radius * math.cos(perp_angle)
                            center_y = current_pos.y + abs_radius * math.sin(perp_angle)
                            
                            # Draw arc as series of small line segments
                            num_segments = 20
                            start_angle = math.atan2(current_pos.y - center_y, current_pos.x - center_x)
                            
                            prev_display = self.get_display_position(current_pos.x, current_pos.y)
                            for i in range(1, num_segments + 1):
                                t = i / num_segments
                                current_angle = start_angle + (theta * t if radius > 0 else -theta * t)
                                arc_x = center_x + abs_radius * math.cos(current_angle)
                                arc_y = center_y + abs_radius * math.sin(current_angle)
                                arc_display = self.get_display_position(arc_x, arc_y)
                                self._draw_dotted_line(prev_display, arc_display, (180, 150, 180), 3, 5)
                                prev_display = arc_display
                        else:
                            # Fallback to straight line
                            start_display = self.get_display_position(current_pos.x, current_pos.y)
                            end_display = self.get_display_position(target.x, target.y)
                            self._draw_dotted_line(start_display, end_display, (150, 150, 150), 3, 5)
                        
                        # Update position and orientation
                        current_pos.x = target.x
                        current_pos.y = target.y
                        # Calculate final orientation (tangent to arc at end)
                        if chord_length <= 2 * abs_radius:
                            end_angle = math.atan2(target.y - center_y, target.x - center_x)
                            tangent_angle = end_angle + (math.pi/2 if radius > 0 else -math.pi/2)
                            current_pos.orientation = math.degrees(tangent_angle) % 360
            
            # Draw final target marker
            display_x, display_y = self.get_display_position(current_pos.x, current_pos.y)
            pygame.draw.circle(self.screen, (100, 255, 100), (display_x, display_y), 8, 2)
    
    def _draw_dotted_line(self, start: Tuple[int, int], end: Tuple[int, int],
                          color: Tuple[int, int, int], dot_size: int, gap: int):
        """Draw a dotted line"""
        x1, y1 = start
        x2, y2 = end
        
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if distance == 0:
            return
        
        steps = int(distance / (dot_size + gap))
        for i in range(steps):
            t = i / max(steps, 1)
            x = int(x1 + (x2 - x1) * t)
            y = int(y1 + (y2 - y1) * t)
            pygame.draw.circle(self.screen, color, (x, y), dot_size // 2)
    
    def draw_ui_buttons(self):
        """Draw UI control buttons"""
        button_x = BOARD_DISPLAY_OFFSET_X + 8 * BOARD_DISPLAY_SQUARE_SIZE + BOARD_EXTRA_SIDE + 100
        self.buttons = [
            {"label": "Randomize", "action": "randomize", "x": button_x, "y": 100},
            {"label": "Plan Moves", "action": "plan", "x": button_x, "y": 160},
            {"label": "Execute", "action": "execute", "x": button_x, "y": 220},
        ]
        
        for btn in self.buttons:
            rect = pygame.Rect(btn["x"], btn["y"], 150, 40)
            pygame.draw.rect(self.screen, (200, 200, 200), rect)
            pygame.draw.rect(self.screen, (0, 0, 0), rect, 2)
            
            text = self.font_small.render(btn["label"], True, (0, 0, 0))
            self.screen.blit(text, (rect.centerx - text.get_width() // 2, rect.centery - text.get_height() // 2))
            
            btn["rect"] = rect
    
    def draw_status(self):
        """Draw status information"""
        words_x = BOARD_DISPLAY_OFFSET_X + 8 * BOARD_DISPLAY_SQUARE_SIZE + BOARD_EXTRA_SIDE + 100
        y_offset = 100
        
        # Planner name
        planner_name = self.path_planner.get_name()
        text = self.font_small.render(f"Planner: {planner_name}", True, (0, 0, 0))
        self.screen.blit(text, (words_x, y_offset + 300))
        
        # Execution status
        status = "Executing..." if self.simulator.state.executing else "Idle"
        status_color = (255, 0, 0) if self.simulator.state.executing else (0, 100, 0)
        text = self.font_small.render(f"Status: {status}", True, status_color)
        self.screen.blit(text, (words_x, y_offset + 330))
        
        # Total move time
        total_time = self.simulator.get_total_move_time()
        text = self.font_small.render(f"Move Time: {total_time:.1f}s", True, (0, 0, 0))
        self.screen.blit(text, (words_x, y_offset + 360))
        
        # Collision status
        collisions = self.simulator.check_collisions()
        if collisions:
            text = self.font_small.render(f"COLLISIONS: {len(collisions)}", True, (255, 0, 0))
        else:
            text = self.font_small.render("No collisions", True, (0, 100, 0))
        self.screen.blit(text, (words_x, y_offset + 390))
    
    def handle_input(self):
        """Handle user input"""
        mouse_pos = pygame.mouse.get_pos()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self.handle_button_click(mouse_pos)
            elif event.type == pygame.KEYDOWN:
                self.handle_key_press(event.key)
    
    def handle_button_click(self, mouse_pos: Tuple[int, int]):
        """Handle button clicks"""
        for btn in self.buttons:
            if btn["rect"].collidepoint(mouse_pos):
                if btn["action"] == "randomize":
                    self.simulator.randomize_positions()
                    print("Positions randomized")
                elif btn["action"] == "plan":
                    self.plan_movements()
                    print("Movements planned")
                elif btn["action"] == "execute":
                    if self.simulator.state.execution_plan:
                        self.simulator.start_execution(self.simulator.state.execution_plan)
                        print("Execution started")
    
    def handle_key_press(self, key: int):
        """Handle keyboard input"""
        if key == pygame.K_SPACE:
            # Toggle between planners
            self.planner_index = (self.planner_index + 1) % len(self.available_planners)
            self.path_planner = self.available_planners[self.planner_index]
            print(f"Switched to {self.path_planner.get_name()}")
        elif key == pygame.K_p:
            # Toggle path display
            self.show_paths = not self.show_paths
            print(f"Path display: {'ON' if self.show_paths else 'OFF'}")
        elif key == pygame.K_r:
            # Reset
            self.simulator.stop_execution()
            self.simulator.initialize_board()
            print("Board reset")
    
    def plan_movements(self):
        """Plan movements to start positions"""
        target_positions = {}
        for piece_id, (col, row) in PIECE_START_POSITIONS.items():
            # Use board_coords_to_world to center pieces on their target squares
            x, y = board_coords_to_world(col, row)
            target_positions[piece_id] = Position(x, y, 0)
        
        try:
            plan = self.path_planner.plan_movements(self.simulator.state.pieces, target_positions)
            self.simulator.state.execution_plan = plan
            print(f"\n✓ Plan created successfully with {sum(len(seqs) for seqs in plan.sequences.values())} sequences")
        except Exception as e:
            print(f"\n✗ Planning failed with error: {e}")
            # Create empty plan so we don't crash
            plan = ExecutionPlan()
            self.simulator.state.execution_plan = plan
            import traceback
            traceback.print_exc()
    
    def run(self):
        """Main UI loop"""
        print("\n=== Chess Robot Coordinator ===")
        print("Controls:")
        print("  - Click 'Randomize' to randomize piece positions")
        print("  - Click 'Plan Moves' to plan movements to starting positions")
        print("  - Click 'Execute' to execute the planned movements")
        print("  - Press SPACE to switch between path planners")
        print("  - Press P to toggle path display")
        print("  - Press R to reset the board")
        print()
        
        while self.running:
            dt = self.clock.tick(60) / 1000.0
            
            self.handle_input()
            self.simulator.update(dt)
            
            # Draw everything
            self.screen.fill((255, 255, 255))
            self.draw_board()
            self.draw_paths()
            self.draw_pieces()
            self.draw_ui_buttons()
            self.draw_status()
            
            pygame.display.flip()
        
        pygame.quit()

# ============================================================================
# Testing and Benchmarking
# ============================================================================

class PathPlannerBenchmark:
    """Benchmark path planning algorithms"""
    
    def __init__(self, iterations: int = 10):
        self.iterations = iterations
        self.results = {}
    
    def benchmark_planner(self, planner: PathPlanner) -> Dict:
        """Benchmark a path planner over multiple randomizations"""
        simulator = SimulatorEngine()
        simulator.initialize_board()
        
        total_move_time = 0.0
        total_execution_time = 0.0
        collision_count = 0
        accuracy_errors = []
        
        print(f"\nBenchmarking {planner.get_name()}...")
        print(f"Running {self.iterations} iterations...")
        
        for iteration in range(self.iterations):
            # Randomize positions
            simulator.randomize_positions()
            
            # Plan movements
            target_positions = {}
            for piece_id, (col, row) in PIECE_START_POSITIONS.items():
                x = BOARD_EXTRA_SIDE + col * BOARD_SQUARE_SIZE
                y = row * BOARD_SQUARE_SIZE
                target_positions[piece_id] = Position(x, y, 0)
            
            paths = planner.plan_movements(simulator.state.pieces, target_positions)
            simulator.state.paths = paths
            
            # Execute and measure
            move_time = simulator.get_total_move_time()
            total_move_time += move_time
            
            # Simulate execution to check for collisions
            simulator.start_execution(paths)
            sim_start_time = time.time()
            
            while simulator.state.executing:
                simulator.update(0.016)  # ~60 FPS
            
            execution_time = time.time() - sim_start_time
            total_execution_time += execution_time
            
            # Check final positions
            collisions = simulator.check_collisions()
            collision_count += len(collisions)
            
            # Check accuracy
            for piece_id, piece in simulator.state.pieces.items():
                if piece_id in target_positions:
                    error = piece.position.distance_to(target_positions[piece_id])
                    accuracy_errors.append(error)
            
            if (iteration + 1) % max(1, self.iterations // 5) == 0:
                print(f"  {iteration + 1}/{self.iterations} completed")
        
        # Calculate statistics
        avg_move_time = total_move_time / self.iterations
        avg_execution_time = total_execution_time / self.iterations
        avg_accuracy = sum(accuracy_errors) / len(accuracy_errors) if accuracy_errors else 0
        collision_rate = collision_count / self.iterations
        
        results = {
            "planner": planner.get_name(),
            "iterations": self.iterations,
            "avg_move_time": avg_move_time,
            "avg_execution_time": avg_execution_time,
            "total_move_time": total_move_time,
            "collisions": collision_count,
            "collision_rate": collision_rate,
            "avg_accuracy_error": avg_accuracy,
        }
        
        return results
    
    def print_results(self, results: Dict):
        """Print benchmark results"""
        print("\n" + "=" * 60)
        print(f"Results for {results['planner']}")
        print("=" * 60)
        print(f"Iterations:           {results['iterations']}")
        print(f"Avg Move Time:        {results['avg_move_time']:.2f}s")
        print(f"Total Move Time:      {results['total_move_time']:.2f}s")
        print(f"Avg Execution Time:   {results['avg_execution_time']:.2f}s")
        print(f"Total Collisions:     {results['collisions']}")
        print(f"Collision Rate:       {results['collision_rate']:.2f}")
        print(f"Avg Accuracy Error:   {results['avg_accuracy_error']:.2f}mm")
        print("=" * 60)

# ============================================================================
# Main
# ============================================================================

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "benchmark":
        # Run benchmark mode
        iterations = int(sys.argv[2]) if len(sys.argv) > 2 else 10
        benchmark = PathPlannerBenchmark(iterations=iterations)
        
        planners = [
            SequentialPathPlanner(),
        ]
        
        for planner in planners:
            results = benchmark.benchmark_planner(planner)
            benchmark.print_results(results)
    else:
        # Run UI mode
        ui = ChessRobotUI()
        ui.run()
