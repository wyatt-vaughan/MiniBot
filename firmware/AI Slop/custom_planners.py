"""
Example custom path planners for the Chess Robot Coordinator

This file demonstrates how to implement custom path planning algorithms
for the chess robot coordinator system.
"""

from coordinator import PathPlanner, Position, MovePath, Piece, BOARD_SQUARE_SIZE, BOARD_EXTRA_SIDE, PIECE_RADIUS
from typing import Dict, List
import math


class ClusterBasedPlanner(PathPlanner):
    """
    Planner that groups pieces into clusters and moves them together
    to minimize total movement time.
    
    Good for: Organized movement patterns where pieces can move together
    """
    
    def plan_movements(self, pieces: Dict[str, Piece],
                       target_positions: Dict[str, Position]) -> Dict[str, MovePath]:
        """Plan movements using cluster-based approach"""
        paths = {}
        
        # Group pieces by target region (clusters)
        clusters = self._create_clusters(pieces, target_positions)
        
        for cluster_id, piece_ids in clusters.items():
            for piece_id in piece_ids:
                if piece_id not in pieces or piece_id not in target_positions:
                    continue
                
                piece = pieces[piece_id]
                target_pos = target_positions[piece_id]
                
                waypoints = [piece.position.copy()]
                
                # Create path similar to sequential planner
                dx = target_pos.x - piece.position.x
                dy = target_pos.y - piece.position.y
                target_angle = math.degrees(math.atan2(dy, dx))
                
                waypoints.append(Position(piece.position.x, piece.position.y, target_angle))
                waypoints.append(target_pos.copy())
                
                rotation_time = 1.5
                movement_distance = piece.position.distance_to(target_pos)
                movement_time = movement_distance / 100.0
                
                paths[piece_id] = MovePath(
                    piece_id=piece_id,
                    waypoints=waypoints,
                    duration=rotation_time + movement_time
                )
        
        return paths
    
    def _create_clusters(self, pieces: Dict[str, Piece],
                         target_positions: Dict[str, Position]) -> Dict[int, List[str]]:
        """Create clusters of pieces based on target proximity"""
        clusters = {}
        cluster_radius = 200  # mm
        cluster_id = 0
        
        assigned = set()
        
        for piece_id, target_pos in target_positions.items():
            if piece_id in assigned:
                continue
            
            cluster = [piece_id]
            assigned.add(piece_id)
            
            # Find nearby pieces
            for other_id, other_target in target_positions.items():
                if other_id not in assigned:
                    if target_pos.distance_to(other_target) < cluster_radius:
                        cluster.append(other_id)
                        assigned.add(other_id)
            
            clusters[cluster_id] = cluster
            cluster_id += 1
        
        return clusters
    
    def get_name(self) -> str:
        return "Cluster-Based Planner"


class CornerPreferencePlanner(PathPlanner):
    """
    Planner that prioritizes moving pieces already close to their targets.
    
    Good for: Scenarios where pieces need to be moved in a specific order
    """
    
    def plan_movements(self, pieces: Dict[str, Piece],
                       target_positions: Dict[str, Position]) -> Dict[str, MovePath]:
        """Plan movements prioritizing pieces close to targets"""
        paths = {}
        
        # Sort by how close piece is to target (closest first)
        sorted_items = sorted(
            target_positions.items(),
            key=lambda x: pieces[x[0]].position.distance_to(x[1])
        )
        
        occupied = {pid: p.position for pid, p in pieces.items()}
        
        for piece_id, target_pos in sorted_items:
            if piece_id not in pieces:
                continue
            
            piece = pieces[piece_id]
            waypoints = [piece.position.copy()]
            
            # Check for obstacles and create detour if needed
            if self._has_obstacle(piece.position, target_pos, occupied, piece_id):
                # Create detour point
                detour = self._calculate_detour(piece.position, target_pos, occupied)
                waypoints.append(detour)
            
            # Angle to target
            dx = target_pos.x - piece.position.x
            dy = target_pos.y - piece.position.y
            target_angle = math.degrees(math.atan2(dy, dx))
            
            waypoints.append(Position(piece.position.x, piece.position.y, target_angle))
            waypoints.append(target_pos.copy())
            
            # Calculate duration
            total_distance = sum(
                waypoints[i].distance_to(waypoints[i+1])
                for i in range(len(waypoints)-1)
            )
            duration = total_distance / 100.0 + 2.0
            
            paths[piece_id] = MovePath(
                piece_id=piece_id,
                waypoints=waypoints,
                duration=duration
            )
            
            occupied[piece_id] = target_pos
        
        return paths
    
    def _has_obstacle(self, start: Position, end: Position,
                      occupied: Dict[str, Position], exclude_id: str) -> bool:
        """Check if path has obstacles"""
        collision_distance = PIECE_RADIUS * 2 + 5
        
        for piece_id, pos in occupied.items():
            if piece_id == exclude_id:
                continue
            
            distance = self._point_to_segment_distance(pos, start, end)
            if distance < collision_distance:
                return True
        
        return False
    
    def _point_to_segment_distance(self, point: Position,
                                   seg_start: Position, seg_end: Position) -> float:
        """Calculate distance from point to line segment"""
        dx = seg_end.x - seg_start.x
        dy = seg_end.y - seg_start.y
        
        if dx == 0 and dy == 0:
            return point.distance_to(seg_start)
        
        t = max(0, min(1, ((point.x - seg_start.x) * dx + (point.y - seg_start.y) * dy) / (dx*dx + dy*dy)))
        
        closest_x = seg_start.x + t * dx
        closest_y = seg_start.y + t * dy
        closest = Position(closest_x, closest_y)
        
        return point.distance_to(closest)
    
    def _calculate_detour(self, start: Position, end: Position,
                          occupied: Dict[str, Position]) -> Position:
        """Calculate a detour point around obstacles"""
        dx = end.x - start.x
        dy = end.y - start.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance == 0:
            return start.copy()
        
        # Perpendicular offset
        perp_x = -dy / distance * 100
        perp_y = dx / distance * 100
        
        return Position(start.x + perp_x, start.y + perp_y, 0)
    
    def get_name(self) -> str:
        return "Corner Preference Planner"


class MinimizeRotationPlanner(PathPlanner):
    """
    Planner that tries to minimize the amount of rotation needed.
    
    Good for: When rotation is significantly slower than translation
    """
    
    def plan_movements(self, pieces: Dict[str, Piece],
                       target_positions: Dict[str, Position]) -> Dict[str, MovePath]:
        """Plan movements minimizing rotation"""
        paths = {}
        
        for piece_id, target_pos in target_positions.items():
            if piece_id not in pieces:
                continue
            
            piece = pieces[piece_id]
            waypoints = [piece.position.copy()]
            
            # Calculate required rotation
            dx = target_pos.x - piece.position.x
            dy = target_pos.y - piece.position.y
            target_angle = math.degrees(math.atan2(dy, dx))
            
            # Normalize angles
            current_angle = piece.position.orientation % 360
            target_angle = target_angle % 360
            
            # Find shortest rotation direction
            angle_diff = (target_angle - current_angle) % 360
            if angle_diff > 180:
                angle_diff -= 360
            
            # If rotation is significant, consider moving backwards to avoid it
            if abs(angle_diff) > 90:
                # Move in the direction we're already facing
                direction_angle = piece.position.orientation
                move_distance = piece.position.distance_to(target_pos)
                
                # Estimate new position if moving in current direction
                rad = math.radians(direction_angle)
                new_x = piece.position.x + math.cos(rad) * move_distance
                new_y = piece.position.y + math.sin(rad) * move_distance
                
                # Move towards this intermediate point
                intermediate = Position(new_x, new_y, piece.position.orientation)
                waypoints.append(intermediate)
            
            # Final rotation and movement
            waypoints.append(Position(piece.position.x, piece.position.y, target_angle))
            waypoints.append(target_pos.copy())
            
            # Calculate duration
            movement_distance = piece.position.distance_to(target_pos)
            movement_time = movement_distance / 100.0
            rotation_time = abs(angle_diff) / 180.0 * 2.0  # Proportional to rotation amount
            
            paths[piece_id] = MovePath(
                piece_id=piece_id,
                waypoints=waypoints,
                duration=rotation_time + movement_time
            )
        
        return paths
    
    def get_name(self) -> str:
        return "Minimize Rotation Planner"


if __name__ == "__main__":
    # Example: Test custom planners
    print("Custom planners available:")
    print("- ClusterBasedPlanner: Groups nearby pieces for coordinated movement")
    print("- CornerPreferencePlanner: Prioritizes pieces close to targets")
    print("- MinimizeRotationPlanner: Optimizes for minimal rotation")
    print("\nTo use these planners, add them to coordinator.py's available_planners list")
