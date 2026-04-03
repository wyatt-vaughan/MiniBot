"""
Quick Start Guide for Chess Robot Coordinator

This script provides examples of how to use the coordinator system
"""

import sys
import os

# Add the current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from coordinator import (
    ChessRobotUI, SimulatorEngine, SequentialPathPlanner,
    OptimizedPathPlanner, PathPlannerBenchmark, Position,
    PIECE_START_POSITIONS, BOARD_SQUARE_SIZE, BOARD_EXTRA_SIDE
)


def example_basic_usage():
    """
    Example 1: Basic simulator usage without UI
    """
    print("\n" + "="*60)
    print("Example 1: Basic Simulator Usage")
    print("="*60)
    
    # Create simulator
    sim = SimulatorEngine()
    sim.initialize_board()
    
    print(f"Initialized chess board with {len(sim.state.pieces)} pieces")
    
    # Check initial positions
    print("\nInitial piece positions:")
    for piece_id, piece in list(sim.state.pieces.items())[:4]:  # Show first 4
        print(f"  {piece_id}: ({piece.position.x:.1f}, {piece.position.y:.1f})")
    
    # Randomize
    sim.randomize_positions()
    print("\nAfter randomization:")
    for piece_id, piece in list(sim.state.pieces.items())[:4]:
        print(f"  {piece_id}: ({piece.position.x:.1f}, {piece.position.y:.1f})")


def example_path_planning():
    """
    Example 2: Plan movements programmatically
    """
    print("\n" + "="*60)
    print("Example 2: Path Planning")
    print("="*60)
    
    # Create simulator and planner
    sim = SimulatorEngine()
    sim.initialize_board()
    sim.randomize_positions()
    
    planner = SequentialPathPlanner()
    
    # Create target positions (starting positions)
    target_positions = {}
    for piece_id, (col, row) in PIECE_START_POSITIONS.items():
        x = BOARD_EXTRA_SIDE + col * BOARD_SQUARE_SIZE
        y = row * BOARD_SQUARE_SIZE
        target_positions[piece_id] = Position(x, y, 0)
    
    # Plan movements
    paths = planner.plan_movements(sim.state.pieces, target_positions)
    
    print(f"Planned movements for {len(paths)} pieces")
    print(f"Using: {planner.get_name()}")
    
    # Show planned paths
    print("\nPath durations:")
    for piece_id, path in list(paths.items())[:4]:  # Show first 4
        print(f"  {piece_id}: {path.duration:.2f}s (waypoints: {len(path.waypoints)})")
    
    total_time = sim.get_total_move_time()
    print(f"\nTotal estimated move time: {total_time:.2f}s")


def example_collision_detection():
    """
    Example 3: Detect collisions after execution
    """
    print("\n" + "="*60)
    print("Example 3: Collision Detection")
    print("="*60)
    
    sim = SimulatorEngine()
    sim.initialize_board()
    sim.randomize_positions()
    
    # Check initial collisions
    collisions = sim.check_collisions()
    print(f"Initial collisions: {len(collisions)}")
    
    if collisions:
        print("Colliding pairs:")
        for piece1, piece2 in collisions[:3]:  # Show first 3
            dist = sim.state.pieces[piece1].distance_to(sim.state.pieces[piece2])
            print(f"  {piece1} <-> {piece2}: {dist:.1f}mm apart")


def example_compare_planners():
    """
    Example 4: Quick comparison of different planners
    """
    print("\n" + "="*60)
    print("Example 4: Compare Planners")
    print("="*60)
    
    sim = SimulatorEngine()
    sim.initialize_board()
    sim.randomize_positions()
    
    # Create target positions
    target_positions = {}
    for piece_id, (col, row) in PIECE_START_POSITIONS.items():
        x = BOARD_EXTRA_SIDE + col * BOARD_SQUARE_SIZE
        y = row * BOARD_SQUARE_SIZE
        target_positions[piece_id] = Position(x, y, 0)
    
    planners = [
        SequentialPathPlanner(),
        OptimizedPathPlanner(),
    ]
    
    print("Planning with different algorithms:")
    for planner in planners:
        paths = planner.plan_movements(sim.state.pieces, target_positions)
        total_time = sum(p.duration for p in paths.values())
        waypoint_count = sum(len(p.waypoints) for p in paths.values())
        
        print(f"\n{planner.get_name()}:")
        print(f"  Total move time: {total_time:.2f}s")
        print(f"  Total waypoints: {waypoint_count}")
        print(f"  Avg waypoints per piece: {waypoint_count / len(paths):.1f}")


def example_run_benchmark():
    """
    Example 5: Run a quick benchmark
    """
    print("\n" + "="*60)
    print("Example 5: Quick Benchmark")
    print("="*60)
    
    benchmark = PathPlannerBenchmark(iterations=5)
    planner = SequentialPathPlanner()
    
    results = benchmark.benchmark_planner(planner)
    benchmark.print_results(results)


def main():
    """Run all examples"""
    print("\nChess Robot Coordinator - Quick Start Examples")
    print("=" * 60)
    
    try:
        example_basic_usage()
    except Exception as e:
        print(f"Error in example 1: {e}")
    
    try:
        example_path_planning()
    except Exception as e:
        print(f"Error in example 2: {e}")
    
    try:
        example_collision_detection()
    except Exception as e:
        print(f"Error in example 3: {e}")
    
    try:
        example_compare_planners()
    except Exception as e:
        print(f"Error in example 4: {e}")
    
    try:
        example_run_benchmark()
    except Exception as e:
        print(f"Error in example 5: {e}")
    
    print("\n" + "="*60)
    print("Examples complete!")
    print("\nTo run the interactive simulator:")
    print("  python coordinator.py")
    print("\nTo run benchmarks:")
    print("  python coordinator.py benchmark 20")
    print("="*60 + "\n")


if __name__ == "__main__":
    main()
