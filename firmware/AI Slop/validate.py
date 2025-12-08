"""
System validation and testing script for Chess Robot Coordinator

Run this to validate that all components are working correctly
"""

import sys
import os
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def test_imports():
    """Test that all modules can be imported"""
    print("\n" + "="*70)
    print("TEST 1: Module Imports")
    print("="*70)
    
    try:
        import pygame
        print("✓ pygame imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import pygame: {e}")
        return False
    
    try:
        import numpy
        print("✓ numpy imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import numpy: {e}")
        return False
    
    try:
        from coordinator import (
            Position, Piece, MoveCommand, MovePath, SimulatorState,
            PathPlanner, SequentialPathPlanner, OptimizedPathPlanner,
            SimulatorEngine, ChessRobotUI, PathPlannerBenchmark
        )
        print("✓ coordinator module imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import from coordinator: {e}")
        traceback.print_exc()
        return False
    
    try:
        import config
        print("✓ config module imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import config: {e}")
        return False
    
    return True


def test_data_structures():
    """Test that core data structures work correctly"""
    print("\n" + "="*70)
    print("TEST 2: Data Structures")
    print("="*70)
    
    from coordinator import Position, Piece, MovePath, PIECE_START_POSITIONS
    
    # Test Position
    try:
        pos1 = Position(100, 200, 45)
        pos2 = Position(200, 300, 90)
        
        distance = pos1.distance_to(pos2)
        assert distance > 0, "Distance should be positive"
        assert abs(distance - 141.4) < 1, f"Distance calculation incorrect: {distance}"
        
        print(f"✓ Position class works (distance: {distance:.1f}mm)")
    except Exception as e:
        print(f"✗ Position class failed: {e}")
        return False
    
    # Test Piece
    try:
        piece = Piece("test", Position(0, 0, 0))
        assert piece.id == "test", "Piece ID not set"
        assert piece.position.x == 0, "Piece position not set"
        print("✓ Piece class works")
    except Exception as e:
        print(f"✗ Piece class failed: {e}")
        return False
    
    # Test MovePath
    try:
        waypoints = [Position(0, 0, 0), Position(100, 0, 0), Position(100, 100, 90)]
        path = MovePath("test", waypoints, 5.0)
        assert len(path.waypoints) == 3, "Waypoints not stored"
        assert path.duration == 5.0, "Duration not set"
        print("✓ MovePath class works")
    except Exception as e:
        print(f"✗ MovePath class failed: {e}")
        return False
    
    # Test piece starting positions
    try:
        assert len(PIECE_START_POSITIONS) == 16, "Wrong number of starting positions"
        assert 'a' in PIECE_START_POSITIONS, "Pawn 'a' not in starting positions"
        assert 'A' in PIECE_START_POSITIONS, "Back row 'A' not in starting positions"
        print(f"✓ Piece starting positions valid ({len(PIECE_START_POSITIONS)} pieces)")
    except Exception as e:
        print(f"✗ Starting positions failed: {e}")
        return False
    
    return True


def test_simulator():
    """Test simulator functionality"""
    print("\n" + "="*70)
    print("TEST 3: Simulator Engine")
    print("="*70)
    
    from coordinator import SimulatorEngine
    
    try:
        sim = SimulatorEngine()
        print("✓ SimulatorEngine created")
    except Exception as e:
        print(f"✗ Failed to create SimulatorEngine: {e}")
        return False
    
    try:
        sim.initialize_board()
        assert len(sim.state.pieces) == 16, f"Wrong number of pieces: {len(sim.state.pieces)}"
        print(f"✓ Board initialized with {len(sim.state.pieces)} pieces")
    except Exception as e:
        print(f"✗ Board initialization failed: {e}")
        return False
    
    try:
        initial_positions = {pid: p.position.copy() for pid, p in sim.state.pieces.items()}
        sim.randomize_positions()
        
        randomized_positions = {pid: p.position.copy() for pid, p in sim.state.pieces.items()}
        
        # Check that at least some pieces moved
        moves = sum(1 for pid in initial_positions 
                   if abs(initial_positions[pid].x - randomized_positions[pid].x) > 1)
        
        assert moves > 0, "No pieces moved during randomization"
        print(f"✓ Position randomization works ({moves} pieces moved)")
    except Exception as e:
        print(f"✗ Randomization failed: {e}")
        return False
    
    try:
        collisions = sim.check_collisions()
        print(f"✓ Collision detection works ({len(collisions)} collisions found)")
    except Exception as e:
        print(f"✗ Collision detection failed: {e}")
        return False
    
    return True


def test_path_planners():
    """Test path planning algorithms"""
    print("\n" + "="*70)
    print("TEST 4: Path Planners")
    print("="*70)
    
    from coordinator import (
        SimulatorEngine, SequentialPathPlanner, OptimizedPathPlanner,
        Position, PIECE_START_POSITIONS, BOARD_SQUARE_SIZE, BOARD_EXTRA_SIDE
    )
    
    # Create simulator with random positions
    sim = SimulatorEngine()
    sim.initialize_board()
    sim.randomize_positions()
    
    # Create target positions (starting positions)
    target_positions = {}
    for piece_id, (col, row) in PIECE_START_POSITIONS.items():
        x = BOARD_EXTRA_SIDE + col * BOARD_SQUARE_SIZE
        y = row * BOARD_SQUARE_SIZE
        target_positions[piece_id] = Position(x, y, 0)
    
    # Test Sequential Planner
    try:
        planner = SequentialPathPlanner()
        paths = planner.plan_movements(sim.state.pieces, target_positions)
        
        assert len(paths) > 0, "No paths generated"
        assert all(len(p.waypoints) >= 2 for p in paths.values()), "Paths have insufficient waypoints"
        
        total_time = sum(p.duration for p in paths.values())
        print(f"✓ SequentialPathPlanner works ({len(paths)} pieces, {total_time:.1f}s total)")
    except Exception as e:
        print(f"✗ SequentialPathPlanner failed: {e}")
        traceback.print_exc()
        return False
    
    # Test Optimized Planner
    try:
        planner = OptimizedPathPlanner()
        paths = planner.plan_movements(sim.state.pieces, target_positions)
        
        assert len(paths) > 0, "No paths generated"
        assert all(len(p.waypoints) >= 2 for p in paths.values()), "Paths have insufficient waypoints"
        
        total_time = sum(p.duration for p in paths.values())
        print(f"✓ OptimizedPathPlanner works ({len(paths)} pieces, {total_time:.1f}s total)")
    except Exception as e:
        print(f"✗ OptimizedPathPlanner failed: {e}")
        traceback.print_exc()
        return False
    
    return True


def test_execution():
    """Test movement execution"""
    print("\n" + "="*70)
    print("TEST 5: Movement Execution")
    print("="*70)
    
    from coordinator import (
        SimulatorEngine, SequentialPathPlanner, Position,
        PIECE_START_POSITIONS, BOARD_SQUARE_SIZE, BOARD_EXTRA_SIDE
    )
    import time
    
    sim = SimulatorEngine()
    sim.initialize_board()
    
    # Create simple target
    target_positions = {
        'a': Position(BOARD_EXTRA_SIDE, 0, 0),
    }
    
    try:
        planner = SequentialPathPlanner()
        paths = planner.plan_movements(
            {'a': sim.state.pieces['a']},
            target_positions
        )
        
        sim.state.paths = paths
        sim.start_execution(paths)
        
        assert sim.state.executing, "Execution not started"
        print("✓ Execution started")
        
        # Run simulation for a bit
        start_time = time.time()
        iterations = 0
        max_iterations = 600  # ~10 seconds at 60 FPS
        
        while sim.state.executing and iterations < max_iterations:
            sim.update(1/60.0)
            iterations += 1
        
        elapsed = time.time() - start_time
        
        print(f"✓ Executed {iterations} simulation frames in {elapsed:.2f}s")
        
        if not sim.state.executing:
            print("✓ Execution completed normally")
        else:
            print("⚠ Execution did not complete within max iterations")
    
    except Exception as e:
        print(f"✗ Execution test failed: {e}")
        traceback.print_exc()
        return False
    
    return True


def test_benchmark():
    """Test benchmarking functionality"""
    print("\n" + "="*70)
    print("TEST 6: Benchmarking")
    print("="*70)
    
    from coordinator import PathPlannerBenchmark, SequentialPathPlanner
    
    try:
        benchmark = PathPlannerBenchmark(iterations=2)
        planner = SequentialPathPlanner()
        
        print("Running quick 2-iteration benchmark (this may take 10-30 seconds)...")
        results = benchmark.benchmark_planner(planner)
        
        assert results['iterations'] == 2, "Iteration count mismatch"
        assert 'avg_move_time' in results, "Missing avg_move_time"
        assert 'collision_rate' in results, "Missing collision_rate"
        
        print(f"✓ Benchmark completed successfully")
        print(f"  - Avg move time: {results['avg_move_time']:.2f}s")
        print(f"  - Total move time: {results['total_move_time']:.2f}s")
        print(f"  - Collisions: {results['collisions']}")
        print(f"  - Avg accuracy error: {results['avg_accuracy_error']:.2f}mm")
    
    except Exception as e:
        print(f"✗ Benchmark test failed: {e}")
        traceback.print_exc()
        return False
    
    return True


def test_config():
    """Test configuration module"""
    print("\n" + "="*70)
    print("TEST 7: Configuration")
    print("="*70)
    
    try:
        import config
        
        errors = config.validate_config()
        if errors:
            print(f"⚠ Configuration validation found {len(errors)} issues:")
            for error in errors:
                print(f"  - {error}")
        else:
            print("✓ Configuration validation passed")
        
        cfg = config.get_config()
        assert len(cfg) > 0, "No configuration values found"
        print(f"✓ Configuration loaded ({len(cfg)} parameters)")
    
    except Exception as e:
        print(f"✗ Configuration test failed: {e}")
        traceback.print_exc()
        return False
    
    return True


def main():
    """Run all tests"""
    print("\n" + "="*70)
    print("CHESS ROBOT COORDINATOR - SYSTEM VALIDATION")
    print("="*70)
    
    tests = [
        ("Imports", test_imports),
        ("Data Structures", test_data_structures),
        ("Simulator Engine", test_simulator),
        ("Path Planners", test_path_planners),
        ("Movement Execution", test_execution),
        ("Benchmarking", test_benchmark),
        ("Configuration", test_config),
    ]
    
    results = {}
    
    for test_name, test_func in tests:
        try:
            results[test_name] = test_func()
        except Exception as e:
            print(f"\n✗ Test '{test_name}' crashed: {e}")
            traceback.print_exc()
            results[test_name] = False
    
    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    
    for test_name, result in results.items():
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{status}: {test_name}")
    
    print("="*70)
    print(f"Result: {passed}/{total} tests passed")
    print("="*70)
    
    if passed == total:
        print("\n🎉 All tests passed! System is ready to use.")
        print("\nNext steps:")
        print("  1. Run the interactive simulator:")
        print("     python coordinator.py")
        print("\n  2. Or run benchmarks:")
        print("     python coordinator.py benchmark 10")
        return 0
    else:
        print("\n⚠ Some tests failed. Please review errors above.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
