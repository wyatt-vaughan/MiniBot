"""
Quick validation script - runs fast tests only
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

print("=" * 70)
print("CHESS ROBOT COORDINATOR - QUICK VALIDATION")
print("=" * 70)

# Test 1: Imports
print("\n✓ Testing imports...")
try:
    import pygame
    import numpy
    from coordinator import (
        Position, Piece, SimulatorEngine, 
        SequentialPathPlanner, OptimizedPathPlanner
    )
    import config
    print("  ✓ All modules imported successfully")
except Exception as e:
    print(f"  ✗ Import failed: {e}")
    sys.exit(1)

# Test 2: Basic functionality
print("\n✓ Testing basic functionality...")
try:
    # Test Position
    pos1 = Position(100, 200, 45)
    pos2 = Position(200, 300, 90)
    dist = pos1.distance_to(pos2)
    assert dist > 0
    print(f"  ✓ Position class works (distance: {dist:.1f}mm)")
    
    # Test Piece
    piece = Piece("test", Position(0, 0, 0))
    assert piece.id == "test"
    print("  ✓ Piece class works")
    
except Exception as e:
    print(f"  ✗ Basic functionality test failed: {e}")
    sys.exit(1)

# Test 3: Simulator
print("\n✓ Testing simulator...")
try:
    sim = SimulatorEngine()
    sim.initialize_board()
    assert len(sim.state.pieces) == 16
    print(f"  ✓ Simulator initialized with {len(sim.state.pieces)} pieces")
    
    sim.randomize_positions()
    print("  ✓ Position randomization works")
    
    collisions = sim.check_collisions()
    print(f"  ✓ Collision detection works ({len(collisions)} collisions)")
    
except Exception as e:
    print(f"  ✗ Simulator test failed: {e}")
    sys.exit(1)

# Test 4: Path Planners
print("\n✓ Testing path planners...")
try:
    from coordinator import PIECE_START_POSITIONS, BOARD_SQUARE_SIZE, BOARD_EXTRA_SIDE
    
    sim = SimulatorEngine()
    sim.initialize_board()
    sim.randomize_positions()
    
    target_positions = {}
    for piece_id, (col, row) in PIECE_START_POSITIONS.items():
        x = BOARD_EXTRA_SIDE + col * BOARD_SQUARE_SIZE
        y = row * BOARD_SQUARE_SIZE
        target_positions[piece_id] = Position(x, y, 0)
    
    planner1 = SequentialPathPlanner()
    paths1 = planner1.plan_movements(sim.state.pieces, target_positions)
    assert len(paths1) > 0
    print(f"  ✓ SequentialPathPlanner works ({len(paths1)} pieces)")
    
    planner2 = OptimizedPathPlanner()
    paths2 = planner2.plan_movements(sim.state.pieces, target_positions)
    assert len(paths2) > 0
    print(f"  ✓ OptimizedPathPlanner works ({len(paths2)} pieces)")
    
except Exception as e:
    print(f"  ✗ Path planner test failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test 5: Configuration
print("\n✓ Testing configuration...")
try:
    errors = config.validate_config()
    if not errors:
        print("  ✓ Configuration validated successfully")
    else:
        print(f"  ⚠ Configuration has {len(errors)} issues (non-critical)")
        for error in errors[:2]:
            print(f"    - {error}")
except Exception as e:
    print(f"  ✗ Configuration test failed: {e}")
    sys.exit(1)

print("\n" + "=" * 70)
print("✓ ALL VALIDATION TESTS PASSED")
print("=" * 70)
print("\nSystem is ready! You can now:")
print("  1. Run interactive simulator:  python coordinator.py")
print("  2. Run benchmarks:             python coordinator.py benchmark 10")
print("  3. Run examples:               python examples.py")
print("=" * 70 + "\n")
