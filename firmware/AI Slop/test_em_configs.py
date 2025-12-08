#!/usr/bin/env python3
"""Test different electromagnet configurations quickly"""
import subprocess
import re
import sys

# Different electromagnet configurations to test (4 groups of 3 orthogonal coils each)
CONFIGS = {
    "original_scattered": [
        (25.0, 50.0, 0.0, 0.0, 1.0),
        (25.0, 50.0, 1.5708, 0.0, 1.0),
        (25.0, 50.0, 0.0, 1.5708, 1.0),
        
        (150.0, 50.0, 0.0, 0.0, 1.0),
        (150.0, 50.0, 1.5708, 0.0, 1.0),
        (150.0, 50.0, 0.0, 1.5708, 1.0),
        
        (150.0, 250.0, 0.0, 0.0, 1.0),
        (150.0, 250.0, 1.5708, 0.0, 1.0),
        (150.0, 250.0, 0.0, 1.5708, 1.0),
        
        (275.0, 250.0, 0.0, 0.0, 1.0),
        (275.0, 250.0, 1.5708, 0.0, 1.0),
        (275.0, 250.0, 0.0, 1.5708, 1.0),
    ],
    "corners_only": [
        (50.0, 50.0, 0.0, 0.0, 1.0),
        (50.0, 50.0, 1.5708, 0.0, 1.0),
        (50.0, 50.0, 0.0, 1.5708, 1.0),
        
        (250.0, 50.0, 0.0, 0.0, 1.0),
        (250.0, 50.0, 1.5708, 0.0, 1.0),
        (250.0, 50.0, 0.0, 1.5708, 1.0),
        
        (50.0, 250.0, 0.0, 0.0, 1.0),
        (50.0, 250.0, 1.5708, 0.0, 1.0),
        (50.0, 250.0, 0.0, 1.5708, 1.0),
        
        (250.0, 250.0, 0.0, 0.0, 1.0),
        (250.0, 250.0, 1.5708, 0.0, 1.0),
        (250.0, 250.0, 0.0, 1.5708, 1.0),
    ],
    "cross_pattern": [
        (100.0, 100.0, 0.0, 0.0, 1.0),
        (100.0, 100.0, 1.5708, 0.0, 1.0),
        (100.0, 100.0, 0.0, 1.5708, 1.0),
        
        (200.0, 100.0, 0.0, 0.0, 1.0),
        (200.0, 100.0, 1.5708, 0.0, 1.0),
        (200.0, 100.0, 0.0, 1.5708, 1.0),
        
        (100.0, 200.0, 0.0, 0.0, 1.0),
        (100.0, 200.0, 1.5708, 0.0, 1.0),
        (100.0, 200.0, 0.0, 1.5708, 1.0),
        
        (200.0, 200.0, 0.0, 0.0, 1.0),
        (200.0, 200.0, 1.5708, 0.0, 1.0),
        (200.0, 200.0, 0.0, 1.5708, 1.0),
    ],
    "outer_ring": [
        (30.0, 30.0, 0.0, 0.0, 1.0),
        (30.0, 30.0, 1.5708, 0.0, 1.0),
        (30.0, 30.0, 0.0, 1.5708, 1.0),
        
        (270.0, 30.0, 0.0, 0.0, 1.0),
        (270.0, 30.0, 1.5708, 0.0, 1.0),
        (270.0, 30.0, 0.0, 1.5708, 1.0),
        
        (30.0, 270.0, 0.0, 0.0, 1.0),
        (30.0, 270.0, 1.5708, 0.0, 1.0),
        (30.0, 270.0, 0.0, 1.5708, 1.0),
        
        (270.0, 270.0, 0.0, 0.0, 1.0),
        (270.0, 270.0, 1.5708, 0.0, 1.0),
        (270.0, 270.0, 0.0, 1.5708, 1.0),
    ],
}

# Weights to test
WEIGHTS = [
    (1.0, 2.0),  # angle weight 2x magnitude
    (1.0, 3.0),  # angle weight 3x magnitude (current)
    (1.0, 4.0),  # angle weight 4x magnitude
    (1.0, 5.0),  # angle weight 5x magnitude
]

def update_em_config(config_list):
    """Update magfieldsim.py with new EM configuration"""
    # Read the file
    with open('magfieldsim.py', 'r') as f:
        lines = f.readlines()
    
    # Find ELECTROMAGNETS start and end
    start_idx = None
    end_idx = None
    for i, line in enumerate(lines):
        if 'ELECTROMAGNETS = [' in line:
            start_idx = i
        elif start_idx is not None and line.strip().startswith(']'):
            end_idx = i
            break
    
    if start_idx is None or end_idx is None:
        print("ERROR: Could not find ELECTROMAGNETS in magfieldsim.py")
        return
    
    # Build new EM list
    new_lines = []
    new_lines.append('    ELECTROMAGNETS = [\n')
    for x, y, angle_xy, angle_z, dipole in config_list:
        new_lines.append(f'        ({x}, {y}, {angle_xy}, {angle_z}, {dipole}),\n')
    new_lines.append('    ]\n')
    
    # Replace in file
    lines = lines[:start_idx] + new_lines + lines[end_idx+1:]
    
    with open('magfieldsim.py', 'w') as f:
        f.writelines(lines)

def update_weights(mag_weight, angle_weight):
    """Update weighting in magfieldsim.py"""
    with open('magfieldsim.py', 'r') as f:
        content = f.read()
    
    # Replace weights
    content = re.sub(
        r'POS_EST_MAG_WEIGHT = [\d.]+',
        f'POS_EST_MAG_WEIGHT = {mag_weight}',
        content
    )
    content = re.sub(
        r'POS_EST_ANGLE_WEIGHT = [\d.]+',
        f'POS_EST_ANGLE_WEIGHT = {angle_weight}',
        content
    )
    
    with open('magfieldsim.py', 'w') as f:
        f.write(content)

def run_test():
    """Run magfieldsim and extract stats"""
    try:
        result = subprocess.run(
            ['python', 'magfieldsim.py'],
            capture_output=True,
            text=True,
            timeout=120
        )
        output = result.stdout
        
        # Extract statistics
        mean_error = re.search(r'Mean Error: ([\d.]+) mm', output)
        confidence = re.search(r'Mean Confidence: ([\d.]+)', output)
        timing = re.search(r'Average Time: ([\d.]+) ms', output)
        
        if mean_error and confidence and timing:
            return {
                'error': float(mean_error.group(1)),
                'confidence': float(confidence.group(1)),
                'timing': float(timing.group(1)),
            }
    except subprocess.TimeoutExpired:
        print("  TIMEOUT - test took too long")
        return None
    except Exception as e:
        print(f"  ERROR: {e}")
        return None
    
    return None

def main():
    print("\n" + "="*70)
    print("ELECTROMAGNET CONFIGURATION OPTIMIZATION")
    print("="*70 + "\n")
    
    results = {}
    
    # Test each EM configuration with baseline weights
    print("Testing EM CONFIGURATIONS (with baseline weights 1.0/3.0):\n")
    for config_name, em_config in CONFIGS.items():
        print(f"Testing: {config_name}")
        update_em_config(em_config)
        update_weights(1.0, 3.0)
        
        stats = run_test()
        if stats:
            results[config_name] = stats
            print(f"  Error: {stats['error']:.2f}mm, Confidence: {stats['confidence']:.3f}, Timing: {stats['timing']:.1f}ms")
        else:
            print(f"  FAILED")
    
    # Find best config
    if results:
        best_config = min(results.keys(), key=lambda x: results[x]['error'])
        print(f"\n► Best config: {best_config} with {results[best_config]['error']:.2f}mm error\n")
        
        # Now test weights with best config
        print("Testing WEIGHTS with best config:\n")
        best_em = CONFIGS[best_config]
        
        for mag_w, angle_w in WEIGHTS:
            weight_str = f"{mag_w}/{angle_w}"
            print(f"Testing weights: {weight_str}")
            update_weights(mag_w, angle_w)
            
            stats = run_test()
            if stats:
                results[weight_str] = stats
                print(f"  Error: {stats['error']:.2f}mm, Confidence: {stats['confidence']:.3f}, Timing: {stats['timing']:.1f}ms")
            else:
                print(f"  FAILED")
        
        # Summary
        print("\n" + "="*70)
        print("SUMMARY (sorted by error):")
        print("="*70)
        sorted_results = sorted(results.items(), key=lambda x: x[1]['error'])
        for name, stats in sorted_results:
            print(f"{name:25} Error: {stats['error']:6.2f}mm  Conf: {stats['confidence']:.3f}  Time: {stats['timing']:6.1f}ms")

if __name__ == '__main__':
    main()
