# Magnetic Field Simulator

A modular, high-quality Python simulation tool for visualizing magnetic fields created by multiple electromagnets with independent control.

## Features

- **Multiple Electromagnets**: Place and control up to N electromagnets at arbitrary positions and orientations
- **Independent Control**: Each electromagnet can be turned on/off independently via an activation cycle
- **Field Measurement**: Continuously measure field strength and orientation at a specific point
- **Realistic Physics**: Uses proper magnetic dipole field equations with configurable falloff
- **Rich Visualization**:
  - Field magnitude contour plots
  - Vector field visualization
  - Time-series measurements
  - Electromagnet state tracking
  - Animated simulation
  
## Configuration

All parameters are centralized in the `Config` class at the top of the script:

### Scene Setup
```python
SCENE_WIDTH = 200.0          # Scene width in mm
SCENE_HEIGHT = 200.0         # Scene height in mm
GRID_RESOLUTION = 100        # Grid points for field calculation
```

### Measurement Point
```python
MEASUREMENT_X = 100.0        # X coordinate (mm)
MEASUREMENT_Y = 100.0        # Y coordinate (mm)
```

### Electromagnet Definition
Define electromagnets in the `ELECTROMAGNETS` list:
```python
ELECTROMAGNETS = [
    (x_mm, y_mm, angle_rad, dipole_moment),
    ...
]
```

Example:
```python
ELECTROMAGNETS = [
    (50.0, 100.0, 0.0, 1.0),           # At (50,100), pointing right (0°)
    (150.0, 100.0, math.pi, 1.0),      # At (150,100), pointing left (180°)
    (100.0, 50.0, math.pi/2, 1.0),     # At (100,50), pointing up (90°)
]
```

### Activation Cycle
Define when electromagnets turn on/off:
```python
ACTIVATION_CYCLE = [
    (timestep, electromagnet_idx, state),
    (0, 0, True),      # Turn on EM 0 at t=0
    (50, 0, False),    # Turn off EM 0 at t=50
    (50, 1, True),     # Turn on EM 1 at t=50
    ...
]
```

### Physics Parameters
```python
MAGNETIC_CONSTANT_K = 1.0         # Field strength scaling
FIELD_FALLOFF_EXPONENT = 2.5      # Distance falloff (2.5 ~ 1/r^2.5)
```

## Output

The script generates:
- `field_magnitude_t0.png` - Field visualization at start
- `field_magnitude_final.png` - Field visualization at end
- `measurements.png` - Time-series measurements
- `field_animation.gif` - Animated simulation

## Architecture

### Main Classes

1. **Config** - Central configuration class
2. **Electromagnet** - Represents a single electromagnet
3. **MagneticFieldSample** - Data container for measurements
4. **MagneticFieldCalculator** - Physics calculations
5. **MagneticFieldSimulation** - Main simulation engine
6. **MagneticFieldVisualizer** - Visualization and plotting

### Key Methods

- `MagneticFieldCalculator.calculate_dipole_field()` - Single dipole field calculation
- `MagneticFieldCalculator.calculate_total_field()` - Superposition of all active dipoles
- `MagneticFieldSimulation.step()` - Execute one simulation step
- `MagneticFieldSimulation.run_simulation()` - Run full simulation
- `MagneticFieldSimulation.get_field_grid()` - Calculate field on spatial grid

## Physics

The simulator uses the magnetic dipole field formula:

```
B = (μ₀/4π) * (3(m·r̂)r̂ - m) / r³
```

Where:
- B is the magnetic field
- m is the dipole moment vector
- r̂ is the unit vector from dipole to measurement point
- r is the distance

Field falloff can be customized with `FIELD_FALLOFF_EXPONENT` for non-ideal behavior.

## Example Usage

### Modify Configuration

Edit the `Config` class:

```python
class Config:
    # Change measurement point
    MEASUREMENT_X = 75.0
    MEASUREMENT_Y = 125.0
    
    # Add more electromagnets
    ELECTROMAGNETS = [
        (50.0, 100.0, 0.0, 1.0),
        (150.0, 100.0, math.pi, 1.0),
        (100.0, 50.0, math.pi/2, 0.8),      # Weaker dipole
        (100.0, 150.0, -math.pi/2, 1.2),    # Stronger dipole
        (200.0, 50.0, math.pi/4, 0.5),      # Diagonal
    ]
    
    # Sequence all electromagnets
    ACTIVATION_CYCLE = [
        (0, 0, True),
        (100, 0, False),
        (100, 1, True),
        (200, 1, False),
        (200, 2, True),
        (300, 2, False),
        (300, 3, True),
        (400, 3, False),
        (400, 4, True),
        (500, 4, False),
    ]
```

### Run Simulation

```bash
python magfieldsim.py
```

## Measurements Output

The simulator records and plots:
1. **Field Magnitude** - Total field strength over time
2. **Field Angle** - Field direction/orientation over time
3. **Active Electromagnets** - Which electromagnets are on at each timestep

All measurements are available in `sim.measurements` list containing `MagneticFieldSample` objects with:
- `time` - Simulation time in seconds
- `active_magnets` - Number of active electromagnets
- `field_magnitude` - Total field strength
- `field_x`, `field_y` - Field components
- `field_angle` - Field direction in radians

## Customization

### Change Visualization
- `FIELD_COLORMAP` - Use any matplotlib colormap ('plasma', 'viridis', 'hot', etc.)
- `VECTOR_SPACING` - Adjust density of vector field arrows
- `FPS` - Animation frame rate

### Adjust Physics
- `MAGNETIC_CONSTANT_K` - Make fields stronger/weaker
- `FIELD_FALLOFF_EXPONENT` - Adjust how quickly field decays with distance
- `COIL_RADIUS` - Visual size of electromagnet circles

### Performance
- `GRID_RESOLUTION` - Fewer points = faster but lower quality
- `MEASUREMENT_SAMPLES` - Fewer samples = shorter simulation

## Requirements

```
numpy
matplotlib
```

Install with:
```bash
pip install numpy matplotlib
```
