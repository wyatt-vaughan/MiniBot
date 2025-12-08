#!/usr/bin/env python3
"""
Simplified Magnetic Field Simulator
Measures field at one point for each electromagnet (one at a time)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch
from dataclasses import dataclass
from typing import List, Tuple
import math
import os

# =============================================================================
# CONFIGURATION - All parameters defined here
# =============================================================================

class Config:
    """Central configuration for magnetic field simulation"""
    
    # ===== Scene Configuration =====
    SCENE_WIDTH = 200.0          # mm
    SCENE_HEIGHT = 200.0         # mm
    GRID_RESOLUTION = 500        # Number of grid points per axis
    
    # ===== Measurement Point =====
    MEASUREMENT_X = 100.0        # mm - X coordinate of measurement point
    MEASUREMENT_Y = 90.0        # mm - Y coordinate of measurement point
    MEASUREMENT_Z = 40.0         # mm - Z offset (height above XY plane)
    
    # ===== Grid Measurement Configuration =====
    GRID_MEAS_X_MIN = 0.0         # mm - Minimum X for grid measurements
    GRID_MEAS_X_MAX = 300.0       # mm - Maximum X for grid measurements
    GRID_MEAS_Y_MIN = 0.0         # mm - Minimum Y for grid measurements
    GRID_MEAS_Y_MAX = 300.0       # mm - Maximum Y for grid measurements
    GRID_MEAS_Z = 40.0            # mm - Z height for grid measurements
    GRID_MEAS_POINTS = 15         # Number of points per axis (15x15 = 225 points)
    
    # ===== Electromagnet Configuration =====
    COIL_RADIUS = 10.0            # mm - Physical radius of coil
    MAGNETIC_CONSTANT_K = 1000.0    # Field strength scaling factor
    FIELD_FALLOFF_EXPONENT = 2.5 # How quickly field falls off with distance
    
    # ===== Position Estimator Configuration =====
    # These parameters control the pose estimation algorithm
    
    # Search resolution (higher = slower but more accurate)
    POS_EST_COARSE_GRID = 13           # Coarse grid search resolution (XY)
    POS_EST_COARSE_THETA = 12          # Coarse grid search orientations
    POS_EST_FINE_GRID = 15             # Fine grid search resolution around best
    POS_EST_FINE_THETA = 12            # Fine grid search orientations
    
    # Error weighting
    POS_EST_MAG_WEIGHT = 1.0           # Weight for magnitude in cost function
    POS_EST_ANGLE_WEIGHT = 3.0         # Weight for azimuth angle in cost function (higher = more important)
    POS_EST_ANGLE_WEIGHT_Z = 0.5       # Z-component penalty (not used, kept for compatibility)
    
    # Distance-based weighting (closer measurements more reliable)
    POS_EST_USE_DISTANCE_WEIGHTING = False  # Weight errors by distance to source
    POS_EST_DISTANCE_THRESHOLD = 200.0    # mm - Distance at which weighting equals 1.0
    
    # Outlier handling
    POS_EST_MAGNITUDE_THRESHOLD = 0.00001  # Ignore very weak signals
    POS_EST_OUTLIER_THRESHOLD = 2.5    # Std devs above mean to consider outlier
    POS_EST_USE_OUTLIER_REJECTION = True  # Enable outlier rejection
    
    # Measurement noise model
    POS_EST_MAGNITUDE_NOISE = 0.05      # Expected measurement noise (5% of magnitude)
    POS_EST_ANGLE_NOISE = 10.0          # Expected angle noise (degrees)
    
    # Refinement parameters
    POS_EST_REFINE_ITERATIONS = 50     # More iterations for better convergence
    POS_EST_ADAPTIVE_STEP = True       # Use adaptive step sizes
    
    # ===== Visualization =====
    FIELD_COLORMAP = 'plasma'    # Colormap for field magnitude visualization
    VECTOR_SPACING = 8            # Space between vector field arrows
    VECTOR_MAGNITUDE_MAX = 1.0    # Cap on vector magnitude to prevent huge vectors near source
    
    # ===== Electromagnet Definitions =====
    # Format: (x_mm, y_mm, angle_xy_rad, angle_z_rad, dipole_moment)
    # angle_xy: rotation in XY plane (0 = pointing right, π/2 = pointing up)
    # angle_z: tilt toward Z axis (0 = in XY plane, π/2 = pointing out of plane, -π/2 = pointing into plane)
    # 12 electromagnets = 4 groups of 3 orthogonal coils
    # Original scattered configuration that achieves 5.53mm accuracy
    ELECTROMAGNETS = [
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
    ]


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class Electromagnet:
    """Represents a single electromagnet in the scene"""
    x: float              # Position X (mm)
    y: float              # Position Y (mm)
    angle_xy: float       # Rotation in XY plane (radians)
    angle_z: float        # Tilt toward Z axis (radians)
    dipole_moment: float  # Magnetic dipole strength
    
    def get_position(self) -> Tuple[float, float]:
        """Get electromagnet position"""
        return (self.x, self.y)
    
    def get_dipole_direction(self) -> Tuple[float, float, float]:
        """Get unit vector in direction of magnetic dipole in 3D"""
        # Start with XY plane orientation
        xy_magnitude = math.cos(self.angle_z)  # How much projects onto XY plane
        z_component = math.sin(self.angle_z)    # Z component
        
        # XY components scaled by projection
        dx = math.cos(self.angle_xy) * xy_magnitude
        dy = math.sin(self.angle_xy) * xy_magnitude
        dz = z_component
        
        return (dx, dy, dz)


@dataclass
class MagneticFieldMeasurement:
    """Measurement of magnetic field from a single electromagnet"""
    electromagnet_index: int  # Which electromagnet was on
    field_magnitude: float    # Total field magnitude (Tesla/unit)
    field_x: float           # Field X component
    field_y: float           # Field Y component
    field_z: float           # Field Z component
    field_angle_xy: float    # Field orientation in XY plane (radians)


@dataclass
class GridMeasurement:
    """Measurement of magnetic field at a grid point from all electromagnets"""
    x: float                          # X position (mm)
    y: float                          # Y position (mm)
    z: float                          # Z position (mm)
    measurements: List[Tuple]         # List of (em_idx, magnitude, azimuth_deg) for each EM
    
    def get_measurement(self, em_idx: int) -> Tuple:
        """Get measurement from specific electromagnet"""
        return self.measurements[em_idx]


@dataclass
class PositionEstimate:
    """Estimated position and orientation"""
    x: float                 # Estimated X position (mm)
    y: float                 # Estimated Y position (mm)
    theta: float             # Estimated orientation in radians
    confidence: float        # Confidence score (0-1)


# =============================================================================
# MAGNETIC FIELD CALCULATION
# =============================================================================

class MagneticFieldCalculator:
    """Calculates magnetic fields from electromagnets"""
    
    @staticmethod
    def calculate_dipole_field(source_x: float, source_y: float,
                              dipole_x: float, dipole_y: float, dipole_z: float,
                              moment: float,
                              meas_x: float, meas_y: float, meas_z: float) -> Tuple[float, float, float]:
        """
        Calculate magnetic field from a 3D dipole at measurement point.
        
        Uses dipole field formula:
        B = (μ₀/4π) * (3(m·r̂)r̂ - m) / r³
        
        Args:
            source_x, source_y: Dipole source position in XY plane
            dipole_x, dipole_y, dipole_z: Unit dipole direction (3D)
            moment: Dipole moment strength
            meas_x, meas_y, meas_z: Measurement point in 3D
            
        Returns:
            (Bx, By, Bz) - Field components in 3D
        """
        # Vector from source to measurement point (3D)
        rx = meas_x - source_x
        ry = meas_y - source_y
        rz = meas_z
        r = math.sqrt(rx**2 + ry**2 + rz**2)
        
        # Avoid singularity
        if r < 0.1:
            return (0.0, 0.0, 0.0)
        
        # Unit vector and distance
        rx_hat = rx / r
        ry_hat = ry / r
        rz_hat = rz / r
        
        # Dot product m·r̂
        m_dot_r = dipole_x * rx_hat + dipole_y * ry_hat + dipole_z * rz_hat
        
        # Falloff with distance
        r_power = r ** Config.FIELD_FALLOFF_EXPONENT
        
        # Dipole field formula
        constant = (Config.MAGNETIC_CONSTANT_K * moment) / (r_power + 1e-6)
        
        # B = (3(m·r̂)r̂ - m) / r³
        bx = constant * (3 * m_dot_r * rx_hat - dipole_x)
        by = constant * (3 * m_dot_r * ry_hat - dipole_y)
        bz = constant * (3 * m_dot_r * rz_hat - dipole_z)
        
        return (bx, by, bz)
    
    @staticmethod
    def calculate_field_from_electromagnet(em: Electromagnet,
                                          meas_x: float, meas_y: float, meas_z: float) -> Tuple[float, float, float, float]:
        """
        Calculate total magnetic field from one electromagnet at measurement point.
        
        Returns:
            (Bx, By, Bz, magnitude) - Field components and magnitude
        """
        sx, sy = em.get_position()
        dx, dy, dz = em.get_dipole_direction()
        
        bx, by, bz = MagneticFieldCalculator.calculate_dipole_field(
            sx, sy, dx, dy, dz, em.dipole_moment,
            meas_x, meas_y, meas_z
        )
        
        magnitude = math.sqrt(bx**2 + by**2 + bz**2)
        
        return (bx, by, bz, magnitude)


# =============================================================================
# POSITION ESTIMATOR
# =============================================================================

class PositionEstimator:
    """Estimates robot position and orientation from magnetic field measurements"""
    
    def __init__(self, electromagnets: List[Electromagnet]):
        """Initialize estimator with electromagnet positions"""
        self.electromagnets = electromagnets
        self.last_estimate_time_ms = 0.0  # Track time of last estimation
    
    def estimate_pose(self, measurements: List[Tuple[int, float, float]], 
                      search_grid_size: int = None) -> PositionEstimate:
        """
        Estimate robot position (x, y) and orientation (theta) from all electromagnet measurements.
        
        Uses grid search combining all EM measurements to determine best pose.
        Azimuth is weighted higher than magnitude for better orientation accuracy.
        
        Args:
            measurements: List of (em_idx, magnitude, azimuth_deg) tuples from ALL electromagnets
            search_grid_size: Ignored (uses config values)
            
        Returns:
            PositionEstimate with x, y, theta, and confidence
        """
        import time
        start_time = time.time()
        
        # Grid search
        best_pose = None
        best_score = float('inf')
        second_best_score = float('inf')  # Track runner-up for confidence calculation
        
        x_range = np.linspace(Config.GRID_MEAS_X_MIN, Config.GRID_MEAS_X_MAX, Config.POS_EST_COARSE_GRID)
        y_range = np.linspace(Config.GRID_MEAS_Y_MIN, Config.GRID_MEAS_Y_MAX, Config.POS_EST_COARSE_GRID)
        theta_range = np.linspace(0, 2*math.pi, Config.POS_EST_COARSE_THETA, endpoint=False)
        
        for x in x_range:
            for y in y_range:
                for theta in theta_range:
                    score = self._compute_error_fast(x, y, theta, measurements)
                    if score < best_score:
                        second_best_score = best_score  # Previous best becomes second best
                        best_score = score
                        best_pose = (x, y, theta)
                    elif score < second_best_score:
                        second_best_score = score
        
        if best_pose is None:
            return PositionEstimate(x=150, y=150, theta=0, confidence=0.0)
        
        # Refine the best pose with local optimization for better accuracy
        refined_pose = self._refine_pose(best_pose[0], best_pose[1], best_pose[2], measurements)
        best_score_refined = self._compute_error_fast(refined_pose[0], refined_pose[1], refined_pose[2], measurements)
        
        # Compute confidence based on separation between best and runner-up
        # If scores are close, confidence is low (ambiguous)
        # If best is much better than runner-up, confidence is high (clear winner)
        if second_best_score == float('inf') or second_best_score <= best_score_refined:
            confidence = 0.5  # No clear separation
        else:
            # Ratio of scores: how much better is best than second best?
            margin = (second_best_score - best_score_refined) / (second_best_score + 1e-6)
            confidence = min(1.0, margin * 1.5)  # Amplify margin slightly for better distribution
        
        elapsed_time = (time.time() - start_time) * 1000.0  # Convert to milliseconds
        self.last_estimate_time_ms = elapsed_time
        
        return PositionEstimate(
            x=refined_pose[0],
            y=refined_pose[1],
            theta=refined_pose[2],
            confidence=confidence
        )
    
    def _compute_error_fast(self, x: float, y: float, theta: float, 
                           measurements: List[Tuple[int, float, float]]) -> float:
        """
        Compute total error across all electromagnet measurements.
        
        Uses both magnitude and azimuth, with azimuth weighted higher for better orientation accuracy.
        
        Args:
            x, y: Estimated position (mm)
            theta: Estimated orientation (radians)
            measurements: List of (em_idx, magnitude, azimuth_deg) from all EMs
            
        Returns:
            Total error score (lower is better)
        """
        error = 0.0
        
        for em_idx, measured_mag, measured_azimuth in measurements:
            if measured_mag < Config.POS_EST_MAGNITUDE_THRESHOLD:
                continue
            
            em = self.electromagnets[em_idx]
            
            # Calculate distance to electromagnet (used for weighting)
            dist_to_em = math.sqrt((em.x - x)**2 + (em.y - y)**2)
            
            # Distance weighting: closer sources are more reliable
            if Config.POS_EST_USE_DISTANCE_WEIGHTING:
                distance_weight = Config.POS_EST_DISTANCE_THRESHOLD / (dist_to_em + Config.POS_EST_DISTANCE_THRESHOLD)
            else:
                distance_weight = 1.0
            
            # Predict field from this electromagnet at robot position
            pred_bx, pred_by, pred_bz, pred_mag = \
                MagneticFieldCalculator.calculate_field_from_electromagnet(
                    em, x, y, Config.GRID_MEAS_Z
                )
            
            # Magnitude error - simple normalized difference
            if pred_mag > 1e-8:
                mag_error = abs(measured_mag - pred_mag) / (pred_mag + 1e-6)
                error += mag_error * Config.POS_EST_MAG_WEIGHT * distance_weight
            
            # Azimuth error (weighted higher than magnitude)
            if measured_mag > 1e-7:
                pred_azimuth = math.degrees(math.atan2(pred_by, pred_bx))
                if pred_azimuth < 0:
                    pred_azimuth += 360
                
                # Adjust predicted azimuth by robot orientation
                robot_relative_azimuth = (pred_azimuth - math.degrees(theta)) % 360
                
                # Circular distance (shortest path around circle)
                angle_diff = min(abs(measured_azimuth - robot_relative_azimuth),
                               360 - abs(measured_azimuth - robot_relative_azimuth))
                error += angle_diff * Config.POS_EST_ANGLE_WEIGHT * distance_weight
        
        return error
    
    def _compute_error(self, x: float, y: float, theta: float, 
                      measurements: List[Tuple[int, float, float]]) -> float:
        """Compute error between measured and predicted field at pose (with outlier rejection)"""
        error = 0.0
        mag_errors = []
        angle_errors = []
        
        for em_idx, measured_mag, measured_azimuth in measurements:
            if measured_mag < Config.POS_EST_MAGNITUDE_THRESHOLD:
                continue
            
            em = self.electromagnets[em_idx]
            
            # Predict field from this electromagnet at robot position
            pred_bx, pred_by, pred_bz, pred_mag = \
                MagneticFieldCalculator.calculate_field_from_electromagnet(
                    em, x, y, Config.GRID_MEAS_Z
                )
            
            # Magnitude error
            if pred_mag > 0:
                mag_error = abs(measured_mag - pred_mag) / (pred_mag + 1e-6)
                mag_errors.append(mag_error)
                error += mag_error * Config.POS_EST_MAG_WEIGHT
            
            # Azimuth error (taking into account robot orientation)
            pred_azimuth = math.degrees(math.atan2(pred_by, pred_bx))
            if pred_azimuth < 0:
                pred_azimuth += 360
            
            # Adjust predicted azimuth by robot orientation
            robot_relative_azimuth = (pred_azimuth - math.degrees(theta)) % 360
            
            # Angle difference (shortest path around circle)
            angle_diff = min(abs(measured_azimuth - robot_relative_azimuth),
                           360 - abs(measured_azimuth - robot_relative_azimuth))
            angle_errors.append(angle_diff)
            error += angle_diff * Config.POS_EST_ANGLE_WEIGHT
        
        # Apply outlier rejection (simplified)
        if Config.POS_EST_USE_OUTLIER_REJECTION and len(mag_errors) > 3:
            mag_mean = np.mean(mag_errors) if len(mag_errors) > 0 else 0
            mag_std = np.std(mag_errors) if len(mag_errors) > 1 else 1
            if mag_std > 0:
                for mag_err in mag_errors:
                    if mag_err > mag_mean + Config.POS_EST_OUTLIER_THRESHOLD * mag_std:
                        error -= mag_err * Config.POS_EST_MAG_WEIGHT
        
        return error
    
    def _refine_pose(self, x: float, y: float, theta: float,
                     measurements: List[Tuple[int, float, float]]) -> Tuple[float, float, float]:
        """
        Refine pose estimate using adaptive local optimization.
        
        Uses adaptive step sizes that shrink over iterations for convergence.
        """
        step_size_xy = (Config.GRID_MEAS_X_MAX - Config.GRID_MEAS_X_MIN) / 20  # 5% of scene width
        step_size_theta = math.pi / 8  # 22.5 degrees
        max_iterations = Config.POS_EST_REFINE_ITERATIONS
        
        no_improve_count = 0
        max_no_improve = 5  # Reduce step size if no improvement
        
        for iteration in range(max_iterations):
            current_error = self._compute_error_fast(x, y, theta, measurements)
            improved = False
            best_new_error = current_error
            best_delta = None
            
            # Adaptive step size reduction
            if Config.POS_EST_ADAPTIVE_STEP and iteration > 0:
                factor = max(0.3, 1.0 - iteration / (max_iterations * 2))
                current_step_xy = step_size_xy * factor
                current_step_theta = step_size_theta * factor
            else:
                current_step_xy = step_size_xy
                current_step_theta = step_size_theta
            
            # Try all 6 directions
            for dx, dy, dtheta in [
                (current_step_xy, 0, 0), (-current_step_xy, 0, 0),
                (0, current_step_xy, 0), (0, -current_step_xy, 0),
                (0, 0, current_step_theta), (0, 0, -current_step_theta)
            ]:
                new_x = x + dx
                new_y = y + dy
                new_theta = (theta + dtheta) % (2 * math.pi)
                
                # Keep within bounds
                new_x = max(Config.GRID_MEAS_X_MIN, min(Config.GRID_MEAS_X_MAX, new_x))
                new_y = max(Config.GRID_MEAS_Y_MIN, min(Config.GRID_MEAS_Y_MAX, new_y))
                
                new_error = self._compute_error_fast(new_x, new_y, new_theta, measurements)
                
                if new_error < best_new_error:
                    best_new_error = new_error
                    best_delta = (dx, dy, dtheta)
                    improved = True
            
            if improved:
                dx, dy, dtheta = best_delta
                x = x + dx
                y = y + dy
                theta = (theta + dtheta) % (2 * math.pi)
                no_improve_count = 0
            else:
                no_improve_count += 1
                if no_improve_count >= max_no_improve:
                    break
        
        return (x, y, theta)


# =============================================================================
# SIMULATION ENGINE
# =============================================================================

class MagneticFieldSimulation:
    """Main simulation engine"""
    
    def __init__(self):
        """Initialize simulation"""
        self.electromagnets: List[Electromagnet] = []
        self.measurements: List[MagneticFieldMeasurement] = []
        self.grid_measurements: List[GridMeasurement] = []
        
        self._initialize_electromagnets()
    
    def _initialize_electromagnets(self) -> None:
        """Create electromagnets from configuration"""
        for x, y, angle_xy, angle_z, moment in Config.ELECTROMAGNETS:
            em = Electromagnet(x=x, y=y, angle_xy=angle_xy, angle_z=angle_z, dipole_moment=moment)
            self.electromagnets.append(em)
    
    def measure_all(self) -> None:
        """Take one measurement from each electromagnet"""
        print(f"Taking measurements at ({Config.MEASUREMENT_X}, {Config.MEASUREMENT_Y}, {Config.MEASUREMENT_Z}) mm")
        
        for em_idx, em in enumerate(self.electromagnets):
            bx, by, bz, mag = MagneticFieldCalculator.calculate_field_from_electromagnet(
                em,
                Config.MEASUREMENT_X,
                Config.MEASUREMENT_Y,
                Config.MEASUREMENT_Z
            )
            
            angle_xy = math.atan2(by, bx)
            
            measurement = MagneticFieldMeasurement(
                electromagnet_index=em_idx,
                field_magnitude=mag,
                field_x=bx,
                field_y=by,
                field_z=bz,
                field_angle_xy=angle_xy
            )
            self.measurements.append(measurement)
            
            print(f"  EM{em_idx}: B = {mag:.4f} (Bx={bx:.4f}, By={by:.4f}, Bz={bz:.4f})")
    
    def measure_grid(self) -> None:
        """Take measurements across a 2D grid of points"""
        print(f"Measuring grid across XY plane at Z={Config.GRID_MEAS_Z}mm")
        print(f"Grid range: X=[{Config.GRID_MEAS_X_MIN}, {Config.GRID_MEAS_X_MAX}] mm")
        print(f"Grid range: Y=[{Config.GRID_MEAS_Y_MIN}, {Config.GRID_MEAS_Y_MAX}] mm")
        print(f"Grid points: {Config.GRID_MEAS_POINTS}x{Config.GRID_MEAS_POINTS} = {Config.GRID_MEAS_POINTS**2} measurements")
        
        x_points = np.linspace(Config.GRID_MEAS_X_MIN, Config.GRID_MEAS_X_MAX, Config.GRID_MEAS_POINTS)
        y_points = np.linspace(Config.GRID_MEAS_Y_MIN, Config.GRID_MEAS_Y_MAX, Config.GRID_MEAS_POINTS)
        
        self.grid_measurements = []
        
        for y in y_points:
            for x in x_points:
                # Measure field from each electromagnet individually
                em_measurements = []
                
                for em_idx, em in enumerate(self.electromagnets):
                    bx, by, bz, mag = MagneticFieldCalculator.calculate_field_from_electromagnet(
                        em, x, y, Config.GRID_MEAS_Z
                    )
                    azimuth = math.degrees(math.atan2(by, bx))
                    
                    # Normalize azimuth to [0, 360) range
                    if azimuth < 0:
                        azimuth += 360
                    
                    em_measurements.append((em_idx, mag, azimuth))
                
                meas = GridMeasurement(
                    x=x, y=y, z=Config.GRID_MEAS_Z,
                    measurements=em_measurements
                )
                self.grid_measurements.append(meas)
        
        print(f"Grid measurement complete: {len(self.grid_measurements)} points")
    
    def get_field_grid_for_electromagnet(self, em_idx: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Calculate field magnitude and vectors on a grid for one electromagnet.
        
        Returns:
            (X, Y, Bx_grid, By_grid) - Grid positions and field components in XY plane at Z offset
        """
        x = np.linspace(0, Config.SCENE_WIDTH, Config.GRID_RESOLUTION)
        y = np.linspace(0, Config.SCENE_HEIGHT, Config.GRID_RESOLUTION)
        X, Y = np.meshgrid(x, y)
        
        Bx_grid = np.zeros_like(X)
        By_grid = np.zeros_like(Y)
        
        em = self.electromagnets[em_idx]
        
        for i in range(len(x)):
            for j in range(len(y)):
                bx, by, bz, _ = MagneticFieldCalculator.calculate_field_from_electromagnet(
                    em, X[j, i], Y[j, i], Config.MEASUREMENT_Z
                )
                Bx_grid[j, i] = bx
                By_grid[j, i] = by
        
        return (X, Y, Bx_grid, By_grid)


# =============================================================================
# VISUALIZATION
# =============================================================================

class MagneticFieldVisualizer:
    """Handles visualization of simulation"""
    
    def __init__(self, simulation: MagneticFieldSimulation):
        """Initialize visualizer"""
        self.sim = simulation
    
    def plot_all_electromagnets(self) -> None:
        """Create figure with one subplot per electromagnet"""
        n_em = len(self.sim.electromagnets)
        fig, axes = plt.subplots(2, 2, figsize=(14, 12))
        axes = axes.flatten()
        
        for em_idx, ax in enumerate(axes):
            if em_idx < n_em:
                self._plot_electromagnet(ax, em_idx)
            else:
                ax.axis('off')
        
        plt.tight_layout()
        return fig
    
    def _plot_electromagnet(self, ax, em_idx: int) -> None:
        """Plot field for a single electromagnet"""
        X, Y, Bx, By = self.sim.get_field_grid_for_electromagnet(em_idx)
        B_mag = np.sqrt(Bx**2 + By**2)
        
        # Plot magnitude with contours
        contourf = ax.contourf(X, Y, B_mag, levels=20, cmap=Config.FIELD_COLORMAP)
        ax.contour(X, Y, B_mag, levels=10, colors='gray', alpha=0.3, linewidths=0.5)
        
        # Plot vector field with magnitude capped
        skip = Config.VECTOR_SPACING
        B_mag_skip = B_mag[::skip, ::skip]
        Bx_skip = Bx[::skip, ::skip]
        By_skip = By[::skip, ::skip]
        
        # Cap vector magnitudes to prevent huge vectors near source
        B_mag_capped = np.minimum(B_mag_skip, Config.VECTOR_MAGNITUDE_MAX)
        
        # Normalize vectors and scale by capped magnitude
        B_total = np.sqrt(Bx_skip**2 + By_skip**2)
        Bx_normalized = np.where(B_total > 1e-6, Bx_skip / B_total * B_mag_capped, 0)
        By_normalized = np.where(B_total > 1e-6, By_skip / B_total * B_mag_capped, 0)
        
        ax.quiver(X[::skip, ::skip], Y[::skip, ::skip],
                 Bx_normalized, By_normalized,
                 B_mag_capped, cmap=Config.FIELD_COLORMAP, scale=20)
        
        plt.colorbar(contourf, ax=ax, label='Field Magnitude')
        
        # Draw electromagnets and measurement point
        self._draw_electromagnets(ax)
        self._draw_measurement_point(ax)
        
        ax.set_xlim(0, Config.SCENE_WIDTH)
        ax.set_ylim(0, Config.SCENE_HEIGHT)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_title(f'Electromagnet {em_idx} - XY plane at Z={Config.MEASUREMENT_Z}mm')
    
    def _draw_electromagnets(self, ax) -> None:
        """Draw electromagnets on axis"""
        for i, em in enumerate(self.sim.electromagnets):
            color = 'red' if i < len(self.sim.electromagnets) else 'gray'
            
            # Draw coil circle
            circle = Circle((em.x, em.y), Config.COIL_RADIUS,
                           color=color, fill=False, linewidth=2, alpha=0.7)
            ax.add_patch(circle)
            
            # Draw dipole direction (only XY component for in-plane visualization)
            dx, dy, dz = em.get_dipole_direction()
            arrow_length = Config.COIL_RADIUS * 2
            
            # Scale arrow by XY projection (shorter if pointing out of plane)
            xy_projection = math.sqrt(dx**2 + dy**2)
            arrow_length_scaled = arrow_length * xy_projection
            
            if arrow_length_scaled > 0.1:  # Only draw if significant XY component
                arrow = FancyArrowPatch(
                    (em.x, em.y),
                    (em.x + dx * arrow_length_scaled, em.y + dy * arrow_length_scaled),
                    arrowstyle='->', mutation_scale=20, color=color, alpha=0.7, linewidth=2
                )
                ax.add_patch(arrow)
            
            # If pointing significantly out of plane, draw perpendicular marker
            if abs(dz) > 0.3:
                marker_symbol = '↗' if dz > 0 else '↙'  # Out or into plane
                marker_size = 20 * abs(dz)
                ax.text(em.x + 3, em.y + 3, marker_symbol, fontsize=int(marker_size), 
                       color=color, alpha=0.7)
            
            # Label
            ax.text(em.x, em.y - Config.COIL_RADIUS - 5, f'EM{i}',
                   ha='center', fontsize=9, color=color, alpha=0.7)
    
    def _draw_measurement_point(self, ax) -> None:
        """Draw measurement point on axis"""
        ax.plot(Config.MEASUREMENT_X, Config.MEASUREMENT_Y, 'b*', markersize=20,
               label=f'Measurement Point\n(Z={Config.MEASUREMENT_Z}mm)')
        ax.legend(loc='upper right', fontsize=9)
    
    def plot_grid_measurements(self) -> None:
        """Plot grid measurements from first electromagnet as example"""
        if not self.sim.grid_measurements:
            print("No grid measurements to plot. Run measure_grid() first.")
            return
        
        # Extract grid data for first electromagnet
        n_points = Config.GRID_MEAS_POINTS
        
        magnitude_grid = np.zeros((n_points, n_points))
        azimuth_grid = np.zeros((n_points, n_points))
        
        # Reshape measurements into grids for EM0
        for i, meas in enumerate(self.sim.grid_measurements):
            y_idx = i // n_points
            x_idx = i % n_points
            if len(meas.measurements) > 0:
                em_idx, mag, azimuth = meas.measurements[0]
                magnitude_grid[y_idx, x_idx] = mag
                azimuth_grid[y_idx, x_idx] = azimuth
        
        # Create figure with two subplots
        fig, axes = plt.subplots(1, 2, figsize=(16, 6))
        
        # Plot magnitude heatmap
        im1 = axes[0].imshow(magnitude_grid, extent=[Config.GRID_MEAS_X_MIN, Config.GRID_MEAS_X_MAX,
                                                      Config.GRID_MEAS_Y_MIN, Config.GRID_MEAS_Y_MAX],
                            origin='lower', cmap='viridis', aspect='auto')
        axes[0].set_xlabel('X (mm)')
        axes[0].set_ylabel('Y (mm)')
        axes[0].set_title(f'Field Magnitude Grid - EM0 (Z={Config.GRID_MEAS_Z}mm)')
        cbar1 = plt.colorbar(im1, ax=axes[0], label='Magnitude')
        
        # Draw electromagnets on magnitude plot
        for em in self.sim.electromagnets:
            axes[0].plot(em.x, em.y, 'r+', markersize=15, markeredgewidth=2)
        
        # Plot azimuth heatmap
        im2 = axes[1].imshow(azimuth_grid, extent=[Config.GRID_MEAS_X_MIN, Config.GRID_MEAS_X_MAX,
                                                    Config.GRID_MEAS_Y_MIN, Config.GRID_MEAS_Y_MAX],
                            origin='lower', cmap='hsv', aspect='auto', vmin=0, vmax=360)
        axes[1].set_xlabel('X (mm)')
        axes[1].set_ylabel('Y (mm)')
        axes[1].set_title(f'Field Azimuth Grid - EM0 (Z={Config.GRID_MEAS_Z}mm)')
        cbar2 = plt.colorbar(im2, ax=axes[1], label='Azimuth (degrees)')
        
        # Draw electromagnets on azimuth plot
        for em in self.sim.electromagnets:
            axes[1].plot(em.x, em.y, 'r+', markersize=15, markeredgewidth=2)
        
        plt.tight_layout()
        return fig
    
    def export_grid_to_csv(self, filepath: str) -> None:
        """Export grid measurements to CSV file with individual electromagnet data"""
        if not self.sim.grid_measurements:
            print("No grid measurements to export.")
            return
        
        import csv
        
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            
            # Write header - for each electromagnet: magnitude, azimuth
            header = ['X (mm)', 'Y (mm)', 'Z (mm)']
            for em_idx in range(len(self.sim.electromagnets)):
                header.extend([f'EM{em_idx}_Mag', f'EM{em_idx}_Az'])
            writer.writerow(header)
            
            # Write data
            for meas in self.sim.grid_measurements:
                row = [f'{meas.x:.2f}', f'{meas.y:.2f}', f'{meas.z:.2f}']
                
                for em_idx, mag, azimuth in meas.measurements:
                    row.extend([
                        f'{mag:.6f}',
                        f'{azimuth:.2f}'
                    ])
                
                writer.writerow(row)
        
        print(f"Grid measurements exported to: {filepath}")
    
    def plot_position_estimates(self) -> None:
        """Plot position estimation accuracy across the grid"""
        if not self.sim.grid_measurements:
            print("No grid measurements to plot. Run measure_grid() first.")
            return
        
        estimator = PositionEstimator(self.sim.electromagnets)
        
        n_points = Config.GRID_MEAS_POINTS
        pos_errors = np.zeros((n_points, n_points))
        orientation_errors = np.zeros((n_points, n_points))
        confidence_grid = np.zeros((n_points, n_points))
        
        # Track timing statistics
        total_time = 0.0
        num_estimates = 0
        confidence_values = []
        margin_values = []
        
        # Estimate position for each measurement point
        for i, meas in enumerate(self.sim.grid_measurements):
            y_idx = i // n_points
            x_idx = i % n_points
            
            # Get measurements from this point
            estimate = estimator.estimate_pose(meas.measurements)
            
            # Accumulate timing statistics
            total_time += estimator.last_estimate_time_ms
            num_estimates += 1
            confidence_values.append(estimate.confidence)
            
            # Calculate error from true position
            true_pos = np.array([meas.x, meas.y])
            est_pos = np.array([estimate.x, estimate.y])
            pos_error = np.linalg.norm(true_pos - est_pos)
            
            pos_errors[y_idx, x_idx] = pos_error
            confidence_grid[y_idx, x_idx] = estimate.confidence
        
        # Create figure
        fig, axes = plt.subplots(1, 2, figsize=(16, 6))
        
        # Plot position error
        im1 = axes[0].imshow(pos_errors, extent=[Config.GRID_MEAS_X_MIN, Config.GRID_MEAS_X_MAX,
                                                   Config.GRID_MEAS_Y_MIN, Config.GRID_MEAS_Y_MAX],
                            origin='lower', cmap='RdYlGn_r', aspect='auto')
        axes[0].set_xlabel('X (mm)')
        axes[0].set_ylabel('Y (mm)')
        axes[0].set_title('Position Estimation Error (mm)')
        cbar1 = plt.colorbar(im1, ax=axes[0], label='Error (mm)')
        
        # Draw electromagnets
        for em in self.sim.electromagnets:
            axes[0].plot(em.x, em.y, 'b+', markersize=15, markeredgewidth=2)
        
        # Plot confidence
        im2 = axes[1].imshow(confidence_grid, extent=[Config.GRID_MEAS_X_MIN, Config.GRID_MEAS_X_MAX,
                                                       Config.GRID_MEAS_Y_MIN, Config.GRID_MEAS_Y_MAX],
                            origin='lower', cmap='viridis', aspect='auto', vmin=0, vmax=1)
        axes[1].set_xlabel('X (mm)')
        axes[1].set_ylabel('Y (mm)')
        axes[1].set_title('Estimation Confidence')
        cbar2 = plt.colorbar(im2, ax=axes[1], label='Confidence')
        
        # Draw electromagnets
        for em in self.sim.electromagnets:
            axes[1].plot(em.x, em.y, 'b+', markersize=15, markeredgewidth=2)
        
        plt.tight_layout()
        
        # Print statistics
        valid_errors = pos_errors[pos_errors > 0]
        valid_confidence = np.array(confidence_values)[np.array(pos_errors).flatten() > 0]
        
        if len(valid_errors) > 0:
            print(f"\nPosition Estimation Statistics:")
            print(f"  Mean Error: {np.mean(valid_errors):.2f} mm")
            print(f"  Std Dev: {np.std(valid_errors):.2f} mm")
            print(f"  Min Error: {np.min(valid_errors):.2f} mm")
            print(f"  Max Error: {np.max(valid_errors):.2f} mm")
            print(f"  Mean Confidence: {np.mean(confidence_values):.3f}")
            print(f"  Min Confidence: {np.min(confidence_values):.3f}")
            print(f"  Max Confidence: {np.max(confidence_values):.3f}")
            print(f"  Std Dev Confidence: {np.std(confidence_values):.3f}")
            
            # Analyze correlation between error and confidence
            if len(valid_errors) > 1 and len(valid_confidence) > 1:
                correlation = np.corrcoef(valid_errors, valid_confidence)[0, 1]
                print(f"  Correlation (Error vs Confidence): {correlation:.3f}")
            
            # Print timing statistics
            if num_estimates > 0:
                avg_time = total_time / num_estimates
                print(f"\nTiming Statistics (single position estimate):")
                print(f"  Average Time: {avg_time:.2f} ms")
                print(f"  Total Time: {total_time:.2f} ms for {num_estimates} estimates")
        
        return fig
    
    def plot_measurements(self) -> None:
        """Plot measurement comparison"""
        if not self.sim.measurements:
            print("No measurements to plot. Run measurement first.")
            return
        
        n_measurements = len(self.sim.measurements)
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # Extract data
        indices = [m.electromagnet_index for m in self.sim.measurements]
        magnitudes = [m.field_magnitude for m in self.sim.measurements]
        bx = [m.field_x for m in self.sim.measurements]
        by = [m.field_y for m in self.sim.measurements]
        bz = [m.field_z for m in self.sim.measurements]
        angles_xy = [math.degrees(m.field_angle_xy) for m in self.sim.measurements]
        print(angles_xy)
        
        labels = [f'EM{i}' for i in indices]
        x_pos = np.arange(n_measurements)
        
        # Plot magnitude
        axes[0, 0].bar(x_pos, magnitudes, color='blue', alpha=0.7)
        axes[0, 0].set_ylabel('Field Magnitude')
        axes[0, 0].set_title('Total Field Magnitude at Measurement Point')
        axes[0, 0].set_xticks(x_pos)
        axes[0, 0].set_xticklabels(labels)
        axes[0, 0].grid(True, alpha=0.3, axis='y')
        
        # Plot components
        width = 0.25
        axes[0, 1].bar(x_pos - width, bx, width, label='Bx', alpha=0.8)
        axes[0, 1].bar(x_pos, by, width, label='By', alpha=0.8)
        axes[0, 1].bar(x_pos + width, bz, width, label='Bz', alpha=0.8)
        axes[0, 1].set_ylabel('Field Component')
        axes[0, 1].set_title('Field Components at Measurement Point')
        axes[0, 1].set_xticks(x_pos)
        axes[0, 1].set_xticklabels(labels)
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3, axis='y')
        
        # Plot 3D components
        axes[1, 0].bar(x_pos, bx, label='Bx', alpha=0.6)
        axes[1, 0].bar(x_pos, by, bottom=bx, label='By', alpha=0.6)
        axes[1, 0].bar(x_pos, bz, bottom=np.array(bx) + np.array(by), label='Bz', alpha=0.6)
        axes[1, 0].set_ylabel('Cumulative Field')
        axes[1, 0].set_title('Stacked Field Components')
        axes[1, 0].set_xticks(x_pos)
        axes[1, 0].set_xticklabels(labels)
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3, axis='y')
        
        # Summary table
        axes[1, 1].axis('tight')
        axes[1, 1].axis('off')
        
        table_data = []
        table_data.append(['EM', 'Magnitude', 'Bx', 'By', 'Bz', 'Angle XY (deg)'])
        for m in self.sim.measurements:
            table_data.append([
                f'EM{m.electromagnet_index}',
                f'{m.field_magnitude:.4f}',
                f'{m.field_x:.4f}',
                f'{m.field_y:.4f}',
                f'{m.field_z:.4f}',
                f'{math.degrees(m.field_angle_xy):.1f}'
            ])
        
        table = axes[1, 1].table(cellText=table_data, loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 2)
        axes[1, 1].set_title('Measurement Summary')
        
        plt.tight_layout()
        return fig


# =============================================================================
# MAIN ENTRY POINT
# =============================================================================

def main():
    """Run the magnetic field simulator"""
    # Create output directory
    output_dir = 'magfield_output'
    os.makedirs(output_dir, exist_ok=True)
    
    print("=" * 60)
    print("Magnetic Field Simulator - Simplified")
    print("=" * 60)
    print(f"Scene: {Config.SCENE_WIDTH} x {Config.SCENE_HEIGHT} mm")
    print(f"Measurement Point: ({Config.MEASUREMENT_X}, {Config.MEASUREMENT_Y}, {Config.MEASUREMENT_Z}) mm")
    print(f"Electromagnets: {len(Config.ELECTROMAGNETS)}")
    print(f"Vector Magnitude Cap: {Config.VECTOR_MAGNITUDE_MAX}")
    print(f"Output Directory: {output_dir}")
    print()
    
    # Create simulation
    sim = MagneticFieldSimulation()
    
    # Take point measurements
    print("Measuring fields at point...")
    sim.measure_all()
    print()
    
    # Take grid measurements
    print("Measuring fields on grid...")
    sim.measure_grid()
    print()
    
    # Create visualizer
    viz = MagneticFieldVisualizer(sim)
    
    # Generate plots
    print("Generating visualizations...")
    
    # Plot all electromagnet fields
    fig1 = viz.plot_all_electromagnets()
    output_path = os.path.join(output_dir, 'field_visualization.png')
    plt.savefig(output_path, dpi=150)
    print(f"Saved: {output_path}")
    
    # Plot point measurements
    fig2 = viz.plot_measurements()
    output_path = os.path.join(output_dir, 'measurements.png')
    plt.savefig(output_path, dpi=150)
    print(f"Saved: {output_path}")
    
    # Plot grid measurements
    fig3 = viz.plot_grid_measurements()
    output_path = os.path.join(output_dir, 'grid_measurements.png')
    plt.savefig(output_path, dpi=150)
    print(f"Saved: {output_path}")
    
    # Plot position estimation results
    print("\nEstimating positions across grid...")
    fig4 = viz.plot_position_estimates()
    output_path = os.path.join(output_dir, 'position_estimates.png')
    plt.savefig(output_path, dpi=150)
    print(f"Saved: {output_path}")
    
    # Export grid to CSV
    csv_path = os.path.join(output_dir, 'grid_measurements.csv')
    viz.export_grid_to_csv(csv_path)
    
    print("\nDone!")
    # plt.show()  # Disabled for batch processing


if __name__ == '__main__':
    main()
