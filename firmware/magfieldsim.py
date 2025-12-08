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
    GRID_RESOLUTION = 100        # Number of grid points per axis
    
    # ===== Measurement Point =====
    MEASUREMENT_X = 100.0        # mm - X coordinate of measurement point
    MEASUREMENT_Y = 100.0        # mm - Y coordinate of measurement point
    MEASUREMENT_Z = 10.0         # mm - Z offset (height above XY plane)
    
    # ===== Electromagnet Configuration =====
    COIL_RADIUS = 5.0            # mm - Physical radius of coil
    MAGNETIC_CONSTANT_K = 1.0    # Field strength scaling factor
    FIELD_FALLOFF_EXPONENT = 2.5 # How quickly field falls off with distance
    
    # ===== Visualization =====
    FIELD_COLORMAP = 'plasma'    # Colormap for field magnitude visualization
    VECTOR_SPACING = 8            # Space between vector field arrows
    VECTOR_MAGNITUDE_MAX = 1.0    # Cap on vector magnitude to prevent huge vectors near source
    
    # ===== Electromagnet Definitions =====
    # Format: (x_mm, y_mm, angle_xy_rad, angle_z_rad, dipole_moment)
    # angle_xy: rotation in XY plane (0 = pointing right, π/2 = pointing up)
    # angle_z: tilt toward Z axis (0 = in XY plane, π/2 = pointing out of plane, -π/2 = pointing into plane)
    ELECTROMAGNETS = [
        (50.0, 100.0, 0.0, 0.0, 1.0),           # Left, pointing right, in XY plane
        (150.0, 100.0, math.pi, 0.0, 1.0),      # Right, pointing left, in XY plane
        (100.0, 50.0, math.pi/2, 0.0, 1.0),     # Bottom, pointing up, in XY plane
        (100.0, 150.0, -math.pi/2, 0.0, 1.0),   # Top, pointing down, in XY plane
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
# SIMULATION ENGINE
# =============================================================================

class MagneticFieldSimulation:
    """Main simulation engine"""
    
    def __init__(self):
        """Initialize simulation"""
        self.electromagnets: List[Electromagnet] = []
        self.measurements: List[MagneticFieldMeasurement] = []
        
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
        table_data.append(['EM', 'Magnitude', 'Bx', 'By', 'Bz'])
        for m in self.sim.measurements:
            table_data.append([
                f'EM{m.electromagnet_index}',
                f'{m.field_magnitude:.4f}',
                f'{m.field_x:.4f}',
                f'{m.field_y:.4f}',
                f'{m.field_z:.4f}'
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
    
    # Take measurements
    print("Measuring fields...")
    sim.measure_all()
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
    plt.close(fig1)
    
    # Plot measurements
    fig2 = viz.plot_measurements()
    output_path = os.path.join(output_dir, 'measurements.png')
    plt.savefig(output_path, dpi=150)
    print(f"Saved: {output_path}")
    plt.close(fig2)
    
    print("\nDone!")
    plt.show()


if __name__ == '__main__':
    main()
