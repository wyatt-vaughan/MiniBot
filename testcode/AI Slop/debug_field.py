#!/usr/bin/env python3
import math

# Single electromagnet test
source_x = 50.0
source_y = 50.0
meas_x = 100.0
meas_y = 90.0
meas_z = 40.0

# EM0 configuration: (50.0, 50.0, 0.0, 0.0, 1.0)
# angle_xy = 0, angle_z = 0
angle_xy = 0.0
angle_z = 0.0

# Calculate dipole direction
xy_magnitude = math.cos(angle_z)  # 1.0
z_component = math.sin(angle_z)   # 0.0

dx = math.cos(angle_xy) * xy_magnitude  # 1.0 * 1.0 = 1.0
dy = math.sin(angle_xy) * xy_magnitude  # 0.0 * 1.0 = 0.0
dz = z_component  # 0.0

print(f"Dipole direction: ({dx}, {dy}, {dz})")

# Vector from source to measurement
rx = meas_x - source_x  # 50.0
ry = meas_y - source_y  # 40.0
rz = meas_z  # 40.0
r = math.sqrt(rx**2 + ry**2 + rz**2)
print(f"Distance vector: ({rx}, {ry}, {rz}), r = {r}")

# Unit vector
rx_hat = rx / r
ry_hat = ry / r
rz_hat = rz / r
print(f"Unit vector: ({rx_hat}, {ry_hat}, {rz_hat})")

# Dot product
m_dot_r = dx * rx_hat + dy * ry_hat + dz * rz_hat
print(f"m·r̂ = {m_dot_r}")

# Field calculation
K = 1.0
moment = 1.0
falloff = 2.5
r_power = r ** falloff
constant = (K * moment) / (r_power + 1e-6)

print(f"r^{falloff} = {r_power}")
print(f"constant = {constant}")

bx = constant * (3 * m_dot_r * rx_hat - dx)
by = constant * (3 * m_dot_r * ry_hat - dy)
bz = constant * (3 * m_dot_r * rz_hat - dz)

mag = math.sqrt(bx**2 + by**2 + bz**2)

print(f"Field: Bx={bx}, By={by}, Bz={bz}")
print(f"Magnitude: {mag}")
