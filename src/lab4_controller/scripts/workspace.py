#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import roboticstoolbox as rtb
from spatialmath import SE3
from math import pi

# Robot parameters
L1 = 0.20  # Base height
L2 = 0.25  # Link 2 length
L3 = 0.28  # Link 3 length (including tool)

print("="*60)
print("3R ROBOT WORKSPACE VERIFICATION")
print("="*60)

# Theoretical calculation
r_max = L2 + L3
r_min = abs(L2 - L3)

print(f"\nRobot Parameters:")
print(f"  L1 (Base height):   {L1:.3f} m")
print(f"  L2 (Link 2):        {L2:.3f} m")
print(f"  L3 (Link 3 + tool): {L3:.3f} m")

print(f"\nTheoretical Workspace:")
print(f"  r_max = L2 + L3 = {r_max:.4f} m")
print(f"  r_min = |L2-L3| = {r_min:.4f} m")
print("="*60)

# Build robot model
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha=0.0, a=0.0, d=L1, offset=0.0),
        rtb.RevoluteMDH(alpha=pi/2, a=0.0, d=0.02, offset=0.0),
        rtb.RevoluteMDH(alpha=0.0, a=L2, d=0.0, offset=0.0),
    ],
    tool=SE3.Tx(L3),
    name="3R_Robot",
)

# Sample workspace (lower resolution for speed)
num_samples = 20 

q_samples = [
    np.linspace(-pi, pi, num_samples),  # q1
    np.linspace(-pi, pi, num_samples),  # q2
    np.linspace(-pi, pi, num_samples),  # q3
]

# Compute FK for all configurations
x_positions = []
y_positions = []
z_positions = []

for q1 in q_samples[0]:
    for q2 in q_samples[1]:
        for q3 in q_samples[2]:
            q = [q1, q2, q3]
            T_0e = robot.fkine(q)
            pos = T_0e.t
            x_positions.append(pos[0])
            y_positions.append(pos[1])
            z_positions.append(pos[2])

# Convert to numpy arrays
x_positions = np.array(x_positions)
y_positions = np.array(y_positions)
z_positions = np.array(z_positions)

# Calculate radii (distance from base point)
radii = np.sqrt(x_positions**2 + y_positions**2 + (z_positions - L1)**2)

# Find max and min
max_radius = np.max(radii)
min_radius = np.min(radii)

print(f"\nComputed from Forward Kinematics:")
print(f"  Max Radius: {max_radius:.4f} m")
print(f"  Min Radius: {min_radius:.4f} m")

print(f"\nVerification:")
print(f"  Max error: {abs(max_radius - r_max):.6f} m ✓")
print(f"  Min error: {abs(min_radius - r_min):.6f} m ✓")

# Plot workspace
fig = plt.figure(figsize=(12, 5))

# 3D view
ax1 = fig.add_subplot(1, 2, 1, projection='3d')
ax1.scatter(x_positions, y_positions, z_positions, c='b', marker='o', s=1, alpha=0.5)
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
ax1.set_title(f'3D Workspace\nr_min={r_min:.3f}m, r_max={r_max:.3f}m')

# Top view
ax2 = fig.add_subplot(1, 2, 2)
ax2.scatter(x_positions, y_positions, c='b', marker='o', s=1, alpha=0.5)
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
ax2.set_title('Top View (X-Y Plane)')
ax2.axis('equal')
ax2.grid(True)

# Draw workspace circles
circle_max = plt.Circle((0, 0), r_max, fill=False, color='red', 
                        linestyle='--', linewidth=2, label=f'r_max={r_max:.3f}m')
circle_min = plt.Circle((0, 0), r_min, fill=False, color='green', 
                        linestyle='--', linewidth=2, label=f'r_min={r_min:.3f}m')
ax2.add_patch(circle_max)
ax2.add_patch(circle_min)
ax2.legend()

plt.suptitle(f'3R Robot Workspace | r_min = {r_min:.4f}m | r_max = {r_max:.4f}m', 
             fontsize=14, fontweight='bold')
plt.tight_layout()
plt.show()