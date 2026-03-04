import numpy as np

# Modified DH parameters for KUKA KR 500 R2830 (α, a, d, θoffset)
DHM = np.array([
    [0,        0,    1045, 0],
    [-np.pi/2, 500,     0, 0],
    [0,      1300,     0, -np.pi/2],
    [-np.pi/2, -55,  1025, 0],
    [np.pi/2,   0,     0, 0],
    [-np.pi/2,  0,   290, np.pi]
])

# Convenience aliases
alpha        = DHM[:, 0]
a            = DHM[:, 1]
d            = DHM[:, 2]
theta_offset = DHM[:, 3]

# Joint limits in degrees
joint_max_deg = [185, 20, 144, 350, 120, 350]
joint_min_deg = [-185, -130, -100, -350, -120, -350]

"""
This module stores the Modified DH parameters and joint limits for the KUKA KR 500 R2830.
Source: RobotKinematicsCatalogue (KUKA_KR500_R2830.py).
"""
