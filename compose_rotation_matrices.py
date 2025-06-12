import numpy as np

def Rz(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0, 0, 1]
    ])

def Ry(theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

# Rotation from Frame 2 to Frame 1 (R^1_2): rotate 90° about z
R1_2 = Rz(np.pi / 2)

# Rotation from Frame 1 to Frame 0 (R^0_1): rotate 90° about y
R0_1 = Ry(np.pi / 2)

# Compose to get R^0_2
R0_2 = R0_1 @ R1_2

np.set_printoptions(precision=5, suppress=True)
print("R^0_2 = R^0_1 * R^1_2:\n", R0_2)