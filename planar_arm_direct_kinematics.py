import numpy as np

def planar_arm_direct_kinematics(a1, a2, a3, theta1, theta2, theta3):
    theta12 = theta1 + theta2
    theta123 = theta12 + theta3

    x = a1 * np.cos(theta1) + a2 * np.cos(theta12) + a3 * np.sin(theta123)
    y = a1 * np.sin(theta1) + a2 * np.sin(theta12) + a3 * np.sin(theta123)

    T = np.eye(4)
    T[0, 0] = np.cos(theta123)
    T[0, 1] = -np.sin(theta123)
    T[0, 3] = x
    T[1, 0] = np.sin(theta123)
    T[1, 1] = np.cos(theta123)
    T[1, 3] = y
    
    return T

# Link lengths
a1, a2, a3 = 1.0, 1.0, 0.5

# Joint angles in radians
theta1 = np.deg2rad(30)
theta2 = np.deg2rad(45)
theta3 = np.deg2rad(-15)

# Compute direct kinematics
T03 = planar_arm_direct_kinematics(a1, a2, a3, theta1, theta2, theta3)
print("T_0^3 = \n", T03)

