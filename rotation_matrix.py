import numpy as np

def Rx(gamma):
    return np.array([
        [1, 0, 0],
        [0, np.cos(gamma), -np.sin(gamma)],
        [0, np.sin(gamma), np.cos(gamma)]
    ])

def Ry(beta):
    return np.array([
        [np.cos(beta),  0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])

def Rz(alpha):
    return np.array([
        [np.cos(alpha), -np.sin(alpha), 0],
        [np.sin(alpha), np.cos(alpha), 0],
        [0, 0, 1]
    ])

# Rotate 45 deg about each axis
gamma = np.deg2rad(45)
beta = np.deg2rad(45)
alpha = np.deg2rad(45)

print("Rx:\n", Rx(gamma))
print("Ry:\n", Ry(beta))
print("Rz:\n", Rz(alpha))

# Vector in body frame
p_body = np.array([[1], [0], [0]])

# Rotate using Rz(90°)
rotation = Rz(90)
p_world = rotation @ p_body
p_body_back = rotation.T @ p_world

print("p in world frame:\n", p_world)
print("p back in body frame:\n", p_body_back)
