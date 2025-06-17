import numpy as np
from scipy.spatial.transform import Rotation as R

# Define the ground-truth hand-eye transformation X = [R_gt | t_gt]
pos_gt = np.array([0.05, 0.015, 0.08]) # Translation vector in meters
rotvec_gt = np.array([0.15, -0.24, 0.12]) # Rotaion vector in radians
R_gt = R.from_euler('xyz', rotvec_gt).as_matrix()

X_gt = np.eye(4)
X_gt[:3, :3] = R_gt
X_gt[:3, 3] = pos_gt

# Function to generate motion pairs (A_i, B_i)
def generate_motion_pair():
    rotvec = np.random.uniform(-np.pi, np.pi, 3)
    t = np.random.uniform(-0.1, 0.1, 3)
    Rb = R.from_euler('xyz', rotvec).as_matrix()

    B = np.eye(4)
    B[:3, :3] = Rb
    B[:3, 3] = t

    A = X_gt @ B @ np.linalg.inv(X_gt)
    return A, B

# Generate 1000 pairs of (A_i, B_i)
A_list, B_list = zip(*[generate_motion_pair() for _ in range(1000)])

# Print a few samples for verification
np.set_printoptions(precision=4, suppress=True)
print("Ground-truth transformation X_gt:\n", X_gt)
for i in range(3):
    print(f"\nSample A[{i}] Rotation:\n", A_list[i][:3, :3])
    print(f"Sample B[{i}] Rotation:\n", B_list[i][:3, :3])

