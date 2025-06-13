import numpy as np
import matplotlib.pyplot as plt
from spatialmath import SE3

plt.rcParams['figure.dpi'] = 110

# 1. Generic single-pass FK routine
def forwar_kinematics_single_pass(q, parents, T_relative_fn):
    N = len(parents)
    T_world = [None] * N
    for i in range(N):                                                        # single forward sweep
        T_pc = T_relative_fn(i, q)                                            # parent→child
        T_world[i] = T_pc if parents[i] == -1 else T_world[parents[i]] * T_pc # Compute the Transform
    return T_world


# 2. Example robot: planar 3-R arm (RRR)
parents = [-1, 0, 1]               # world - L0 - L1 - L2
lengths = [0.4, 0.35, 0.25]        # link lengths [m]
q_demo = np.deg2rad([30, 20, -25]) # Sample joint angles

def T_relative(i, q):
    """ Revolte Z joint followed by X translation along link length """
    return SE3.Rz(q[i]) * SE3.Tx(lengths[i])

# Compute FK
T_list = forwar_kinematics_single_pass(q_demo, parents, T_relative)

# 3. Print transforms
for i, Ti in enumerate(T_list):
    print(f"Link {i} world transform:\n{np.round(Ti.A, 4)}\n")

# 4. Quick 2‑D plot to verify geometry
pts = np.vstack(([0, 0, 0], [Ti.t for Ti in T_list]))  # include world origin
plt.figure(figsize=(4, 4))
plt.plot(pts[:, 0], pts[:, 1], '-o', lw=3)
plt.gca().set_aspect('equal'); plt.grid(True)
plt.title('3-R planar arm (single-pass FK)')
plt.xlabel('x [m]'); plt.ylabel('y [m]')
plt.show()