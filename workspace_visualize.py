import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

l1, l2 = 0.78, 0.54       # Link Lengths [m]
q1_lim = (-np.pi, np.pi)  # Joint-1 limits [rads]
q2_lim = (-np.pi, np.pi)  # Joint-2 limits [rads]
samples = 250             # Grid density per joint
show_3d = True            # Toggle 3-D (x, y, θ) scatter

# Forward Kinematics (position only)
def fk_pos(q):
    q1, q2 = q
    x = l1 * np.cos(q1) + l2 * np.cos(q1+q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1+q2)
    return np.array([x, y])

# Workspace sampling
q1 = np.linspace(*q1_lim, samples)
q2 = np.linspace(*q2_lim, samples)
Q1, Q2 = np.meshgrid(q1, q2, indexing='ij')
X_flat = np.array([fk_pos((a, b)) for a, b in zip(Q1.ravel(), Q2.ravel())])
theta = (Q1 + Q2).ravel()

# ----------- 2-D scatter (position only) -----------
plt.figure(figsize=(5,5))
plt.scatter(X_flat[:,0], X_flat[:,1], s=1)
plt.gca().set_aspect('equal'); plt.grid(True)
plt.title('Reachable workspace (position only)')
plt.xlabel('x [m]'); plt.ylabel('y [m]')
plt.show()

# ----------- Pose visualization for specific q1, q2 ----------
def show_pose(q1, q2):
    base = np.array([0,0])
    elbow = np.array([l1*np.cos(q1), l1*np.sin(q1)])
    tip = np.array([l1*np.cos(q1)+l2*np.cos(q1+q2),
                    l1*np.sin(q1)+l2*np.sin(q1+q2)])
    plt.figure(figsize=(4,4))
    plt.plot([base[0], elbow[0], tip[0]], [base[1], elbow[1], tip[1]], '-o', lw=3)
    plt.xlim(-1.1*(l1+l2), 1.1*(l1+l2)); plt.ylim(-1.1*(l1+l2), 1.1*(l1+l2))
    plt.gca().set_aspect('equal'); plt.grid(True)
    plt.title(f'Pose - tip = {tip.round(3)} m, θ = {(q1+q2):.2f} rad')
    plt.show()

show_pose(np.deg2rad(30), np.deg2rad(-45))

# ------- 3-D workspace (position + orientation) -------
if show_3d:
    fig = plt.figure(figsize=(6,5))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(X_flat[:,0], X_flat[:,1], theta, s=1)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_zlabel('θ [rad]')
    ax.set_title('Reachable workspace including orientation')
    plt.show()
