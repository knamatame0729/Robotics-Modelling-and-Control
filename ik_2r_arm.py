import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# Link lengths
l1, l2 = 0.6, 0.35

# Forward Kinematics
def fk_pos(q):
    q1, q2 = q
    return np.array([
        l1*np.cos(q1) + l2*np.cos(q1+q2),
        l1*np.sin(q1) + l2*np.sin(q1+q2)
        ])

# Analytic IK
def ik_planar_2R(px, py, elbow_up=True):
    """Return [q1, q2] or None if unreachalbe"""
    D = (px**2 + py**2 - l1**2 - l2**2) / (2*l1*l2)
    if abs(D) > 1.0:
        return None    # Unrechable target

    s2 = np.sqrt(1 - D**2)
    q2 = np.arctan2(+s2, D) if elbow_up else np.arctan(-s2, D)

    k1 = l1 + l2*np.cos(q2)
    k2 =      l2*np.sin(q2)
    q1 = np.arctan2(py, px) - np.arctan2(k2, k1)
    return np.array([q1, q2])

# Parameters
px, py = 0.4, 0.2
elbow_up = True

outer_r = l1 + l2 - 1e-4
inner_r = abs(l1 - l2) + 1e-4

target = np.array([px, py])
q_sol = ik_planar_2R(px, py, elbow_up)

plt.figure(figsize=(5,5))
ax = plt.gca(); ax.set_aspect('equal'); ax.grid(True)
span = outer_r + 0.05
ax.set_xlim(-span, span); ax.set_ylim(-span, span)

# Reachable area
ax.add_patch(Circle((0,0), outer_r, facecolor='none',
                    linestyle='--', edgecolor='grey'))
ax.add_patch(Circle((0,0), inner_r, facecolor='white', edgecolor='grey'))

ax.scatter(*target, color='red', zorder=5, label='target')

if q_sol is None:
    ax.set_title('Unreachable target')
    ax.legend(); plt.show()
else:
    q1, q2 = q_sol
    base = np.zeros(2)
    elbow_pt = np.array([l1*np.cos(q1), l1*np.sin(q1)])
    tip = fk_pos(q_sol)
    ax.plot([base[0], elbow_pt[0], tip[0]],
            [base[1], elbow_pt[1], tip[1]],
            '-o', lw=3,
            label='elbow_up' if elbow_up else 'elbow_down')
    ax.set_title(f'q1={np.degrees(q1):.1f}°, q2={np.degrees(q2):.1f}°')
    ax.legend(); plt.show()









