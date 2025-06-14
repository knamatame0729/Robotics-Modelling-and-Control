import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

l1, l2 = 0.6, 0.35  # link lengths [m]

def fk_pos(q):
    q1, q2 = q
    return np.array([l1 * np.cos(q1) + l2 * np.cos(q1 + q2),
                     l1 * np.sin(q1) + l2 * np.sin(q1 + q2)])

def jacobian(q):
    q1, q2 = q
    return np.array([
        [-l1 * np.sin(q1) - l2 * np.sin(q1 + q2), -l2 * np.sin(q1 + q2)],
        [ l1 * np.cos(q1) + l2 * np.cos(q1 + q2),  l2 * np.cos(q1 + q2)]
    ])

# ---------- fixed values ----------
q1 = np.deg2rad(145)
q2 = np.deg2rad(30)
sigma_q = 0.1  # joint angle uncertainty [rad]

# ---------- computation ----------
q = np.array([q1, q2])
J = jacobian(q)
Sigma_q = (sigma_q ** 2) * np.eye(2)
Sigma_x = J @ Sigma_q @ J.T

eigvals, eigvecs = np.linalg.eigh(Sigma_x)
order = eigvals.argsort()[::-1]
eigvals, eigvecs = eigvals[order], eigvecs[:, order]

width, height = 2 * np.sqrt(eigvals) * 2  # 95% confidence ellipse
angle = np.degrees(np.arctan2(eigvecs[1, 0], eigvecs[0, 0]))

# ---------- plotting ----------
tip = fk_pos(q)
plt.figure(figsize=(5, 5))
ellipse = Ellipse(xy=tip, width=width, height=height,
                  angle=angle, edgecolor='red', facecolor='none', lw=2,
                  label='95% pos. uncertainty')
plt.gca().add_patch(ellipse)

base = np.zeros(2)
elbow = np.array([l1 * np.cos(q1), l1 * np.sin(q1)])
plt.plot([base[0], elbow[0], tip[0]], [base[1], elbow[1], tip[1]],
         '-ok', lw=2, label='arm')
plt.scatter(*tip, color='blue', zorder=5)
span = l1 + l2 + 0.05
plt.xlim(-span, span)
plt.ylim(-span, span)
plt.gca().set_aspect('equal')
plt.grid(True)
plt.title('Propagated Cartesian uncertainty (95%)')
plt.legend()
plt.show()

# ---------- print matrices ----------
print("Σ_q  (rad²):\n", np.round(Sigma_q, 6))
print("\nΣ_x  (m²):\n", np.round(Sigma_x, 8))
print(f"\nMajor/minor axis lengths (95%):  {width:.4f} m  /  {height:.4f} m")
