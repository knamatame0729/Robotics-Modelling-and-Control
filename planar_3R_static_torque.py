import numpy as np
import matplotlib.pyplot as plt

# Link lengths
l1 = l2 = l3 = 1.0

# Fixed Parameters
q1_deg = 45
q2_deg = 20
q3_deg = -30
Fmag = 50
Fdir_deg = 250

q = np.deg2rad([q1_deg, q2_deg, q3_deg])

# Joint Positions
p0 = np.array([0.0, 0.0])
p1 = p0 + l1 * np.array([np.cos(q[0]), np.sin(q[0])])
p2 = p1 + l2 * np.array([np.cos(q[0] + q[1]), np.sin(q[0] + q[1])])
p3 = p2 + l3 * np.array([np.cos(q[0] + q[1] + q[2]), np.sin(q[0] + q[1] + q[2])])

# External force
th = np.deg2rad(Fdir_deg)
F = np.array([Fmag * np.cos(th), Fmag * np.sin(th)])

# Compute torques: tau_i = r_i x F
r1 = p3 - p0
r2 = p3 - p1
r3 = p3 - p2

tau1 = np.cross(r1, F)
tau2 = np.cross(r2, F)
tau3 = np.cross(r3, F)

print(f"q₁ = {q1_deg}°, q₂ = {q2_deg}°, q₃ = {q3_deg}°")
print(f"|F| = {Fmag} N, F dir = {Fdir_deg}°\n")
print("=== Results  ===")
print(f"τ₁ = {tau1:.2f} Nm")
print(f"τ₂ = {tau2:.2f} Nm")
print(f"τ₃ = {tau3:.2f} Nm")

# Plot
fig, ax = plt.subplots(figsize=(5, 5))
plt.plot([p0[0], p1[0]], [p0[1], p1[1]], '-o', lw=2, label='Link 1')
plt.plot([p1[0], p2[0]], [p1[1], p2[1]], '-o', lw=2, label='Link 2')
plt.plot([p2[0], p3[0]], [p2[1], p3[1]], '-o', lw=2, label='Link 3')
plt.arrow(p3[0], p3[1], F[0]/50, F[1]/50, head_width=0.05, head_length=0.1,
          color='red', length_includes_head=True, label='External Force')
plt.title("3‑R Planar Manipulator")
plt.xlabel("X")
plt.ylabel("Y")
plt.xlim(-3.5, 3.5)
plt.ylim(-3.5, 3.5)
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box')
plt.legend()
plt.show()
