import numpy as np
import matplotlib.pyplot as plt

# Arm Parameters
g = 9.81
l1, l2, l3 = 0.40, 0.30, 0.20
m1, m2, m3 = 2.0, 1.5, 1.0

# Gravity torque
def gravity_torque(q1, q2, q3):
    """
    Return (tau1, tau2, tau3) in N·m to hold the arm static
    """
    c1 = np.cos(q1)
    c12 = np.cos(q1+q2)
    c123 = np.cos(q1+q2+q3)

    tau1 = g*((m1*l1/2 + m2*l1 + m3*l1)*c1 + (m2*l2/2 + m3*l2)*c12 + m3*l3/2 * c123)

    tau2 = g*((m2*l2/2 + m3*l2)*c12 + m3*l3/2 * c123)

    tau3 = g*(m3*l3/2 * c123)

    return tau1, tau2, tau3

# Forward Kinematics
def joint_positions(q1, q2, q3):
    """
    Return posisitons of base, joint2, joint3, and end-effector
    """
    p0 = np.array([0, 0])
    p1 = p0 + l1*np.array([np.cos(q1), np.sin(q1)])
    p2 = p1 + l2*np.array([np.cos(q1+q2), np.sin(q1+q2)])
    p3 = p2 + l3*np.array([np.cos(q1+q2+q3), np.sin(q1+q2+q3)])

    return p0, p1, p2, p3

# Draw a pose
def draw_arm(q1_deg, q2_deg, q3_deg):
    q1, q2, q3 = np.deg2rad([q1_deg, q2_deg, q3_deg])
    p0, p1, p2, p3 = joint_positions(q1, q2, q3)
    tau1, tau2, tau3 = gravity_torque(q1, q2, q3)

    # Plot Arm
    plt.figure(figsize=(6,6))
    plt.plot(*zip(p0, p1, p2, p3), '-o', lw=4, ms=8)
    plt.xlim(-1.0, 1.0); plt.ylim(-1.5, 1.5)
    plt.gca().set_aspect('equal')
    plt.title('3-R planar arm - gravity-compensation torques')
    plt.grid(True)

    # Draw torque arrows (scaled)
    scale = 0.02
    for pj, tau in zip([p0, p1, p2], [tau1, tau2, tau3]):
        direction = np.array([0, 1]) if tau >= 0 else np.array([0, -1])
        plt.arrow(pj[0], pj[1],
                direction[0]*0.0001, direction[1]*scale*abs(tau),
                head_width=0.02, length_includes_head = True,
                linestyle='-', linewidth=1.5, alpha=0.8)

    plt.show()

    # Print torque values
    print(f"τ₁ = {tau1:+.2f} N·m   τ₂ = {tau2:+.2f} N·m   τ₃ = {tau3:+.2f} N·m")

# Run with fixed joint angles
draw_arm(q1_deg=30, q2_deg=-45, q3_deg=20)

