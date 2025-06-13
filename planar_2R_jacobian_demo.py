import numpy as np

# Link lengths
l1, l2 = 0.6, 0.35

# Forward Kinematics (position only)
def fk_pos(q):
    q1, q2 = q
    x = l1*np.cos(q1) + l2*np.cos(q1+q2)
    y = l1*np.sin(q1) + l2*np.sin(q1+q2)
    return np.array([x, y])

# Analytic Jacobian
def jacobian_analytic(q):
    q1, q2 = q
    J11 = -l1*np.sin(q1) - l2*np.sin(q1+q2)
    J12 = -l2*np.sin(q1+q2)
    J21 =  l1*np.cos(q1) + l2*np.cos(q1+q2)
    J22 =  l2*np.cos(q1+q2)
    return np.array([[J11, J12],
                     [J21, J22]])

# Finite-difference Jacobian (numeric)
def jacobian_fd(q, eps=1e-6):

    J = np.zeros((2, 2))         # Initialize the Jacobian matrix (2x2)
    f0 = fk_pos(q)               # Evaluate the end-effector position at the current joint angles (reference)

    # Loop over each joint angle to compute partial derivatives numerically
    for i in range(2):
        dq = np.zeros(2)         # Create a zero perturbation vector
        dq[i] = eps              # Perturb only the i-th joint by a small amount (eps)

        f_eps = fk_pos(q + dq)   # Evaluate forward kinematics at the perturbed joint angle
        J[:, i] = (f_eps - f0) / eps  # Compute finite difference approximation for the i-th column

    return J

# test at a random configuration
q_test = np.deg2rad([25.0, -40.0])

J_an = jacobian_analytic(q_test)
J_fd = jacobian_fd(q_test)

print("Analytic Jacobian:\n", np.round(J_an, 5))
print("\nFinite-difference Jacobian:\n", np.round(J_fd, 5))
print("\nΔ (analytic - FD):\n", np.round(J_an - J_fd, 8))