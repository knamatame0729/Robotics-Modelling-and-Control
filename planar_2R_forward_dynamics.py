import numpy as np
import matplotlib.pyplot as plt

# Link Lengths and masses
L = [0.5, 0.4]
m = [1.0, 0.8]
GRAV = 9.81

def mass_matrix(q):
    q2 = q[1]
    a1, a2 = L
    m1, m2 = m
    I1 = m1*(a1/2)**2
    I2 = m2*(a2/2)**2
    M11 = I1 + I2 + m2*(a1**2 + (a2/2)**2 + 2*a1*(a2/2)*np.cos(q2))
    M12 = I2 + m2*((a2/2)**2 + a1*(a2/2)*np.cos(q2))
    M22 = I2

def gravity_vector(q):
    q1, q2 = q
    c1 = np.cos(q1)
    c12 = np.cos(q1 + q2)
    G1 = m[0]*GRAV*(L[0]/2)*c1 + m[1]*GRAV*(L[0]*c1 + (L[1]/2)*c12)
    G2 = m[1]*GRAV*(L[1]/2)*c12
    
    return np.array([G1, G2])

def forward_dynamics(q, tau):
    M = mass_matrix(q)
    Gv = gravity_vector(q)
    qdd = np.linalg.inv(M) @ (tau - Gv)
    
    return qdd

# Eigen Value
q_deg = [30, 20]
q = np.deg2rad(q_deg)
tau = np.array([0.0, 0.0])

# Compute
M = mass_matrix(q)
G_vec = gravity_vector(q)
add = forward_dynamics(q, tau)

# Results
print("Mass matrix M(q):")
print(np.round(M, 3))
print("\nGravity vector G(q):")
print(np.round(G_vec, 3))
print("\nJoint acceleraitons qdd = M^{-1}(tau - G)]")
print(np.round(qdd, 3))

# Visualize arm
a1, a2 = L
po = np.array([0.0, 0.0])
p1 = p0 + a1 * np.array([np.cos(q[0]), np.sin(q[0])])
p2 = p1 + a2 * np.array([np.cos(q[0]+q[1]), np.sin(q[0]+q[1])])

fig, ax = plt.subplots(figsize=(4,4))
ax.plot([p0[0], p1[0]], [p0[1], p1[1]], '-o', lw=3)
ax.plot([p1[0], p2[0]], [p1[1], p2[1]], '-o', lw=3)

ax.set_aspect('equal')
lim = sum(L) + 0.1
ax.set_xlim(-lim, lim)
az.set_ylim(-lim, lim)
ax.set_title(f'Arm config: q = {q_deg[0]:.0f}°, {q_deg[1]:.0f}°')
plt.show()
        

