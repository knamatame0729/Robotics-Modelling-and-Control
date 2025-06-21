import sympy as sp
import numpy as np
import time
import math
sp.init_printing()

# SYMBOLIC MODEL
# Joint angles, velocities, acceleraions (time-dependent)
q1, q2, q3 = sp.symbols('q1 q2 q3')
qd1, qd2, qd3 = sp.symbols('qd1 qd2 qd3')
qdd1, qdd2, qdd3 = sp.symbols('qdd1 qdd2 qdd3')

# Kinematic parameters (link lengths, masses, inertias)
l1, l2, l3, m1, m2, m3, g = sp.symbols('l1 l2 l3 m1 m2 m3 g', positive=True)

# Forward kinematics: CoM positions (mid-link points for brevity)
x1 = (l1/2)*sp.cos(q1)
y1 = (l2/2)*sp.sin(q1)

x2 = l1*sp.cos(q1) + (l2/2)*sp.cos(q1+q2)
y2 = l1*sp.sin(q1) + (l2/2)*sp.sin(q1+q2)

x3 = l1*sp.cos(q1) + l2*sp.cos(q1+q2) + (l3/2)*sp.cos(q1+q2+q3)
y3 = l1*sp.sin(q1) + l2*sp.sin(q1+q2) + (l3/2)*sp.sin(q1+q2+q3)

# Velocities via Jacobian
J1 = sp.Matrix([[sp.diff(x1, q1), 0, 0],
                [sp.diff(y1, q1), 0, 0]])
J2 = sp.Matrix([[sp.diff(x2, q1), sp.diff(x2, q2), 0],
                [sp.diff(y2, q1), sp.diff(y2, q2), 0]])
J3 = sp.Matrix([[sp.diff(x3, q1), sp.diff(x2, q2), sp.diff(x3, q3)],
                [sp.diff(y3, q1), sp.diff(y3, q2), sp.diff(y3, q3)]])

qdot = sp.Matrix([qd1, qd2, qd3])

v1 = J1*qdot
v2 = J2*qdot
v3 = J3*qdot

# Kinetic & potential energies (planar → ignore link rotational inertia for speed)
KE = (m1/2) * (v1.T*v1)[0] + (m2/2) * (v2.T*v2)[0] + (m3/2) * (v3.T*v3)[0]
PE = m1 * g * y1 + m2 * g * y2 + m3 * g * y3
Lag = sp.simplify(KE-PE)

# Euler-Lagrange
q = (q1, q2, q3)
qd = (qd1, qd2, qd3)
qdd = (qdd1, qdd2, qdd3)
tau_sym = []
for qi,qdi in zip(q,qd):
    dLdq = sp.diff(Lag, qi)
    dLdqd = sp.diff(Lag, qdi)
    ddt_dLdqd = sum([sp.diff(dLdqd, qj)*qdk for qj, qdk in zip (q,qd)]) \
              + sum([sp.diff(dLdqd, qdj)*qddk for qdj, qddk in zip (qd,qdd)])
    tau_sym.append(sp.simplify(ddt_dLdqd - dLdq))
tau_sym = sp.Matrix(tau_sym)

# Split into M(q), C(q, qd), G(q)
# tau = M*qdd + C + G (collect qdd linearly)
M_sym = tau_sym.jacobian(qdd)
CplusG = sp.simplify(tau_sym - M_sym*sp.Matrix(qdd))

# Separate gravity term and C(the rest)
G_sym = sp.Matrix([sp.expand(expr).coeff(g)*g for expr in CplusG])
C_sym = sp.simplify(CplusG - G_sym)

print("Symbolic Deriviation Complete.")

# Lambdify for fast numeric evaluation
params = (l1, l2, l3, m1, m2, m3, g, q1, q2, q3, qd1, qd2, qd3)
M_func = sp.lambdify(params, M_sym, 'numpy')
C_func = sp.lambdify(params, C_sym, 'numpy')
G_func = sp.lambdify(params, G_sym, 'numpy')

# RNEA (planar, point-mass model, no link inertia for speed)
def rnea_numeric(q, qd, qdd, p):
    """ Recursive Newton-Euler for planar 3R, point masses,
    q, qd, qdd = (3,) arrays; p tuple=(l1, l2, l3, m1, m2, m, g)
    """
    l1, l2, l3, m1, m2, m3, g = p
    
    # Forward pass: angular vel/acc
    w1, w2, w3 = qd
    wd1, wd2, wd3 = qdd
    # Linear accelerations of CoM (planar, using simple formulas)
    ac1 = np.array([
        - (l1/2)*w1**2*np.cos(q[0]) + (l1/2)*wd1*np.sin(q[0]),
        - (l1/2)*w1**2*np.sin(q[0]) - (l1/2)*wd1*np.cos(q[0]) - g])
    ac2 = np.array([
        - l1*w1**2*np.cos(q[0]) - l1*wd1*np.sin(q[0]) \
        - (l2/2)*w2**2*np.cos(q[0]+q[1]) + (l2/2)*wd2*np.sin(q[0]+q[1]),
        - l1*w1**2*np.sin(q[0]) + l1*wd1*np.cos(q[0]) \
        - (l2/2)*w2**2*np.sin(q[0]+q[1]) - (l2/2)*wd2*np.cos(q[0]+q[1]) - g])
    ac3 = np.array([
        - l1*w1**2*np.cos(q[0])    - l1*wd1*np.sin(q[0]) \
        - l2*w2**2*np.cos(q[0]+q[1]) - l2*wd2*np.sin(q[0]+q[1]) \
        - (l3/2)*w3**2*np.cos(q[0]+q[1]+q[2]) + (l3/2)*wd3*np.sin(q[0]+q[1]+q[2]),
        - l1*w1**2*np.sin(q[0])    + l1*wd1*np.cos(q[0]) \
        - l2*w2**2*np.sin(q[0]+q[1]) + l2*wd2*np.cos(q[0]+q[1]) \
        - (l3/2)*w3**2*np.sin(q[0]+q[1]+q[2]) - (l3/2)*wd3*np.cos(q[0]+q[1]+q[2]) - g])

    # Backward pass: forces & torques
    F3 = m3 * ac3
    tau3 = np.cross([(l3/2)*np.cos(sum(q)), (l3/2)*np.sin(sum(q)), 0],
                    np.append(F3,0))[2]

    F2 = m2*ac2 + F3
    tau2 = np.cross([(l2/2)*np.cos(q[0]+q[1]), l2/2*np.sin(q[0]+q[1]), 0],
                    np.append(m2*ac2,0))[2] \
         + np.cross([l2*np.cos(q[0]+q[1]), l2*np.sin(q[0]+q[1]), 0],
                    np.append(F3, 0))[2] + tau3

    F1 = m1*ac1 + F2
    tau1 = np.cross([(l1/2)*np.cos(q[0]), l1/2*np.sin(q[0]), 0],
                     np.append(m1*ac1,0))[2] \
         + np.cross([l1*np.cos(q[0]), l1*np.sin(q[0]), 0],
                    np.append(F2,0))[2] + tau2

    return np.array([tau1, tau2, tau3])

def bench(n_samples=1000):
    # Generate Samples
    rng = np.random.default_rng(0)
    Q = rng.uniform(-np.pi, np.pi, (n_samples,3))
    QD = rng.uniform(-2.0, 2.0, (n_samples,3))
    QDD = rng.uniform(-3.0, 3.0, (n_samples,3))

    p_vals = (1.0, 1.0, 1.0, 1.0, 10.0, 5.0, 9.81)

    # Lagrange
    t0 = time.perf_counter()
    for i in range(n_samples):
        q = Q[i]; qd = QD[i]
        args = (*p_vals, *q, *qd)
        M = np.array(M_func(*args), dtype=float)
        C = np.array(C_func(*args), dtype=float).flatten()
        G = np.array(G_func(*args), dtype=float).flatten()
        tau = M.dot(QDD[i]) + C + G
    lag_time = time.perf_counter() - t0

    # RNEA
    t0 = time.perf_counter()
    for i in range(n_samples):
        tau_rnea = rnea_numeric(Q[i], QD[i], QDD[i], p_vals)
    rnea_time = time.perf_counter() - t0

    # Results
    print(f"Samples: {n_samples}")
    print(f"Lagrange (M,C,G) evaluation: {lag_time*1e3:.2f} ms")
    print(f"RNEA numeric: {rnea_time*1e3:.2f} ms")

bench(500)
