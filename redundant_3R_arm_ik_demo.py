import numpy as np
import matplotlib.pyplot as plt

# Link lengths
L = np.array([0.4, 0.3, 0.2])

def fk(q):
    s = np.sin(np.cumsum(q))
    c = np.cos(np.cumsum(q))
    return np.array([np.sum(L * c), np.sum(L * s)])

def jac(q):
    c = np.cos(np.cumsum(q))
    s = np.sin(np.cumsum(q))
    J = np.zeros((2,3))
    for i in range(3):
        J[:, i] = [-np.sum(L[i:] * s[i:]), np.sum(L[i:] * c[i:])]
    return J

def damped_pinv(J, lam=0.0):
    JJt = J @ J.T
    return J.T @ np.linalg.inv(JJt + (lam ** 2) * np.eye(JJt.shape[0]))

def gn_step(q, err, J, lam):
    return damped_pinv(J, lam) @ err

def gn_null_step(q, err, J, z, lam):
    Jpinv = damped_pinv(J, lam)
    null = (np.eye(3) - Jpinv @ J) @ z
    return Jpinv @ err + null

def posture_gradient(q):
    return -0.05 * q

def run_ik_demo(tx=0.5, ty=0.1, use_damping=False, use_null=False):
    target = np.array([tx, ty])
    q = np.deg2rad([10, -30, 20])
    lam = 0.1 if use_damping else 0.0
    traj = [q.copy()]

    for _ in range(200):
        err = target - fk(q)
        if np.linalg.norm(err) < 1e-4:
            break
        J = jac(q)
        step = (gn_null_step(q, err, J, posture_gradient(q), lam)
                if use_null else
                gn_step(q, err, J, lam))

        q += step
        traj.append(q.copy())

    # Plot
    plt.figure(figsize=(6,6))
    ax = plt.gca()
    ax.set_aspect('equal')
    ax.grid(True)
    ax.scatter(*target, color='red', s=70, label='target')

    span = L.sum() + 0.05
    ax.set_xlim(-span, span)
    ax.set_ylim(-span, span)

    # Arm trace
    pts_final = np.zeros((4,2))
    a = 0
    for i in range(3):
        a += q[i]
        pts_final[i + 1] = pts_final[i] + L[i] * np.array([np.cos(a), np.sin(a)])
    ax.plot(pts_final[:, 0], pts_final[:, 1], '-o', lw=3, label='final pose')

    title = "IK: "
    title += "damped " if use_damping else "undamped"
    title += "+ null-space" if use_null else ""
    ax.set_title(title)
    ax.legend()
    plt.show()

    print(f"iterations : {len(traj) - 1}")
    print(f"final error (m) : {np.linalg.norm(target - fk(q)):.3e}")
    print("final joint (deg) :", np.round(np.degrees(q), 1))

run_ik_demo(tx=0.5, ty=0.1, use_damping=True, use_null=True)

