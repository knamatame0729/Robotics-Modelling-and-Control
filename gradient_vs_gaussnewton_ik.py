import numpy as np
import matplotlib.pyplot as plt
import time

# Link lengths
link_len = np.array([0.2, 0.3, 0.2])

# Forward Kinematics (position only)
def fk(q):
    s = np.sin(np.cumsum(q)); c = np.cos(np.cumsum(q))
    x = np.sum(link_len * c)
    y = np.sum(link_len * s)
    return np.array([x, y])

# Analytic Jacobian (2x3)
def jacobian(q):
    c = np.cos(np.cumsum(q)); s = np.sin(np.cumsum(q))
    J = np.zeros((2,3))
    for i in range(3):
        dx = -np.sum(link_len[i:] * s[i:])
        dy = np.sum(link_len[i:] * c[i:])
        J[:, i] = [dx, dy]
    return J

# Optimisation loop
def ik_opt(target, q_init, method='gd', alpha=0.4, max_iter=300):
    q = q_init.copy()
    t0 = time.perf_counter()
    for k in range(max_iter):
        err = target - fk(q)
        if np.linalg.norm(err) < 1e-4:
            break
        J = jacobian(q)
        if method == 'gd':  # Gradient Descent Step
            q += alpha * J.T @ err
        else:              # Gauss-Newton step
            q += np.linalg.pinv(J) @ err
    t_elapsed = (time.perf_counter() - t0) * 1000 #ms
    return q, k+1, t_elapsed, np.linalg.norm(err)

# targets and initial guesses
target = np.array([0.55, 0.25])

q_init_close = np.deg2rad([10, -60, 5])   # Close initial guess
q_init_far = np.deg2rad([-170, 70, 135])  # Far initial guess

# Run experiments
results = {}
for label, q0 in [('close', q_init_close), ('far', q_init_far)]:
    results[f'GD_{label}'] = ik_opt(target, q0, 'gd') #Gradient Descent Results
    results[f'GN_{label}'] = ik_opt(target, q0, 'gn') #Gauss-Newton Results

# Plot arms
def plot_arm(q, style, label):
    pts = np.zeros((4,2))
    angle  = 0.0
    for i in range(3):
        angle += q[i]
        pts[i+1] = pts[i] + link_len[i]*np.array([np.cos(angle), np.sin(angle)])
    plt.plot(pts[:,0], pts[:,1], style, lw=2, label=label)

plt.figure(figsize=(6,6))
plt.gca().set_aspect('equal'); plt.grid(True)
plt.scatter(*target, color='red', zorder=5, label='target')

plot_arm(*results['GD_close'][:1], '-o', 'GD (close init)')
plot_arm(*results['GN_close'][:1], '-x',  'GN (close init)')
plot_arm(*results['GD_far'][:1],  '--o', 'GD (far init)')
plot_arm(*results['GN_far'][:1],  '--x', 'GN (far init)')

plt.title('IK paths – Gradient Descent vs Gauss–Newton')
plt.legend(); plt.show()

# Text Summary
print("Target position:", np.round(target,3), "m\n")
for key, (q_sol, iters, t_ms, err) in results.items():
    mth, init = key.split('_')
    print(f"{mth}  ({init} init):")
    print(f"  iterations : {iters}")
    print(f"  time       : {t_ms:.2f} ms")
    print(f"  final error: {err:.2e} m")
    print(f"  q (deg)    : {np.round(np.degrees(q_sol),1)}\n")


            
