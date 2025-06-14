import numpy as np
import matplotlib.pyplot as plt

#　Link lengths
l1, l2 = 0.6, 0.35

# FK & analytic Jacobian
def fk_pos(q):
    q1, q2 = q
    return np.array([l1*np.cos(q1)+l2*np.cos(q1+q2),
                     l1*np.sin(q1)+l2*np.sin(q1+q2)])

def jacobian(q):
    q1, q2 = q
    J11 = -l1*np.sin(q1) - l2*np.sin(q1+q2)
    J12 = -l2*np.sin(q1+q2)
    J21 =  l1*np.cos(q1) + l2*np.cos(q1+q2)
    J22 =  l2*np.cos(q1+q2)
    return np.array([[J11, J12],
                     [J21, J22]])

# Joint values
q1, q2 = 0.5, -0.5
q = np.array([q1, q2])
J = jacobian(q)

# singular values & manipulability
U, s, VT = np.linalg.svd(J, full_matrices=False)
manipulability = np.prod(s)
cond = s[0]/s[1] if s[1] > 1e-9 else np.inf

# ellipse points
theta = np.linspace(0, 2*np.pi, 200)
unit_circle = np.vstack((np.cos(theta), np.sin(theta)))
ellipse = (J @ unit_circle).T

# plot
tip = fk_pos(q)
plt.figure(figsize=(5,5))
plt.plot(ellipse[:,0]+tip[0], ellipse[:,1]+tip[1], 'b')
plt.scatter(*tip, color='red', zorder=5, label='End effector')
plt.gca().set_aspect('equal')
plt.grid(True)
span = l1 + l2 + 0.05
plt.xlim(-span, span)
plt.ylim(-span, span)
plt.title('Manipulability ellipse at tip')
plt.legend()

# print info
print(f"Joint values: q1 = {q1:.2f}, q2 = {q2:.2f}")
print(f"Singular values σ₁,σ₂  : {s.round(4)}")
print(f"Manipulability w(q)    : {manipulability:.5f}")
print(f"Condition number κ(J)  : {cond:.3f}")
if cond > 1e3:
    print('Near singularity!')

plt.show()
