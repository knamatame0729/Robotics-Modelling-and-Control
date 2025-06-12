import numpy as np
import matplotlib.pyplot as plt

# Define the rotation matrix R
R = np.array([[0.707, -0.707, 0],
              [0.707, 0.707, 0],
              [0, 0, 1]])

print("Rotatio matrix R:\n", R)

# Visualize the coordinate system
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the reference frame axes
ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='X')
ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y')
ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z')

# Plot the rotated frame axes
x_prime = np.dot(R, np.array([1, 0, 0]))
y_prime = np.dot(R, np.array([0, 1, 0]))
z_prime = np.dot(R, np.array([0, 0, 1]))

ax.quiver(0, 0, 0, x_prime[0], x_prime[1], x_prime[2], color='r', linestyle='--', label="X'")
ax.quiver(0, 0, 0, y_prime[0], y_prime[1], y_prime[2], color='g', linestyle='--', label="Y'")
ax.quiver(0, 0, 0, z_prime[0], z_prime[1], z_prime[2], color='b', linestyle='--', label="Z'")

ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_zlim([-1.5, 1.5])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.title('Orientation of a Rigid Body')
plt.show()