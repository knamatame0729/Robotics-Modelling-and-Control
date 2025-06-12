import numpy as np
import matplotlib.pyplot as plt

# Define the position vector o
o = np.array([1.0, 2.0, 3.0])  # [ox, oy, oz]

# Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.quiver(0, 0, 0, o[0], o[1], o[2], color='r', arrow_length_ratio=0.1, label='Position vector o')
ax.set_xlim([0, 3])
ax.set_ylim([0, 3])
ax.set_zlim([0, 3])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.title('Position of a Point in 3D Space')
plt.show()