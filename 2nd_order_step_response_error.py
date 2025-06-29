import numpy as np
from scipy.signal import TransferFunction, step
import matplotlib.pyplot as plt

# First-order example: K=2, tau=0.5
G1 = TransferFunction([2], [0.5, 1])
t, y = step(G1)
plt.plot(t, y)
plt.title('Step Response: G1(s)=2/(0.5s+!)')
plt.xlabel('Time [s]')
plt.ylabel('Output')
plt.grid(True)
plt.show()

# Second-order example: m=1, b=0.8, k=4
m, b, k = 1.0, 0.8, 4.0
omega_n = np.sqrt(k/m)
zeta = b / (2 * np.sqrt(m * k))
G2 = TransferFunction([omega_n**2], [1, 2*zeta*omega_n, omega_n**2])
t, y = step(G2)
plt.plot(t, y)
plt.title(f'Step Response: 2nd-order (zeta={zeta:2f})')
plt.xlabel('Time(s)')
plt.ylabel('Output')
plt.grid(True)
plt.show()

