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


