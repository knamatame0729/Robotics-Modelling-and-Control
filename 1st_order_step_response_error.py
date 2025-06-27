import numpy as np
import matplotlib.pyplot as plt

# Time vector
t = np.linspace(0, 5, 501)

# Step reference
r = 1.0

# System parameters
K = 1.0
taus = [0.5, 1.0, 2.0] # time constants to compare

plt.figure(figsize=(8, 5))
for tau in taus:
    # output and error for step input
    y = K * (1 - np.exp(-t / tau))
    e = r -y

    plt.plot(t, y, label=f'$y(t),\\ \\tau={tau}$')
    plt.plot(t, e, '--', label=f'$e(t),\\ \\tau={tau}$')

plt.title('Fisrt-Order Step Response and Error')
plt.xlabel('Time [s]')
plt.ylabel('Output / Error')
plt.legend(loc='best')
plt.show()

