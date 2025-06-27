import numpy as np
from scipy.signal import TransferFunction, step
import matplotlib.pyplot as plt

# scipy.singal.TransferFunction → Transfer Function
# G1 : First-oder System with gain = 1 and time constant = 2
G1 = TransferFunction([1], [1, 1])

# G2 : First-oder System with Gain = 2 and time constant = 2
G2 = TransferFunction([2], [2, 1])

# H : Unity Feedback Transfer Function (gain = 1)
H = TransferFunction([1], [1])

# Series connection: G_series(s) =  G2(s) * G1(s)
num_series = np.convolve(G1.num, G2.num)
den_series = np.convolve(G1.den, G2.den)
G_series = TransferFunction(num_series, den_series)

# Parallel connection: G_parallel(s) = G1(s) + G2(s)
num_parallel = np.polyadd(
        np.convolve(G1.num, G2.den),
        np.convolve(G2.num, G1.den)
 )
den_parallel = np.convolve(G1.den, G2.den)
G_parallel = TransferFunction(num_parallel, den_parallel)

# Closed-loop (unity feedback) connection:
# G_closed(s) = G_series(s) / [1 + G_series(s)*H(s)]
num_closed = G_series.num
den_closed = np.polyadd(G_series.den, G_series.num)
G_closed = TransferFunction(num_closed, den_closed)

# Plot step responses for each configuration
plt.figure()
t, y = step(G_series)
plt.plot(t, y)
plt.grid(True)
plt.title('Step Response: Series (G2 x G1)')

plt.figure()
t, y = step(G_parallel)
plt.plot(t, y)
plt.grid(True)
plt.title('Step Response: Parallel (G1 + G2)')

plt.figure()
t, y = step(G_closed)
plt.plot(t, y)
plt.grid(True)
plt.title('Step Response: Close-Loop (Unity Feedback)')

plt.show()

