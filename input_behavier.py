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


