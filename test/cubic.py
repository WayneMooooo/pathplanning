import matplotlib.pyplot as plt
import numpy as np

# x = np.arange(5)
# y = [1.7, -6, 5, 6.5, 0.0]
# xi = np.linspace(0.0, 5.0)
x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]
a = np.loadtxt("C:/Users/MoWeimin/Desktop/C++2Python/cubic.txt")
plt.subplots(1)
plt.plot(x, y, "xb", label="Data points")
plt.plot(a[:, 0], a[:, 1], "r", label="Cubic spline interpolation")
plt.grid(True)
plt.legend()

s = np.loadtxt("C:/Users/MoWeimin/Desktop/C++2Python/cubic_yaw.txt")
plt.subplots(1)
plt.plot(s[:, 0], [np.rad2deg(iyaw) for iyaw in s[:, 1]], "-r", label="yaw")
plt.grid(True)
plt.legend()
plt.xlabel("line length[m]")
plt.ylabel("yaw angle[deg]")

plt.show()
