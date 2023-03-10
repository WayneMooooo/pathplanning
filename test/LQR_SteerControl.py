import matplotlib.pyplot as plt
import numpy as np

a = np.loadtxt("C:/Users/Moweimin/Desktop/C++2Python/course.txt")
b = np.loadtxt("C:/Users/Moweimin/Desktop/C++2Python/lqr.txt")
ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
x, y, v, t = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
cx, cy, cyaw, ck, s = a[:, 0], a[:, 1], a[:, 2], a[:, 3], a[:, 4]
plt.close()
plt.subplots(1)
plt.plot(ax, ay, "xb", label="input")
plt.plot(cx, cy, "-r", label="spline")
plt.plot(x, y, "-g", label="tracking")
plt.grid(True)
plt.axis("equal")
plt.xlabel("x[m]")
plt.ylabel("y[m]")
plt.legend()
plt.subplots(1)
plt.plot(t, v, "xb", label="speed")
plt.xlabel("t[s]")
plt.ylabel("v[m/s]")

plt.show()
