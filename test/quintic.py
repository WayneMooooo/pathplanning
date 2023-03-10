import matplotlib.pyplot as plt
import numpy as np

plt.cla()
plt.grid(True)
plt.axis("equal")
a = np.loadtxt("C:/Users/Moweimin/Desktop/C++2Python/quintic.txt")
plt.plot(a[:, 0], a[:, 1], "-r")
plt.grid(True)
plt.show()
