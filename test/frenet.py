import matplotlib.pyplot as plt
import numpy as np

MAX_ROAD_WIDTH = 7.0
ob = np.array([[20.0, 10.0],
               [30.0, 6.0],
               [30.0, 8.0],
               [35.0, 8.0],
               [50.0, 3.0]
               ])

course = np.loadtxt("C:/Users/MoWeimin/Desktop/C++2Python/frenet_course.txt")
path = np.loadtxt("C:/Users/MoWeimin/Desktop/C++2Python/frenet_path.txt")
plt.cla()
plt.plot(course[:, 0], course[:, 1], "--k")
plt.plot(path[:, 0], path[:, 1])
plt.plot(ob[:, 0], ob[:, 1], "xk")
yup = [t + MAX_ROAD_WIDTH for t in course[:, 1]]
ydown = [t - MAX_ROAD_WIDTH for t in course[:, 1]]
plt.plot(course[:, 0], yup, "-k")
plt.plot(course[:, 0], ydown, "-k")
plt.grid(True)
plt.show()
