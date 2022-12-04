import math
import numpy as np
import matplotlib.pyplot as plt


dis_arr = np.load("/home/dklee98/git/term_ws/src/mpc_casadi/dis.npy")
yaw_arr = np.load("/home/dklee98/git/term_ws/src/mpc_casadi/yaw.npy")

plt.style.use('default')
plt.rcParams['figure.figsize'] = (4, 3)
plt.rcParams['font.size'] = 12

fig, ax = plt.subplots()

t = []
for n in yaw_arr:
    n = n * 180 / np.pi
    if n < -180:
        n = n + 180
    elif n > -180 and n < 0:
        n = n + 180
    elif n > 180:
        n = n - 180
    elif n < 180 and n > 0:
        n = 180 - n
    print(n)
    t.append(n)

ax.boxplot([np.array(t), dis_arr])
# ax.boxplot([dis_arr, yaw_arr])
# ax.set_ylim(-10.0, 10.0)
ax.set_xlabel('Data Type')
ax.set_ylabel('Value')

plt.show()