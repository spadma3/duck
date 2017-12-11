#!/usr/bin/env python

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

path = np.loadtxt('path.txt')
x = path[:,0]
y = path[:,1]
theta = path[:,2]


mpl.rcParams['legend.fontsize'] = 10
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x, y, theta, label='parametric curve')
ax.legend()
plt.xlabel('x')
plt.ylabel('y')

plt.show()
