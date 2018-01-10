#!/usr/bin/python
# -*- coding: utf-8 -*-

import random
import math
import copy
import numpy as np
import dubins_path_planning
import rrt_star_car


print("Start rrt start planning")
import matplotlib.pyplot as plt
fig = plt.figure()

# ====Search Path with RRT====
obstacleList = [
    ("circle", 5, 5, 1),
    ("circle", 3, 6, 2),
    ("circle", 3, 8, 2),
    ("circle", 3, 10, 2),
    ("circle", 7, 5, 2),
    ("circle", 9, 5, 2),
    ("rectangle", 1, 1, 2, 1),
]  # [x,y,size(radius)]

curvature = 1

# Set Initial parameters
start = [0.0, 0.0, math.radians(0.0)]
goal = [10.0, 10.0, math.radians(0.0)]

rrt = rrt_star_car.RRT(start, goal, randArea=[-2.0, 15.0], obstacleList=obstacleList,
maxIter=50, fig=fig, curvature=curvature, radius_graph_refinement=0.5)
path = rrt.Planning(animation=True)

# start, goal, obstacleList, randArea, goalSampleRate=10, maxIter=1000
# Draw final path
rrt.DrawGraph()
plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
plt.grid(True)
plt.pause(0.001)
plt.show()
