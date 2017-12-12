#! /usr/bin/python
# -*- coding: utf-8 -*-

from dubins_path_planning import *
from math import sin, cos, sqrt, degrees, radians
import matplotlib.pyplot as plt


if __name__ == '__main__':
    print("snippets")

    start_x = 0.0  # [m]
    start_y = 0.0  # [m]
    start_yaw = math.radians(0.0)  # [rad]
    sx, sy, syaw = start_x, start_y, start_yaw

    end_x = -1.0  # [m]
    end_y = 1.5  # [m]
    end_yaw = math.radians(90.0)  # [rad]
    ex, ey, eyaw = end_x, end_y, end_yaw

    curvature = 0.25
    c = curvature
    
    mode = ["L","S","R"]
    t = radians(45)
    p = 1
    q = radians(-90)
    clen = (abs(t) + abs(p) + abs(q))
    
    lpx, lpy, lpyaw = generate_course([t, p, q], mode, c)

    # convert back to start coordinate system
    px = [math.cos(-syaw) * x + math.sin(-syaw) *
          y + sx for x, y in zip(lpx, lpy)]
    py = [- math.sin(-syaw) * x + math.cos(-syaw) *
          y + sy for x, y in zip(lpx, lpy)]
    pyaw = [pi_2_pi(iyaw + syaw) for iyaw in lpyaw]


    plt.plot(px, py, label="final course " + "".join(mode))
    
    # plotting
    plot_arrow(start_x, start_y, start_yaw)
    plot_arrow(end_x, end_y, end_yaw)
    
    #  for (ix, iy, iyaw) in zip(px, py, pyaw):
    #  plot_arrow(ix, iy, iyaw, fc="b")
    
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()

