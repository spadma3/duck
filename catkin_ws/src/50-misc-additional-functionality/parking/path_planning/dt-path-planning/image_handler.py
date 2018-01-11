#! /usr/bin/python
# -*- coding: utf-8 -*-

import math, time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.axes as axes

def plot_line(ax, x, y):
    ax.plot(x ,y)

def plot_rectangle(ax, x, y, dx, dy):
    ax.add_patch( patches.Rectangle((x,y),dx,dy))


if __name__ == '__main__':

    x = np.linspace(0.0,2.0*math.pi,100)
    y = np.sin(x)

    for i in range(5):
        fig = plt.figure(1)
        ax = fig.add_subplot(111)
        plt.cla()
        plot_line(ax,x,y)
        plot_rectangle(ax,math.pi/2.0+math.pi/10.0*i,0.2,math.pi,0.3)
        plt.pause(0.001)
        time.sleep(0.2)
