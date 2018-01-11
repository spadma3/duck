#! /usr/bin/python
# -*- coding: utf-8 -*-

import math, time, pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def plot_line(ax, x, y):
    ax.plot(x ,y)

def plot_rectangle(ax, x, y, dx, dy):
    ax.add_patch( patches.Rectangle((x,y),dx,dy))


if __name__ == '__main__':

    x = np.linspace(0.0,2.0*math.pi,100)
    y = np.sin(x)

    ax = plt.subplot(111)
    plt.plot(x,y)
    pickle.dump(ax, file('images/background.pickle', 'w'))

    for i in range(5):
        ax = pickle.load(file('images/background.pickle'))
        plot_rectangle(ax,math.pi/2.0+math.pi/10.0*i,0.2,math.pi,0.3)
        plt.pause(0.2)
