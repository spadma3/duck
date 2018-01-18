#! /usr/bin/python
# -*- coding: utf-8 -*-

import math, time, pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

if __name__ == '__main__':
    plt.ion()

    x = np.linspace(0.0,2.0*math.pi,100)
    y = np.sin(x)

    fig = plt.figure(1)
    ax = fig.add_subplot(111)
    ax.plot(x,y)
    fig.canvas.flush_events()
    time.sleep(0.2)
    pickle.dump(ax, file('images/myplot.pickle', 'w'))

    for i in range(5):
        # ax = pickle.load(file('images/myplot.pickle'))
        ax.plot(x+i/10.0,y)
        fig.canvas.flush_events()
        time.sleep(0.2)

    plt.ioff()
