#! /usr/bin/python
# -*- coding: utf-8 -*-

import math, time, pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def init():
    x = np.linspace(0.0,2.0*math.pi,100)
    y = np.sin(x)
    fig = plt.figure(1)
    ax = fig.add_subplot(111)
    ax.plot(x,y)
    pickle.dump(ax, file('images/background.pickle', 'w'))
    time.sleep(0.2)
    # plt.pause(0.2)
    plt.close()
    return x,y

def loop(x,y):
    for i in range(5):
        ax = pickle.load(file('images/background.pickle'))
        ax.plot(x+i/10.0,y)
        time.sleep(0.2)
        # plt.pause(0.2)
        plt.close()

if __name__ == '__main__':
    plt.ion()
    x,y = init()
    loop(x,y)
    plt.show()
    plt.ioff()
