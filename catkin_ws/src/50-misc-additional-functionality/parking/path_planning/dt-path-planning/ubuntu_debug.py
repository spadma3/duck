#! /usr/bin/python
# -*- coding: utf-8 -*-

import time
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    plt.ion()
    x = np.arange(128)
    fig, ax = plt.subplots()
    ax.hold(False)

    for it in range(5):
        ax.plot(x, x+it)
        fig.canvas.flush_events()
        time.sleep(0.1)

    plt.ioff()
