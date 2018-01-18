#! /usr/bin/python
# -*- coding: utf-8 -*-

import time
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    plt.ion()
    fig, ax = plt.subplots()
    plt.show()
    x = np.arange(128)

    for it in range(5):
        ax.plot(x, x+it)
        time.sleep(0.1)

    plt.ioff()
