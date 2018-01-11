#! /usr/bin/python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import pickle

ax = plt.subplot(111)
x = np.linspace(0, 10)
y = np.exp(x)
plt.plot(x, y)
pickle.dump(ax, file('background.pickle', 'w'))

plt.clf()

ax = pickle.load(file('background.pickle'))
plt.pause(0.5)
