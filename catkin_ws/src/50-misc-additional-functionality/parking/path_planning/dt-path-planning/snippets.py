#! /usr/bin/python
# -*- coding: utf-8 -*-

import matplotlib
matplotlib.use("wx")
from pylab import *
import matplotlib.pyplot as plt

fig, ax = plt.subplots()
mngr = plt.get_current_fig_manager()

# to put it into the upper left corner for example:
mngr.window.setGeometry(50,100,640, 545)
