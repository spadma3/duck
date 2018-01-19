#! /usr/bin/python
# -*- coding: utf-8 -*-

"""
pickle
"""
import matplotlib.pyplot as plt
import numpy as np
import pickle, time
if False:
    ax = plt.subplot(111)
    x = np.linspace(0, 10)
    y = np.sin(x)
    ax.plot(x, y)
    pickle.dump(ax, file('images/myplot.pickle', 'w'))
else:
    ax = pickle.load(file('images/myplot.pickle'))
    # ax.plot(0,1,'k*')
    # plt.show()
    plt.pause(1)


"""
canvas flush_events
"""
# import time, pickle
# import matplotlib.pyplot as plt
# import numpy as np
#
# plt.ion()
# x = np.arange(128)
# fig, ax = plt.subplots()
# ax.hold(False)
#
# for it in range(5):
#
#     ax.plot(x, x+it)
#     fig.canvas.flush_events()
#     time.sleep(0.1)
#
# plt.ioff()


# from __future__ import print_function
#
# from six.moves import input
#
# import numpy as np
#
# from matplotlib.widgets import LassoSelector
# from matplotlib.path import Path
#
#
# class SelectFromCollection(object):
#     """Select indices from a matplotlib collection using `LassoSelector`.
#
#     Selected indices are saved in the `ind` attribute. This tool fades out the
#     points that are not part of the selection (i.e., reduces their alpha
#     values). If your collection has alpha < 1, this tool will permanently
#     alter the alpha values.
#     Note that this tool selects collection objects based on their *origins*
#
#     (i.e., `offsets`).
#
#     Parameters
#     ----------
#     ax : :class:`~matplotlib.axes.Axes`
#         Axes to interact with.
#
#     collection : :class:`matplotlib.collections.Collection` subclass
#         Collection you want to select from.
#
#     alpha_other : 0 <= float <= 1
#         To highlight a selection, this tool sets all selected points to an
#         alpha value of 1 and non-selected points to `alpha_other`.
#     """
#
#     def __init__(self, ax, collection, alpha_other=0.3):
#         self.canvas = ax.figure.canvas
#         self.collection = collection
#         self.alpha_other = alpha_other
#
#         self.xys = collection.get_offsets()
#         self.Npts = len(self.xys)

#
#         # Ensure that we have separate colors for each object
#         self.fc = collection.get_facecolors()
#         if len(self.fc) == 0:
#             raise ValueError('Collection must have a facecolor')
#         elif len(self.fc) == 1:
#             self.fc = np.tile(self.fc, (self.Npts, 1))
#
#         self.lasso = LassoSelector(ax, onselect=self.onselect)
#         self.ind = []
#
#     def onselect(self, verts):
#         path = Path(verts)
#         self.ind = np.nonzero(path.contains_points(self.xys))[0]
#         self.fc[:, -1] = self.alpha_other
#         self.fc[self.ind, -1] = 1
#         self.collection.set_facecolors(self.fc)
#         self.canvas.draw_idle()
#
#     def disconnect(self):
#         self.lasso.disconnect_events()
#         self.fc[:, -1] = 1
#         self.collection.set_facecolors(self.fc)
#         self.canvas.draw_idle()
#
#
# if __name__ == '__main__':
#     import matplotlib.pyplot as plt
#
#     plt.ion()
#     # Fixing random state for reproducibility
#     np.random.seed(19680801)
#
#     data = np.random.rand(100, 2)
#
#     subplot_kw = dict(xlim=(0, 1), ylim=(0, 1), autoscale_on=False)
#     fig, ax = plt.subplots(subplot_kw=subplot_kw)
#
#     pts = ax.scatter(data[:, 0], data[:, 1], s=80)
#     selector = SelectFromCollection(ax, pts)
#
#     plt.draw()
#     input('Press Enter to accept selected points')
#     print("Selected points:")
#     print(selector.xys[selector.ind])
#     selector.disconnect()
#
#     # Block end of script so you can check that the lasso is disconnected.
#     input('Press Enter to quit')

# import time
# import matplotlib.pyplot as plt
# import numpy as np
#
#
# def get_memory(t):
#     "Simulate a function that returns system memory"
#     return 100 * (0.5 + 0.5 * np.sin(0.5 * np.pi * t))
#
#
# def get_cpu(t):
#     "Simulate a function that returns cpu usage"
#     return 100 * (0.5 + 0.5 * np.sin(0.2 * np.pi * (t - 0.25)))
#
#
# def get_net(t):
#     "Simulate a function that returns network bandwidth"
#     return 100 * (0.5 + 0.5 * np.sin(0.7 * np.pi * (t - 0.1)))
#
#
# def get_stats(t):
#     return get_memory(t), get_cpu(t), get_net(t)
#
# fig, ax = plt.subplots()
# ind = np.arange(1, 4)
#
# # show the figure, but do not block
# plt.show(block=False)
#
#
# pm, pc, pn = plt.bar(ind, get_stats(0))
# pm.set_facecolor('r')
# pc.set_facecolor('g')
# pn.set_facecolor('b')
# ax.set_xticks(ind)
# ax.set_xticklabels(['Memory', 'CPU', 'Bandwidth'])
# ax.set_ylim([0, 100])
# ax.set_ylabel('Percent usage')
# ax.set_title('System Monitor')
#
# start = time.time()
# for i in range(200):  # run for a little while
#     m, c, n = get_stats(i / 10.0)
#
#     # update the animated artists
#     pm.set_height(m)
#     pc.set_height(c)
#     pn.set_height(n)
#
#     # ask the canvas to re-draw itself the next time it
#     # has a chance.
#     # For most of the GUI backends this adds an event to the queue
#     # of the GUI frameworks event loop.
#     fig.canvas.draw_idle()
#     try:
#         # make sure that the GUI framework has a chance to run its event loop
#         # and clear any GUI events.  This needs to be in a try/except block
#         # because the default implementation of this method is to raise
#         # NotImplementedError
#         fig.canvas.flush_events()
#     except NotImplementedError:
#         pass
#
# stop = time.time()
# print("{fps:.1f} frames per second".format(fps=200 / (stop - start)))
