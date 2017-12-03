#!/usr/bin/env python

import networkx as nx
import matplotlib.pyplot as plt

G = nx.drawing.nx_agraph.read_dot("winning.dot")
pos = nx.get_node_attributes(G,'pos')

#TODO: order pos.values by factor number!
vals = list(pos.values())
vals = [m.split(",") for m in vals]
vals = [[float(m[0]), float(m[1][:-1])] for m in vals]
x = [m[0] for m in vals]
y = [m[1] for m in vals]
plt.scatter(x,y)
for i,(a,b) in enumerate(zip(x, y)):
    plt.text(a, b, i)

#nx.draw(G)
plt.show()
