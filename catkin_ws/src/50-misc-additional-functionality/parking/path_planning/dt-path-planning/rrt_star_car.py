#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@brief: Path Planning Sample Code with RRT for car like robot.

@author: AtsushiSakai(@Atsushi_twi)

@adapted: Samuel Nyffenegger (samueln@ethz.ch)

@license: MIT

"""

import random, time, math, copy, pickle
import numpy as np
import dubins_path_planning


class RRT():
    u"""
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea, fig, curvature,
                 radius_graph_refinement, goalSampleRate=10, maxIter=1000):
        u"""
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1], start[2])
        self.end = Node(goal[0], goal[1], goal[2])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.obstacleList = obstacleList
        self.curvature = curvature
        self.goalSampleRate = goalSampleRate
        self.radius_graph_refinement = radius_graph_refinement
        self.maxIter = maxIter
        self.fig = fig

    def Planning(self, animation=True):
        u"""
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            rnd = self.get_random_point()
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            newNode = self.steer(rnd, nind)
            #  print(newNode.cost)

            if self.CollisionCheck(newNode):
                nearinds = self.find_near_nodes(newNode)
                newNode = self.choose_parent(newNode, nearinds)
                self.nodeList.append(newNode)
                self.rewire(newNode, nearinds)

            if animation and i % 5 == 0:
                self.DrawGraph(rnd=rnd)

        # generate coruse
        lastIndex = self.get_best_last_index()
        #  print(lastIndex)
        path = self.gen_final_course(lastIndex)
        return path

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            tNode = self.steer(newNode, i)
            if self.CollisionCheck(tNode):
                dlist.append(tNode.cost)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return newNode

        newNode = self.steer(newNode, minind)

        return newNode

    def pi_2_pi(self, angle):
        while(angle >= math.pi):
            angle = angle - 2.0 * math.pi

        while(angle <= -math.pi):
            angle = angle + 2.0 * math.pi

        return angle

    def steer(self, rnd, nind):

        nearestNode = self.nodeList[nind]

        px, py, pyaw, mode, clen = dubins_path_planning.dubins_path_planning(
            nearestNode.x, nearestNode.y, nearestNode.yaw, rnd.x, rnd.y, rnd.yaw, self.curvature)

        newNode = copy.deepcopy(nearestNode)
        newNode.x = px[-1]
        newNode.y = py[-1]
        newNode.yaw = pyaw[-1]

        newNode.path_x = px
        newNode.path_y = py
        newNode.path_yaw = pyaw
        newNode.cost += clen
        newNode.parent = nind

        return newNode

    def get_random_point(self):

        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand),
                   random.uniform(-math.pi, math.pi)
                   ]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y, self.end.yaw]

        node = Node(rnd[0], rnd[1], rnd[2])

        return node

    def get_best_last_index(self):
        #  print("get_best_last_index")

        YAWTH = math.radians(1.0)

        XYTH = self.radius_graph_refinement

        goalinds = []
        for (i, node) in enumerate(self.nodeList):
            if self.calc_dist_to_goal(node.x, node.y) <= XYTH:
                goalinds.append(i)

        # angle check
        fgoalinds = []
        for i in goalinds:
            if abs(self.nodeList[i].yaw - self.end.yaw) <= YAWTH:
                fgoalinds.append(i)

        mincost = min([self.nodeList[i].cost for i in fgoalinds])
        for i in fgoalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
                path.append([ix, iy])
            #  path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode):
        nnode = len(self.nodeList)
        # r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        r = self.radius_graph_refinement
        #  r = self.expandDis * 5.0
        dlist = [(node.x - newNode.x) ** 2 +
                 (node.y - newNode.y) ** 2 +
                 (node.yaw - newNode.yaw) ** 2
                 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def rewire(self, newNode, nearinds):

        nnode = len(self.nodeList)

        for i in nearinds:
            nearNode = self.nodeList[i]
            tNode = self.steer(nearNode, nnode - 1)

            obstacleOK = self.CollisionCheck(tNode)
            imporveCost = nearNode.cost > tNode.cost

            if obstacleOK and imporveCost:
                #  print("rewire")
                self.nodeList[i] = tNode

    def DrawGraph(self, rnd=None):
        u"""
        Draw Graph
        """
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
        plt.cla()
        ax = self.fig.add_subplot(111)
        ax = pickle.load(file('images/background.pickle'))

        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot(node.path_x, node.path_y, "-g")
                #  plt.plot([node.x, self.nodeList[node.parent].x], [
                #  node.y, self.nodeList[node.parent].y], "-g")

        for obstacle in self.obstacleList:
            if obstacle[0] == "circle":
                (ox, oy, size) = obstacle[1:]
                ax.add_patch(patches.Circle((ox,oy),size,fc="k",ec="k"))

            elif obstacle[0] == "rectangle":
                (ox, oy, odx, ody) = obstacle[1:]
                ax.add_patch( patches.Rectangle( (ox, oy), odx, ody, fc="k"))

            else:
                print("Obstacle {} not found.\n".format(obstacle))

        dubins_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        dubins_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

        plt.axis([self.minrand, self.maxrand, self.minrand, self.maxrand])
        plt.grid(True)

        pickle.dump(ax, file('images/rrtstar.pickle', 'w'))

        plt.pause(0.001)


    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd.x) ** 2 +
                 (node.y - rnd.y) ** 2 +
                 (node.yaw - rnd.yaw) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def CollisionCheck(self, node):
        # rectangle, x, y, dx, dy
        # circle, x, y, r

        for (ix, iy) in zip(node.path_x, node.path_y):
            if ix < self.minrand or ix > self.maxrand or iy < self.minrand or iy > self.maxrand:
                return False

        for obstacle in self.obstacleList:
            if obstacle[0] == "circle":
                (ox, oy, size) = obstacle[1:]
                for (ix, iy) in zip(node.path_x, node.path_y):
                    dx = ox - ix
                    dy = oy - iy
                    d = dx * dx + dy * dy
                    if d <= size ** 2:
                        return False  # collision
            elif obstacle[0] == "rectangle":
                (ox, oy, odx, ody) = obstacle[1:]
                for (ix, iy) in zip(node.path_x, node.path_y):
                    if (ox < ix and ix < ox + odx) and (oy < iy and iy < oy + ody):
                        return False

            else:
                print('Object {} not found!'.format(obstacle[0]))

        return True  # safe


class Node():
    u"""
    RRT Node
    """

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        self.cost = 0.0
        self.parent = None


if __name__ == '__main__':
    print("Start rrt start planning")
    import matplotlib.pyplot as plt
    fig = plt.figure(1)
    plt.plot([0,12],[0,12])
    plt.pause(0.001)
    time.sleep(1)
    curvature = 1

    # ====Search Path with RRT====
    obstacleList = [
        ("circle", 5, 5, 1),
        ("circle", 3, 6, 2),
        ("circle", 3, 8, 2),
        ("circle", 3, 10, 2),
        ("circle", 7, 5, 2),
        ("circle", 9, 5, 2),
        ("rectangle", 1, 1, 2, 1),
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    start = [0.0, 0.0, math.radians(0.0)]
    goal = [10.0, 10.0, math.radians(0.0)]

    rrt = RRT(start, goal, randArea=[-2.0, 15.0], obstacleList=obstacleList, maxIter=50,
    fig=fig, curvature=curvature, radius_graph_refinement=0.5)
    path = rrt.Planning(animation=True)

    # start, goal, obstacleList, randArea, goalSampleRate=10, maxIter=1000
    # Draw final path
    rrt.DrawGraph()
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.grid(True)
    plt.pause(0.001)
    plt.show()
