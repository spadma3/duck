#!/usr/bin/env python
# Utility functions for coordinate transformations

import tf
import tf.transformations as tr
import numpy as np
from collections import defaultdict


##### COORDINATE TRANSFORMATION UTILITY FUNCTIONS #############################
# creates a 4x4 translation matrix from a 1x3 translation vector and
# a 1x4 quaternion vector

# Josefine Version of getting tf matrix
# def create_tf_matrix(trans, rot):
#     # numpy arrays to 4x4 transform matrix
#     trans_mat = tr.translation_matrix(trans)
#     rot_mat = tr.quaternion_matrix(rot)
#     transformation_matrix = np.dot(trans_mat, rot_mat)
#     return transformation_matrix

# Eric Version of getting tf matrix
# using tf.transformation.compose_matrix
# See here for more information http://docs.ros.org/jade/api/tf/html/python/transformations.html
def create_tf_matrix(trans, rot):
    # numpy arrays to 4x4 transform matrix
    transformation_matrix = tr.compose_matrix(angles = tr.euler_from_quaternion(rot), translate = trans)
    return transformation_matrix

def inverse_matrix(mat):

    return tr.inverse_matrix(mat)

# #euler and translation to Homography
# #para is x,y,z,roll,pitch,yaw
# def create_tf_matrix(trans, rot):
#     para = np.zeros(6)
#     para[3], para[4], para[5] = tr.euler_from_quaternion(rot)
#
#     Rx = np.array([[1,0,0],
#                    [0,np.cos(para[3]),-np.sin(para[3])],
#                    [0,np.sin(para[3]), np.cos(para[3])]])
#     Ry = np.array([[ np.cos(para[4]),0,np.sin(para[4])],
#                    [0,1,0],
#                    [-np.sin(para[4]),0,np.cos(para[4])]])
#     Rz = np.array([[np.cos(para[5]),-np.sin(para[5]),0],
#                    [np.sin(para[5]), np.cos(para[5]),0],
#                    [0,0,1]])
#     R = np.dot(np.dot(Rz,Ry),Rx)
#     t = np.array([[trans[0]],[trans[1]],[trans[2]]])
#     return np.concatenate((np.concatenate((R,t), axis=1),np.array([[0,0,0,1]])), axis=0)


def rot_trans_from_matrix(mat):
    trans = tf.transformations.translation_from_matrix(mat)
    rot = tf.transformations.quaternion_from_matrix(mat)
    return trans, rot

# This function projects the 3D position onto the 2D plane
def project_position_to_2D_plane(mat_bot_abs):

    trans_bot_abs, rot_bot_abs = rot_trans_from_matrix(mat_bot_abs)

    x = trans_bot_abs[0]
    y = trans_bot_abs[1]
    # returns theta in radians
    alpha,beta,theta = tr.euler_from_matrix(mat_bot_abs, 'rxyz')

    return x,y,theta


# computes a coordinate transformation through concationation
# of translation matrices
# INPUT:    translation matrix of A with respect to B
#           translation matrix of B with respect to C
# OUTPUT:   translation matrix of A with respect to C
def absolute_from_relative_position(mat_A_B, mat_B_C):
    # https://answers.ros.org/question/215656/how-to-transform-a-pose/

    # TODO: verify order of arguments
    # mat_bot_abs = tr.concatenate_matrices(mat_tag_abs, mat_bot_tag)
    # mat_A_C = tr.concatenate_matrices(mat_A_B, mat_B_C)
    mat_A_C = np.dot(mat_A_B, mat_B_C)
    return mat_A_C



# takes a PoseStamped.pose and returns translation and orientations as arrays
def get_trans_rot_from_pose(pose):
    # trans: xyz translation
    # rot: xyzw quaternion
    # mat: 4x4 transformation matrix
    position = pose.position
    orientation = pose.orientation
    trans = [position.x, position.y, position.z]
    rot = [orientation.x, orientation.y, orientation.z, orientation.w]
    return trans, rot

# takes a PoseStamped.pose and returns a 4x4 translation matrix
def get_matrix_from_pose(pose):
    # mat: 4x4 transformation matrix
    position = pose.position
    orientation = pose.orientation
    trans = [position.x, position.y, position.z]
    rot = [orientation.x, orientation.y, orientation.z, orientation.w]
    mat = create_tf_matrix(trans, rot)
    return mat



##########################
### code from: https://gist.github.com/econchick/4666413
# default edge weight is 1
class Graph:
    def __init__(self):
        self.nodes = []
        self.edges = defaultdict(list)
        #self.edges = {}
        self.distances = {}
        self.edges2 = []

    def add_node(self, value):
        if not value in self.nodes:
            self.nodes.append(value)

    # INPUT: frame_id, bot_id In this order! default distance is 1
    # from --> to
    def add_edge(self, parent_node, child_node, distance=1):
        # from --> to
        self.edges[parent_node].append(child_node)
        # self.edges[to_node].append(from_node) # graph is unidirectional
        self.distances[(parent_node, child_node)] = distance
        self.edges2.append((parent_node, child_node, distance))

# # Origin of the Grapg is the first node
# def dijkstra(graph, initial, origin):
#     if initial==origin:
#         return initial
#
#     visited = {initial: 0}
#     path = {}
#
#     nodes = set(graph.nodes)
#
#     while nodes:
#         min_node = None
#         for node in nodes:
#             if node in visited:
#                 if min_node is None:
#                     min_node = node
#                 elif visited[node] < visited[min_node]:
#                     min_node = node
#         if min_node is None:
#             break
#
#         nodes.remove(min_node)
#         current_weight = visited[min_node]
#
#         for edge in graph.edges[min_node]:
#             weight = current_weight + graph.distances[(min_node, edge)]
#             if edge not in visited or weight < visited[edge]:
#                 visited[edge] = weight
#                 path[edge] = min_node
#
#     path_nodes = [initial].append(dijkstra(graph, path[origin],origin))
#     #path_nodes = path.keys()
#     #path_nodes.append(initial)
#     print visited
#     print path
#     return path_nodes
#
# # Dijkstra's algorithm for shortest paths
# # David Eppstein, UC Irvine, 4 April 2002
#
# # http://aspn.activestate.com/ASPN/Cookbook/Python/Recipe/117228
# #from priodict import priorityDictionary
#
# def Dijkstra(G,start,end=None):
# 	"""
# 	Find shortest paths from the start vertex to all
# 	vertices nearer than or equal to the end.
#
# 	The input graph G is assumed to have the following
# 	representation: A vertex can be any object that can
# 	be used as an index into a dictionary.  G is a
# 	dictionary, indexed by vertices.  For any vertex v,
# 	G[v] is itself a dictionary, indexed by the neighbors
# 	of v.  For any edge v->w, G[v][w] is the length of
# 	the edge.  This is related to the representation in
# 	<http://www.python.org/doc/essays/graphs.html>
# 	where Guido van Rossum suggests representing graphs
# 	as dictionaries mapping vertices to lists of neighbors,
# 	however dictionaries of edges have many advantages
# 	over lists: they can store extra information (here,
# 	the lengths), they support fast existence tests,
# 	and they allow easy modification of the graph by edge
# 	insertion and removal.  Such modifications are not
# 	needed here but are important in other graph algorithms.
# 	Since dictionaries obey iterator protocol, a graph
# 	represented as described here could be handed without
# 	modification to an algorithm using Guido's representation.
#
# 	Of course, G and G[v] need not be Python dict objects;
# 	they can be any other object that obeys dict protocol,
# 	for instance a wrapper in which vertices are URLs
# 	and a call to G[v] loads the web page and finds its links.
#
# 	The output is a pair (D,P) where D[v] is the distance
# 	from start to v and P[v] is the predecessor of v along
# 	the shortest path from s to v.
#
# 	Dijkstra's algorithm is only guaranteed to work correctly
# 	when all edge lengths are positive. This code does not
# 	verify this property for all edges (only the edges seen
#  	before the end vertex is reached), but will correctly
# 	compute shortest paths even for some graphs with negative
# 	edges, and will raise an exception if it discovers that
# 	a negative edge has caused it to make a mistake.
# 	"""
#
# 	D = {}	# dictionary of final distances
# 	P = {}	# dictionary of predecessors
# 	Q = priorityDictionary()   # est.dist. of non-final vert.
# 	Q[start] = 0
#
# 	for v in Q:
# 		D[v] = Q[v]
# 		if v == end: break
#
# 		for w in G[v]:
# 			vwLength = D[v] + G[v][w]
# 			if w in D:
# 				if vwLength < D[w]:
# 					raise ValueError, \
#   "Dijkstra: found better path to already-final vertex"
# 			elif w not in Q or vwLength < Q[w]:
# 				Q[w] = vwLength
# 				P[w] = v
#
# 	return (D,P)
#
# def shortestPath(G,start,end):
# 	"""
# 	Find a single shortest path from the given start vertex
# 	to the given end vertex.
# 	The input has the same conventions as Dijkstra().
# 	The output is a list of the vertices in order along
# 	the shortest path.
# 	"""
#
# 	D,P = Dijkstra(G,start,end)
# 	Path = []
# 	while 1:
# 		Path.append(end)
# 		if end == start: break
# 		end = P[end]
# 	Path.reverse()
# 	return Path

## algorithm from wikipedia:
#https://de.wikibooks.org/wiki/Algorithmensammlung:_Graphentheorie:_Dijkstra-Algorithmus

def dijkstra(knoten, kanten, start, ziel):
    # knoten ist eine Liste von Knoten
    # kanten ist eine Liste von 3-Tupeln:
    #   (knoten1, knoten2, kosten)
    # start ist der Knoten, in dem die Suche startet
    # ziel ist der Knoten, zu dem ein Weg gesucht werden soll
    # Gibt ein Tupel zuruck mit dem Weg und den Kosten
    #
    knotenEigenschaften = [ [i, 1000000, None, False] for i in knoten if i != start ]
    knotenEigenschaften += [ [start, 0, None, False] ]
    for i in range(len(knotenEigenschaften)):
    	knotenEigenschaften[i] += [ i ]

    while True:
    	unbesuchteKnoten = filter(lambda x: not x[3], knotenEigenschaften)
    	if not unbesuchteKnoten: break
    	sortierteListe = sorted(unbesuchteKnoten, key=lambda i: i[1])
    	aktiverKnoten = sortierteListe[0]
    	knotenEigenschaften[aktiverKnoten[4]][3] = True
    	if aktiverKnoten[0] == ziel:
    		break
    	aktiveKanten = filter(lambda x: x[0] == aktiverKnoten[0], kanten)
    	for kante in aktiveKanten:
    		andererKnotenId = filter(lambda x: x[0] == kante[1], knotenEigenschaften)[0][4]
    		gewichtSumme = aktiverKnoten[1]	+ kante[2]
    		if gewichtSumme < knotenEigenschaften[andererKnotenId][1]:
    			knotenEigenschaften[andererKnotenId][1] = gewichtSumme
    			knotenEigenschaften[andererKnotenId][2] = aktiverKnoten[4]

    if aktiverKnoten[0] == ziel:
    	weg = []
    	weg += [ aktiverKnoten[0] ]
    	kosten = 0
    	while aktiverKnoten[0] != start:
            #print aktiverKnoten
            if aktiverKnoten[2] == None:
                #print "No path to node: ",start
                return None
                #return

            aktiverKnoten = knotenEigenschaften[aktiverKnoten[2]]
            weg += [ aktiverKnoten[0] ]
            kosten += aktiverKnoten[1]
        weg.reverse()
        return weg
    else:
        raise "No way found"
