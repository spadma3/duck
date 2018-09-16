import pickle, csv, os, sys, cv2
import numpy as np
from graph import Graph
#from duckietown_description import Csv2Xacro

class Node():
	n = 1
	def __init__(self, pos, direction,name='default'):
		if name == 'default':
			self.name = str(Node.n)
			Node.n = Node.n + 1
		else:
			self.name = name
		self.pos = pos
		self.direction = direction


	def flow(self, node):
		return (self.direction[0]+node.direction[0])**2 + (self.direction[1]+node.direction[1])**2

	def rotateDirection(self,theta):
		theta = np.deg2rad(theta)
		vec = np.matrix(self.direction).transpose()
		rotMatrix = np.matrix([[np.cos(theta), -np.sin(theta)],
			[np.sin(theta), np.cos(theta)]])
		res = np.dot(rotMatrix,vec)
		res = res.tolist()
		return (int(res[0][0]),int(res[1][0]))

	def rotateAndTranslatePos(self,theta,x,y):
		theta = np.deg2rad(theta)
		vec = np.matrix(self.pos).transpose()
		vec = np.append(vec,[[1]],axis=0)
		rotMatrix = np.matrix([[np.cos(theta), -np.sin(theta), x+0.5],
			[np.sin(theta), np.cos(theta), y+0.5],
			[0,0,1]])

		res = np.dot(rotMatrix,vec)
		res = res[0:2,0]
		res = res.tolist()

		return (res[0][0],res[1][0])

	def globalPosAndDirection(self,theta,x,y,name='default'):
		return Node(self.rotateAndTranslatePos(theta,x,y), self.rotateDirection(theta),name)

class Tile():
	def __init__(self, csv_row):
		self.x = float(csv_row[0])
		self.y = float(csv_row[1])
		self.type = csv_row[2]
		self.rotation = float(csv_row[3])

	def create_nodes(self):
		return {},[]

	def create_edges(self,tile_map):
		return []

	def connect_node(self, node,tile_map):

		next_tile_pos_x = self.x + node.direction[0]
		next_tile_pos_y = self.y + node.direction[1]
		t = self.get_tile(next_tile_pos_x,next_tile_pos_y,tile_map)
		while t.type == 'straight':
			next_tile_pos_x = next_tile_pos_x + node.direction[0]
			next_tile_pos_y = next_tile_pos_y + node.direction[1]
			t = self.get_tile(next_tile_pos_x,next_tile_pos_y,tile_map)

		if t.type == 'turn':
			if t.node1.flow(node) == 2:
				return [node.name, t.node1.name, 'f']
			elif t.node2.flow(node) == 2:
				return [node.name, t.node2.name, 'f']
		elif t.type == '3way':
			if t.node1.flow(node) == 4:
				return [node.name, t.node1.name, 'f']
			elif t.node3.flow(node) == 4:
				return [node.name, t.node3.name, 'f']
			elif t.node5.flow(node) == 4:
				return [node.name, t.node5.name, 'f']
		elif t.type == '4way':
			if t.node1.flow(node) == 4:
				return [node.name, t.node1.name, 'f']
			elif t.node3.flow(node) == 4:
				return [node.name, t.node3.name, 'f']
			elif t.node5.flow(node) == 4:
				return [node.name, t.node5.name, 'f']
			elif t.node7.flow(node) == 4:
				return [node.name, t.node7.name, 'f']

	def get_tile(self, x, y,tile_map):
		for tile in tile_map:
			if tile.x == x and tile.y == y:
				return tile

class TurnTile(Tile):
	name = 1000
	node1_default = Node((-0.25,  0.25), (-1, 0), 'node1_default')
	node2_default = Node(( 0.25, -0.25), ( 0, 1), 'node2_default')
	def create_nodes(self):
		x = self.x
		y = self.y
		theta = self.rotation
		self.node1 = TurnTile.node1_default.globalPosAndDirection(theta, x, y, self.getNodeName())
		self.node2 = TurnTile.node2_default.globalPosAndDirection(theta, x, y, self.getNodeName())
		node_loc = {self.node1.name:self.node1.pos, self.node2.name:self.node2.pos}
		edges = []
		return node_loc, edges
	def create_edges(self,tile_map):
		edges = []
		edges.append(self.connect_node(self.node1,tile_map))
		edges.append(self.connect_node(self.node2,tile_map))
		return edges

	def getNodeName(self):
		res = 'turn'+str(TurnTile.name)
		TurnTile.name = TurnTile.name + 1
		return res

class ThreeWayTile(Tile):
	node1_default = Node(( 0.50,  0.25), (-1,  0), 'node1_default')
	node2_default = Node(( 0.25,  0.50), ( 0,  1), 'node2_default')
	node3_default = Node((-0.25,  0.50), ( 0, -1), 'node3_default')
	node4_default = Node((-0.50,  0.25), (-1,  0), 'node4_default')
	node5_default = Node((-0.50, -0.25), ( 1,  0), 'node5_default')
	node6_default = Node(( 0.50, -0.25), ( 1,  0), 'node6_default')
	def create_nodes(self):
		x = self.x
		y = self.y
		theta = self.rotation
		self.node1 = ThreeWayTile.node1_default.globalPosAndDirection(theta, x, y)
		self.node2 = ThreeWayTile.node2_default.globalPosAndDirection(theta, x, y)
		self.node3 = ThreeWayTile.node3_default.globalPosAndDirection(theta, x, y)
		self.node4 = ThreeWayTile.node4_default.globalPosAndDirection(theta, x, y)
		self.node5 = ThreeWayTile.node5_default.globalPosAndDirection(theta, x, y)
		self.node6 = ThreeWayTile.node6_default.globalPosAndDirection(theta, x, y)
		node_loc = {self.node1.name:self.node1.pos, self.node2.name:self.node2.pos,
					self.node3.name:self.node3.pos, self.node4.name:self.node4.pos,
					self.node5.name:self.node5.pos, self.node6.name:self.node6.pos}

		edges = [[self.node1.name, self.node2.name, 'r'],
				 [self.node1.name, self.node4.name, 's'],
				 [self.node3.name, self.node4.name, 'r'],
				 [self.node3.name, self.node6.name, 'l'],
				 [self.node5.name, self.node2.name, 'l'],
				 [self.node5.name, self.node6.name, 's']]
		return node_loc,edges

	def create_edges(self,tile_map):
		edges = []
		edges.append(self.connect_node(self.node2,tile_map))
		edges.append(self.connect_node(self.node4,tile_map))
		edges.append(self.connect_node(self.node6,tile_map))
		return edges

class FourWayTile(Tile):
	node1_default = Node(( 0.50,  0.25), (-1,  0), 'node1_default')
	node2_default = Node(( 0.25,  0.50), ( 0,  1), 'node2_default')
	node3_default = Node((-0.25,  0.50), ( 0, -1), 'node3_default')
	node4_default = Node((-0.50,  0.25), (-1,  0), 'node4_default')
	node5_default = Node((-0.50, -0.25), ( 1,  0), 'node5_default')
	node6_default = Node((-0.25, -0.50), ( 0, -1), 'node6_default')
	node7_default = Node(( 0.25, -0.50), ( 0,  1), 'node7_default')
	node8_default = Node(( 0.50, -0.25), ( 1,  0), 'node8_default')

	def create_nodes(self):
		x = self.x
		y = self.y
		theta = self.rotation
		self.node1 = FourWayTile.node1_default.globalPosAndDirection(theta, x, y)
		self.node2 = FourWayTile.node2_default.globalPosAndDirection(theta, x, y)
		self.node3 = FourWayTile.node3_default.globalPosAndDirection(theta, x, y)
		self.node4 = FourWayTile.node4_default.globalPosAndDirection(theta, x, y)
		self.node5 = FourWayTile.node5_default.globalPosAndDirection(theta, x, y)
		self.node6 = FourWayTile.node6_default.globalPosAndDirection(theta, x, y)
		self.node7 = FourWayTile.node7_default.globalPosAndDirection(theta, x, y)
		self.node8 = FourWayTile.node8_default.globalPosAndDirection(theta, x, y)
		node_loc = {self.node1.name:self.node1.pos, self.node2.name:self.node2.pos,
					self.node3.name:self.node3.pos, self.node4.name:self.node4.pos,
					self.node5.name:self.node5.pos, self.node6.name:self.node6.pos,
					self.node7.name:self.node7.pos, self.node8.name:self.node8.pos}

		edges = [[self.node1.name, self.node2.name, 'r'],
				 [self.node1.name, self.node4.name, 's'],
				 [self.node1.name, self.node6.name, 'l'],
				 [self.node3.name, self.node4.name, 'r'],
				 [self.node3.name, self.node8.name, 'l'],
				 [self.node3.name, self.node6.name, 's'],
				 [self.node5.name, self.node2.name, 'l'],
				 [self.node5.name, self.node8.name, 's'],
				 [self.node5.name, self.node6.name, 'r'],
				 [self.node7.name, self.node8.name, 'r'],
				 [self.node7.name, self.node2.name, 's'],
				 [self.node7.name, self.node4.name, 'l']]

		return node_loc,edges
	def create_edges(self,tile_map):
		edges = []
		edges.append(self.connect_node(self.node2,tile_map))
		edges.append(self.connect_node(self.node4,tile_map))
		edges.append(self.connect_node(self.node6,tile_map))
		edges.append(self.connect_node(self.node8,tile_map))
		return edges

class StraightTile(Tile):
	pass

class graph_creator():
	def __init__(self):
		self.node_locations = {}
		self.edges = []
		self.tile_map = []
	def add_node_locations(self,node_loc):
		self.node_locations.update(node_loc)

	def add_edges(self, ed):
		for edge in ed:
			source = edge[0]
			target = edge[1]
			action = edge[2]
			manhattan_dist = abs(self.node_locations[source][0] - self.node_locations[target][0]) + abs(self.node_locations[source][1] - self.node_locations[target][1])
			self.edges.append([source, target, manhattan_dist, action])
	def pickle_save(self, name='duckietown_map.pkl'):
		afile = open(r'maps/duckietown_map.pkl', 'w+')
		pickle.dump([self.edges, self.node_locations], afile)
		afile.close()

	def build_graph_from_csv(self, map_dir, csv_filename='autolab_tiles_map'):
		map_path = os.path.join(map_dir, csv_filename + '.csv')
		with open(map_path, 'rb') as f:
			spamreader = csv.reader(f,skipinitialspace=True)
			for i,row in enumerate(spamreader):
				if i != 0:
					row_ = [element.strip() for element in row] # remove white spaces
					if row_[2] == 'turn':
						self.tile_map.append(TurnTile(row_))
					elif row_[2] == '3way':
						self.tile_map.append(ThreeWayTile(row_))
					elif row_[2] == '4way':
						self.tile_map.append(FourWayTile(row_))
					elif row_[2] == 'straight':
						self.tile_map.append(StraightTile(row_))
		self.generate_node_locations()
		self.generate_edges()

		duckietown_graph = Graph()
		for edge in self.edges:
			duckietown_graph.add_edge(edge[0], edge[1], edge[2], edge[3])
		duckietown_graph.set_node_positions(self.node_locations)

		return duckietown_graph
	def generate_node_locations(self):
		for tile in self.tile_map:
			node_loc,edges = tile.create_nodes()
			self.add_node_locations(node_loc)
			self.add_edges(edges)
	def generate_edges(self):
		for tile in self.tile_map:
			edges = tile.create_edges(self.tile_map)
			self.add_edges(edges)


class MapImageCreator:

	def __init__(self, tiles_dir):
		self.tile_length = 201
		self.tile_midpoint = (self.tile_length/2,self.tile_length/2)
		self.turn_tile = self.imread_and_resize(os.path.abspath(tiles_dir+'/turn.png'))
		self.three_way_tile = self.imread_and_resize(os.path.abspath(tiles_dir+'/3way.png'))
		self.four_way_tile = self.imread_and_resize(os.path.abspath(tiles_dir+'/4way.png'))
		self.straight_tile = self.imread_and_resize(os.path.abspath(tiles_dir+'/straight.png'))
		self.empty_tile = self.imread_and_resize(os.path.abspath(tiles_dir+'/empty.png'))

	def imread_and_resize(self,path):
		tmp = cv2.imread(path,cv2.IMREAD_COLOR)
		return cv2.resize(tmp,(self.tile_length,self.tile_length),interpolation=cv2.INTER_AREA)

	def build_map_from_csv(self,map_dir, csv_filename):
		map_path = os.path.join(map_dir,csv_filename + '.csv')
		num_tiles_y = -1
		num_tiles_x = -1
		with open(map_path, 'rb') as f:
			spamreader = csv.reader(f,skipinitialspace=True)
			#analyze the file before we build the image
			for (j,row) in enumerate(spamreader):
					if j != 0:
						row_clean = [element.strip() for element in row]
						num_tiles_y = max(int(row_clean[1]),num_tiles_y)
						num_tiles_x = max(int(row_clean[0]),num_tiles_x)
			#since the indices are zero based, the total count is one higher
			num_tiles_x = num_tiles_x + 1
			num_tiles_y = num_tiles_y + 1
			self.map_height = num_tiles_y*self.tile_length
			map_width = num_tiles_x*self.tile_length

			self.map_image = np.zeros((self.map_height,map_width,3),np.uint8)
			f.seek(0) # reset the reader to the beginning of the file
			for i,row in enumerate(spamreader):
				if i != 0:
					row_ = [element.strip() for element in row] # remove white spaces
					if row_[2] == 'turn':
						self.appendTile(row_,self.turn_tile)
					elif row_[2] == '3way':
						self.appendTile(row_,self.three_way_tile)
					elif row_[2] == '4way':
						self.appendTile(row_,self.four_way_tile)
					elif row_[2] == 'straight':
						self.appendTile(row_,self.straight_tile)
					elif row_[2] == 'empty':
						self.appendTile(row_,self.empty_tile)

		cv2.imwrite(os.path.join(map_dir, csv_filename + '_map.png'), self.map_image)
		return self.map_image

	def appendTile(self,row,tile):
		xS = int(row[0])*self.tile_length
		yS = self.map_height-(int(row[1])+1)*self.tile_length
		rotation_matrix = cv2.getRotationMatrix2D(self.tile_midpoint,float(row[3]),1.0)
		oriented_tile = cv2.warpAffine(tile,rotation_matrix,(self.tile_length,self.tile_length))
		self.map_image[yS:yS+self.tile_length,xS:xS+self.tile_length]=oriented_tile
