import graphviz
import cv2
import os,csv


class NodeNotInGraph(Exception):
    def __init__(self, node):
        self.node = node

    def __str__(self):
        return "Node %s not in graph." % self.node


class Edge(object):
    def __init__(self, source, target, weight=1.0, action=None):
        self.source = source
        self.target = target
        self.weight = weight
        self.action = action

    def __hash__(self):
        return hash("%s_%s_%f_%s" % (self.source, self.target, self.weight, self.action))

    def __eq__(self, other):
        return self.source == other.source and self.target == other.target \
            and self.weight == other.weight and self.action == other.action

    def __repr__(self):
        return "Edge(%r,%r,%r,%r)" % (self.source, self.target, self.weight, self.action)


class Graph(object):
    def __init__(self, node_label_fn=None):
        self._nodes = set()
        self._edges = dict()
        self.node_label_fn = node_label_fn if node_label_fn else lambda x: x
        self.node_positions = dict()

    def __contains__(self, node):
        return node in self._nodes

    def add_node(self, node):
        """Adds a node to the graph."""
        self._nodes.add(node)

    def add_edge(self, node1, node2, weight=1.0,action=None, bidirectional=False):
        """Adds an edge between node1 and node2. Adds the nodes to the graph first
        if they don't exist."""
        self.add_node(node1)
        self.add_node(node2)
        node1_edges = self._edges.get(node1, set())
        node1_edges.add(Edge(node1, node2, weight, action))
        self._edges[node1] = node1_edges
        if bidirectional:
                node2_edges = self._edges.get(node2, set())
                node2_edges.add(Edge(node2, node1, weight, action))
                self._edges[node2] = node2_edges

    def set_node_positions(self, positions):
        self.node_positions = positions

    def set_node_pos(self, node, pos):
        """Sets the (x,y) pos of the node, if it exists in the graph."""
        if not node in self:
            raise NodeNotInGraph(node)
        self.node_positions[node] = pos

    @property
    def intersection_nodes(self):
        "nodes that are at an intersection and are displayed in the gui map"
        nodes = [n for n in self._nodes if n.isdigit()]
        return [node for node in nodes if int(node) % 2 == 1] # only uneven node numbers are relevant. See draw methods.

    def get_node(self, node_number):
        """
        Get the node with a given number
        """
        for n in self._nodes:
            if n == str(node_number):
                return n

        # No node with the given number was found...
        return None

    def get_node_pos(self, node):
        if not node in self:
            raise NodeNotInGraph(node)
        return self.node_positions[node]

    def node_edges(self, node):
        if not node in self:
            raise NodeNotInGraph(node)
        return self._edges.get(node, set())

    def get_node_number(self, node):
        if not node in self:
            raise NodeNotInGraph(node)
        return self.node_label_fn(node)

    def get_inverted_graph(self):
        """
        Returns a graph that has the same nodes. But all edges point in the
        opposite direction.
        :return:
        """
        graph = Graph()
        for n in self._nodes:
            graph.add_node(n)

        for n in self._nodes:
            edges = self.node_edges(n)
            for e in edges:
                # Invert direction
                graph.add_edge(e.target, e.source, e.weight, e.action)

        return graph


    def get_apriltags_mapping(self,map_dir,csv_filename='autolab_tags_map'):
        apriltags_mapping = dict()
        #for n in self._nodes:
        #    print n
        #    print self.get_node_pos(n)
        #    print '--------------'
        map_path = os.path.join(map_dir, csv_filename + '.csv')
        with open(map_path, 'rb') as f:
            spamreader = csv.reader(f,skipinitialspace=True)
            for i,row in enumerate(spamreader):
                if i != 0:
                    row_ = [element.strip() for element in row]
                    #TagID - 0, x - 1, y - 2, pos - 3, rot - 4
                    if row_[3] == '2' and row_[4] == '0':
                        apriltags_mapping[row_[0]] = self.get_node_by_pos((float(row_[1])+1,float(row_[2])-0.25))
                    elif row_[3] == '4' and row_[4] == '90':
                        apriltags_mapping[row_[0]] = self.get_node_by_pos((float(row_[1])+0.25,float(row_[2])+1))
                    elif row_[3] == '6' and row_[4] == '180':
                        apriltags_mapping[row_[0]] = self.get_node_by_pos((float(row_[1])-1,float(row_[2])+0.25))
                    elif row_[3] == '0' and row_[4] == '270':
                        apriltags_mapping[row_[0]] = self.get_node_by_pos((float(row_[1])-0.25,float(row_[2])-1))
        return apriltags_mapping

    def get_node_by_pos(self,position):
        for n in self._nodes:
            if round(self.node_positions[n][0],2) == round(position[0],2) and round(self.node_positions[n][1],2) == round(position[1],2):
                return n
        return None

    def draw(self, map_dir, highlight_edges=None, show_weights=None, map_name = 'duckietown', highlight_nodes = None):
        if highlight_nodes:
            start_node = highlight_nodes[0]
            target_node = highlight_nodes[1]

        g = graphviz.Digraph(name="duckietown", engine="neato")
        g.edge_attr.update(fontsize = '8', arrowsize = '0.5', arrowhead = 'open')
        g.node_attr.update(shape="circle", fontsize='14',margin="0", height='0')
        #g.graph_attr.update(ratio = '0.7', inputscale = '1.3')

        g.body.append(r'label = "\nduckiegraph"')
        g.body.append('fontsize=16')

        for node in self._nodes:
            node_name = self.node_label_fn(node)
            node_pos = "%f,%f!" % (self.node_positions[node][0], self.node_positions[node][1])
            if highlight_nodes and node == target_node:
                g.node(name=node_name, pos=node_pos, color='magenta', shape='circle') #green
            elif highlight_nodes and node == start_node:
                g.node(name=node_name, pos=node_pos, color='red', shape='circle') #blue
            elif node_name[0:4] == 'turn':
                g.node(name=node_name, pos=node_pos, fixedsize='true', width='0', height='0', style='invis', label="")
            elif (int(node_name) % 2) == 0:
                g.node(name=node_name, pos=node_pos, fixedsize='true', width='0', height='0', style='invis', label="")
            else:
                g.node(name=node_name, pos=node_pos)
        for src_node, edges in self._edges.items():
            for e in edges:
                if show_weights:
                    t = str(e.weight)
                else:
                    t = ""

                if highlight_edges and Edge(self.node_label_fn(src_node), self.node_label_fn(e.target), e.weight,
                                            e.action) in highlight_edges:
                    c = 'cyan' #red
                    p = '3.0'
                else:
                    c  = 'black'
                    p = '1.5'

                g.edge(self.node_label_fn(src_node), self.node_label_fn(e.target), taillabel=t, color=c, penwidth = p)


        g.format = 'png'
        g.render(filename=map_name, directory=map_dir, view=False, cleanup=False)

        # crop lower useless title on the bottom of the rendered image
        image_path = os.path.join(map_dir, map_name+'.png')
        img = cv2.imread(image_path)
        return img[:-50, :]
