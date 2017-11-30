import yaml
from pprint import pprint
import sys
import pygraphviz as pgv

class Node(object):
    def __init__(self, name, inputs, outputs):
        self.name = name
        self.inputs = inputs
        self.outputs = outputs
        self.parent_nodes = []  # nodes above this node in tree 
        self.child_nodes= []    # nodes below this node in tree

    def can_connect(self, node2):
        # Checks whether this node can connect its outputs 
        # to node2's inputs
        if set(self.outputs) == set(node2.inputs):
            print 'Check 1 passed: nodes have same input properties ', node2.inputs.keys()
            for input in node2.inputs:
                if node2.inputs[input]['type'] == 'max':
                    print 'max value, checking compatibility'
                    if node1.outputs[input]['value'] < node2.inputs[input]['value']:
                        print 'Check 2 passed: compatible max value, ', node1.outputs[input]['value'], '<', node2.inputs[input]['value']
                        return True
                    else:
                        print 'Check 2 failed: uncompatible, value above max allowed', node1.outputs[input]['value'], '>', node2.inputs[input]['max']
                        return None
        else:
            print 'false'
            return None

    def connect(self, node2):
        # Connects this node's outputs to node2's inputs
        # Note that it should be checked whether they may connect
        self.parent_nodes.append(node2)
        node2.child_nodes.append(self)

class YamlHandler():
    def __init__(self, filename):
        self.filename = filename
        self.contents = None

    def read_yaml(self):
        try:
            with open(self.filename, 'r') as f:
                contents = yaml.safe_load(f)

            inputs = contents['inputs']
            pprint(inputs)

        except:
            print 'failed to open supplied file'
            print sys.exc_info()[0], sys.exc_info()[1]

class System():
    def __init__(self):
        self.nodes = []
        self.graph = pgv.AGraph(directed=True)
        self.graph.layout()

    def add_node(self, node):
        self.nodes.append(node)
        self.graph.add_node(node.name)
        print 'Nodes currently in system: '
        for node in self.nodes:
            print node.name
        
    def create_dot_graph(self):
        for node in self.nodes:
            for child in node.child_nodes:
                print child
                self.graph.add_edge(node.name, child.name)
        self.graph.layout()
        self.graph.draw('graph.dot', 'dot')

if __name__ == '__main__':
    # myYamlHandler = YamlHandler('interfaces/node1.yaml')
    # myYamlHandler.read_yaml()
    node1 = Node('node1', {'apples':3}, {'bananas':{'value':15}})
    node2 = Node('node2', {'bananas':{'type':'max', 'value': 20}}, {'pears':5})

    mySystem = System()
    mySystem.add_node(node2)
    if node1.can_connect(node2):
        node1.connect(node2)
        mySystem.add_node(node1)

    mySystem.create_dot_graph()

    print mySystem.graph