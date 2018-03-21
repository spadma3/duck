#!/usr/bin/env python
'''
Author: Shih-Yuan Liu
'''
import sys
import rospkg
import yaml
from graphviz import Digraph

colours = {"implemented": "black", "in_progress": "black", "planned": "black", "parallel_autonomy": "blue"}

# TODO read from command line

config = "baseline"
param_file = "default"

argv = sys.argv
if len(argv) > 1:
	param_file = sys.argv[1]

file_path = rospkg.RosPack().get_path("duckietown") + "/config/" + config + "/fsm/fsm_node/" + param_file + ".yaml"
print "Load file: %s"%(file_path)
# Load yaml as dictionary
with file(file_path, "r") as f:
	yaml_dict = yaml.load(f)

all_state_dict = yaml_dict["states"]
events_dict = yaml_dict["events"]
global_trans_dict = yaml_dict["global_transitions"]

dot = Digraph(comment=param_file + ".yaml", node_attr={'shape': 'record', 'height': '.1'})
dot_detailed = Digraph(comment=param_file + "_detailed.yaml", node_attr={'shape': 'record', 'height': '.1'})

# Define state nodes
for state_name, state_dict in all_state_dict.items():

	if "active_nodes" in state_dict.keys():
		active_nodes = state_dict["active_nodes"]
		string = "<{ " + state_name + " | <B> ACTIVE NODES </B>"
		for node in active_nodes:
			string += " | " + node
		string += " }>"
	else:
		string = state_name

	if "current_status" in state_dict.keys():
		dot.node(state_name,state_name, fontcolor=colours[state_dict["current_status"]], color=colours[state_dict["current_status"]])
		dot_detailed.node(state_name,string, fontcolor=colours[state_dict["current_status"]], color=colours[state_dict["current_status"]])
	else:
		dot.node(state_name,state_name)
		dot_detailed.node(state_name,string)

	print "State: %s" %(state_name)

# Define transitions
for state_name,state_dict in all_state_dict.items():
	transition_dict = state_dict.get("transitions")
	if transition_dict is not None:
		for event_name, next_state in transition_dict.items():
			if "current_status" in state_dict.keys():
				dot.edge(state_name,next_state,label=event_name, fontcolor=colours[state_dict["current_status"]], color=colours[state_dict["current_status"]])
				dot_detailed.edge(state_name,next_state,label=event_name, fontcolor=colours[state_dict["current_status"]], color=colours[state_dict["current_status"]])

			else:
				dot.edge(state_name,next_state,label=event_name, fontcolor="black", color="black")
				dot_detailed.edge(state_name,next_state,label=event_name, fontcolor="black", color="black")
			print "Transition: %s -- %s --> %s " %(state_name, event_name, next_state)

# Global transitions
if global_trans_dict is not None:
	dot.node("ALL_STATES","All States",style="dashed")
	dot_detailed.node("ALL_STATES","All States",style="dashed")
	for event_name, next_state in global_trans_dict.items():
		dot.edge("ALL_STATES", next_state, label=event_name, style="dashed", concentrate='false')
		dot_detailed.edge("ALL_STATES", next_state, label=event_name, style="dashed", concentrate='false')

dot_file_name = "fsm_"+param_file+".dot"
dot_detailed_file_name = "fsm_"+param_file+"_detailed.dot"
with file(dot_file_name,"w") as f:
	f.write(dot.source)

with file(dot_detailed_file_name,"w") as f:
	f.write(dot_detailed.source)

print "Wrote to %s" %(dot_file_name)
print "Wrote to %s" %(dot_detailed_file_name)
