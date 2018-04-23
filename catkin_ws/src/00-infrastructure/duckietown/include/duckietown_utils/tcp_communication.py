from duckietown_utils import robot_name
import rospy
from duckietown_msgs.srv import GetVariable, SetVariable
from std_msgs.msg import String
import json


def getTCPVariable(variable_name):
    veh = robot_name.get_current_robot_name()

    getVar = rospy.ServiceProxy("/" + str(veh) + "/tcp_communication_client_node/get_variable", GetVariable)

    name = String()
    name.data = json.dumps(variable_name)
    var = getVar(name)

    rospy.loginfo(str(var))
    return json.loads(var.value_json.data)
