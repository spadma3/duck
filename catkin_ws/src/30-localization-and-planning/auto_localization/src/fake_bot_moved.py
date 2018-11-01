#!/usr/bin/env python
import rospkg
import rospy
from duckietown_msgs.msg import GlobalPoseArray, GlobalPose
import math

bot_fps = 5

# Fake data
delta_x = 0.1
delta_y = 0.1
delta_theta = math.radians(5)
cam_id = [12, 13]
reference_tag_id = [123, 124]
latency = 0.1

# The unit are meters

# In this script, we don't really take care about Coordinations.
# Just remember y means z
# And the Coordination looks like this
# ---------------> x
# |
# |
# |
# |
# v
# z
# With clockwise increase angle and the origin of angle points to x directions

def route_plan(pt_num, x, y, r, shape='CIRCLE'):

    route_list = []
    if shape == 'CIRCLE':
        origin_x = x
        origin_y = y
        radius = r
        for i in range(pt_num):
            bot_x = origin_x + radius * math.cos(i*2*math.pi/pt_num)
            bot_z = origin_y + radius * math.sin(-1*i*2*math.pi/pt_num)
            bot_theta = i*2*math.pi/pt_num + math.pi/2
            route_list.append([bot_x, bot_z, bot_theta])
    else:
        pass

    return route_list

def pubPoses():

    pub_poses = rospy.Publisher('/duckietown3/bot_poses', GlobalPoseArray, queue_size=1)
    rate = rospy.Rate(bot_fps) # 5hz

    while True:
        route_list = []
        route_pt_num = 500
        route_list.append(route_plan(route_pt_num, 3.25, 6, 0.5))
        route_list.append(route_plan(route_pt_num, 2, 3, 0.8))

        for i in range(route_pt_num):
            gpa = GlobalPoseArray()
            fake_id = 0
            for j in route_list:
                gp = GlobalPose()
                gp.bot_id = fake_id
                gp.header.stamp = rospy.get_rostime()
                gp.pose.x = j[i][0]
                gp.pose.y = j[i][1]
                gp.pose.theta = j[i][2]
                gp.delta_x = delta_x
                gp.delta_y = delta_y
                gp.delta_theta = delta_theta
                gp.cam_id = cam_id
                gp.reference_tag_id = reference_tag_id
                gpa.poses.append(gp)
                fake_id += 1
            rospy.sleep(latency)
            pub_poses.publish(gpa)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('fake_bot_move',anonymous=False)
    try:
        pubPoses()
    except rospy.ROSInterruptException:
        pass
