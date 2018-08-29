#!/usr/bin/env python
import rospkg
import rospy
from duckietown_msgs.msg import GlobalPoseArray, GlobalPose
import math
import csv

bot_fps = 20

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

def pubPoses():

    pub_poses = rospy.Publisher('bot_global_poses', GlobalPoseArray, queue_size=1)
    rate = rospy.Rate(bot_fps)

    while True:

        with open(rospkg.RosPack().get_path('auto_localization') + "/config/" + 'test20180824-1827040.csv', 'rb') as botfile:
            datas = csv.reader(botfile, delimiter=',')
            for row in datas:
                if row[0] == 'time':
                    continue
                gpa = GlobalPoseArray()
                gp = GlobalPose()
                gp.header.stamp = rospy.Time.from_sec(float(row[0])/1e9)
                gp.bot_id = int(row[1])
                gp.pose.x = float(row[2])
                gp.pose.y = float(row[3])
                gp.pose.theta = float(row[4])
                gp.cam_id.append(int(row[5][2:-1]))
                gp.reference_tag_id.append(int(row[6][2:-1]))
                gpa.poses.append(gp)
                pub_poses.publish(gpa)
                rate.sleep()
            break


if __name__ == '__main__':
    rospy.init_node('fake_bot_move',anonymous=False)
    try:
        pubPoses()
    except rospy.ROSInterruptException:
        pass
