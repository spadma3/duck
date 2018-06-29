#!/usr/bin/env python

## AIDO localization Sysem Calibration
# Author: Chen-Lung (Eric) Lu , ETHZ NCTU, eric565648.eed03@g2.nctu.edu.tw

## This script records the positions of all reference Apriltags
# and save them into map file
# The scenerio is that the system will calibrate itself from time to time.

class system_calibration(object):

    def __init__(self):

        self.node_name = 'global_localization_node'

        # load the map file
        self.map_filename = rospy.get_param("~map") + ".yaml"
        self.map_data = self.load_map_info(self.map_filename)

        # Start Calibration


    ## Load Map Data
    def load_map_info(self, filename):

        map_data = yaml.load(file(rospkg.RosPack().get_path('auto_localization')+"/config/"+filename,'r')) # Need RosPack get_path to find the file path
        print "Loaded map from file: ", self.map_filename

        print "\nThis is your map: \n", map_data['tiles']

        return map_data


### ------------------- ------- MAIN -------------------------------#####
if __name__ == '__main__':
    rospy.init_node('system_calibration',anonymous=False)
    node = system_calibration()
    rospy.spin()
