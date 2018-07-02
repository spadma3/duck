#!/usr/bin/env python

## AIDO localization Sysem Calibration
# Author: Chen-Lung (Eric) Lu , ETHZ NCTU, eric565648.eed03@g2.nctu.edu.tw

## This script records the positions of all reference Apriltags
# and save them into map file
# The scenerio is that the system will calibrate itself from time to time.

class system_calibration(object):

    def __init__(self):

        self.node_name = 'system_calibration'

        # load the map file, notice that it will overwrite the file
        self.map_filename = rospy.get_param("~map") + ".yaml"
        self.map_data = self.load_map_info(self.map_filename)

        # Subscribe all tfs from subfserver node
        self.sub_tfs = rospy.Subscriber("local_poses", RemapPoseArray, self.callback, queue_size=1)

        # Start Calibration

        #Parameters
        self.start_calibrate = False
        self.wait_for_message = 15 # At least wait 3 secs (15/5hz from subfserver_easy node) for tags collection after all watchtower have publish things.

        #Watchtowers, to make sure they all send datas
        self.watchtowers = {}
        for wt in self.map_watchtowers:
            self.watchtowers[wt] = False

    # callback make sure that all watchtowers have sent messages.
    def callback(self, msg_tfs):

        # Return callback if calibration has already started
        if self.start_calibrate == True:
            return

        # Make sure that we get meesage from all watchtowers
        if self.wait_for_message == 15:
            for tf in msg_tfs:
                self.watchtowers[tf.host] = True
            not_ready = ""
            for wt in self.watchtowers:
                if self.watchtowers[wt] == False:
                    not_ready += wt + ", "
            if not_ready == "":
                rospy.loginfo("Get all tags. Start Counting Down")
            else:
                rospy.loginfo("Still waiting for: " + not_ready)
                return

        self.wait_for_message -= 1
        rospy.loginfo("Start Calibration in %d secs", self.wait_for_message/5)

        if self.wait_for_message == 0:
            self.sys_calib(msg_tfs)
            self.start_calibrate = True


    def sys_calib(self, msg_tfs):

        self.find_tag_relationship(msg_tfs)


    def find_tag_relationship(self, tfs):

        # https://www.python.org/doc/essays/graphs/
        tag_graph = {}
        for tf in tfs:
            if not graph.has_key(tf.frame_id):
                graph{'tf.frame_id'} = []
            else:
                graph{'tf.frame_id'}.append(tf.bot_id)


    ## Load Map Data
    def load_map_info(self, filename):

        map_data = yaml.load(file(rospkg.RosPack().get_path('auto_localization')+"/config/"+filename,'w'+'r')) # Need RosPack get_path to find the file path
        print "Loaded map from file: ", self.map_filename

        self.map_tiles = map_data['tiles']
        print "\nThis is your map: \n", self.map_tiles

        self.map_watchtowers = map_data['watchtowers']
        print "\nThese watchtowers suppose to work: \n", self.map_watchtowers

        self.map_origins = map_data['origin']
        print "The origins: \n", self.map_origins

        return map_data


### ------------------- ------- MAIN -------------------------------#####
if __name__ == '__main__':
    rospy.init_node('system_calibration',anonymous=False)
    node = system_calibration()
    rospy.spin()
