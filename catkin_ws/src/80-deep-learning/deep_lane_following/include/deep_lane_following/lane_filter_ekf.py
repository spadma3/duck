from duckietown_utils.parameters import Configurable
from duckietown_msgs.msg import Segment
import numpy as np
from lane_filter.lane_filter_interface import LaneFilterInterface
import copy



class LaneFilterEKF(Configurable, LaneFilterInterface):
    """LaneFilterEKF"""

    def __init__(self,configuration):
        # TODO list whatever params you need here:
        param_names = [
            'example2'
        ]
        configuration = copy.deepcopy(configuration)
        Configurable.__init__(self,param_names,configuration)

        self.initialize()
        
        
    def predict(self, dt, v, w):
        #TODO: do the predict step
        return

    def update(self, segments):
        #TODO: do the update step
        return
 
    def getEstimate(self):
        #TODO: output the current estimate
        return
        
    def initialize(self):
        #TODO: initialize things
        return

    def inLane(self):
        #TODO: output true or false based on whether the estimate is "good" enough
        return True
