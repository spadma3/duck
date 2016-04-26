#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32, Int8, String
from rgb_led import RGB_LED
from duckietown_msgs.msg import BoolStamped


class LEDEmitter(object):
    def __init__(self):
        self.led = RGB_LED()
        self.node_name = rospy.get_name()
        self.cycle = 0.0
        
        self.is_on = False
        self.active = True

        self.protocol = rospy.get_param("~LED_protocol") #should be a list of tuples

        self.pattern_off = [[0,0,0]] * 5
        self.pattern = self.pattern_off

        scale = 0.5
        for _, c in self.protocol['colors'].items():
            for i in range(3):
                c[i] = c[i]  * scale

        self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(.1), self.cycleTimer)
        self.current_pattern_name = None

        self.pub_state = rospy.Publisher("~current_led_state",Float32,queue_size=1)
        self.sub_pattern = rospy.Subscriber("~change_color_pattern", String, self.changePattern)
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped,self.cbSwitch)
        
        self.changePattern_('CAR_SIGNAL_A')

    def cbSwitch(self, switch_msg): # active/inactive switch from FSM
        rospy.loginfo('[%s] received switch signal %s'%(self.node_name, switch_msg.data))
        self.active = switch_msg.data
        if(not self.active):
            self.changePattern_('light_off') 

        self.changeFrequency()
    
    def cycleTimer(self,event):
        #rospy.loginfo('[%s] Timer running, %s' % (self.node_name, self.is_on))
        for i in range(5):
            pt = [0, 0, 0] if self.is_on else self.pattern[i]
            self.led.setRGB(i, pt)
            self.is_on = not self.is_on

    def changePattern(self, msg):
        if(self.active):
            self.changePattern_(msg.data)
        else:
            rospy.loginfo('[%s] WARNING: message received, but node is not active!', self.node_name)

    def changePattern_(self, pattern_name):
        if pattern_name:
            if (self.current_pattern_name == pattern_name):
                return
            else:
                self.current_pattern_name = pattern_name

            rospy.loginfo('[%s] changePattern(%r)' % (self.node_name, pattern_name))
            color = self.protocol['signals'][pattern_name]['color']
            self.cycle = self.protocol['signals'][pattern_name]['frequency']
            rospy.loginfo("[%s] color: %s, freq (Hz): %s "%(self.node_name, color, self.cycle))

            self.pattern = [[0,0,0]] * 5
            self.pattern[2] = self.protocol['colors'][color]
            #print(self.pattern)

            if pattern_name in ['traffic_light_go', 'traffic_light_stop']:
                self.pattern = [self.protocol['colors'][color]] * 5

            self.changeFrequency()

    def changeFrequency(self): 
        try:
            self.cycle_timer.shutdown()
            # initially set color (covers the case when frequency is 0.0)
            for i in range(5):
                self.led.setRGB(i, self.pattern[i])
                self.is_on = True
                
            # if frequency is 0.0, timer is not initialized at all
            if(self.cycle):
                #below, convert to hz
                d = 1.0/(2.0*self.cycle)
                self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(d), self.cycleTimer)

        except ValueError as e:
            self.cycle = 0.0
            self.current_pattern_name = None
    	self.pub_state.publish(float(self.cycle))

if __name__ == '__main__':
    rospy.init_node('led_emitter',anonymous=False)
    node = LEDEmitter()
    rospy.spin()

