#!/usr/bin/env python
from __future__ import division
import roslib
import rospy

from phidgets.srv import *
from phidgets.msg import *
from std_msgs.msg import Float32, Header, String

import Phidgets
from Phidgets.Devices.InterfaceKit import InterfaceKit

###############################################################################
###############################################################################
class PhidgetsSolidStateRelay:

    def __init__(self, topic=None):
        ##########################################################
        #Create an interfacekit object
        try:
            self.interfacekit = InterfaceKit()
        except RuntimeError as e:
            print("Runtime Exception: %s" % e.details)
            print("Exiting....")
            exit(1)
        try:
            self.interfacekit.openPhidget()
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print("Exiting....")
            exit(1)
        rospy.sleep(1)
        ###########################################################
        
        if topic is None:
            topic = '/phidgets/ssr_service'
        self.toggleSSR = rospy.Service(topic, solidStateRelayService, self.solidStateRelayService_callback)
        self.publisher = rospy.Publisher('/phidgets/ssr_activity', ssrActivity)
        
        rospy.init_node('ssr_service', anonymous=True)
        
    def solidStateRelayService_callback(self, solidStateRelay):
        self.control_solid_state_relay(solidStateRelay.port, solidStateRelay.state)        
        return 1
        
    def control_solid_state_relay(self, port, state):
        self.interfacekit.setOutputState(port, state)
        msg = ssrActivity()
        msg.header = Header(stamp=rospy.Time.now())
        msg.ssr_port = port
        msg.ssr_state = state
        self.publisher.publish(msg)
        print port, state
        
    def spin(self):
        rospy.spin()
    
    def pulse(self, pulselength, pulseinterval, port, npulses=1):
        for p in range(npulses):
            self.control_solid_state_relay(port, 1)
            rospy.sleep(pulselength)
            self.control_solid_state_relay(port, 0)
            if npulses > 1:
                rospy.sleep(pulseinterval)
        return
        
if __name__ == '__main__':

    pssr = PhidgetsSolidStateRelay()
    pssr.pulse(1,.5,0,3)

