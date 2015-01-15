#!/usr/bin/env python
from __future__ import division
import roslib#; roslib.load_manifest('phidgets')
import rospy
import rosparam

from std_msgs.msg import Float32

import Phidgets
from Phidgets.Devices.TemperatureSensor import TemperatureSensor, ThermocoupleType


###############################################################################
###############################################################################
class PhidgetsTemperature:

    def __init__(self):
        # Connect to the Phidget.
        self.temperatureSensor = TemperatureSensor()
        self.temperatureSensor.openPhidget()
        rospy.sleep(1)
        self.temperatureSensor.setThermocoupleType(0, ThermocoupleType.PHIDGET_TEMPERATURE_SENSOR_K_TYPE)
        
        self.publish_temperature = rospy.Publisher('/temperature', Float32)

        rospy.init_node('temperature', anonymous=True)
        
    def run(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            temp = self.temperatureSensor.getTemperature(0)
            self.publish_temperature.publish(temp)
            rate.sleep()


if __name__ == '__main__':

    pt = PhidgetsTemperature()
    pt.run()

