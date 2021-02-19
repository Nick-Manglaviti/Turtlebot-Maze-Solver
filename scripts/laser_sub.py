#! /usr/bin/env python 

import rospy
from sensor_msgs.msg import LaserScan
import time

class LaserTopicSub(object):

    def __init__(self):
        self._subscriber = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.__debug_callback)
        self._laser_data = LaserScan()
        self._front = 0.0
        self._left = 0.0
        self._right = 0.0

    def __debug_callback(self, msg):
        self._laser_data = msg
        rospy.logdebug(self._laser_data)

    def get_laser_data(self):
        return self._laser_data


if __name__ == '__main__':
    rospy.init_node("laser_sub_node", log_level=rospy.INFO)
    laser_scan_obj = LaserTopicSub()
    time.sleep(2)
    rate = rospy.Rate(0.5)
    ctrl_c = False

    def shutdownhook():
        rospy.loginfo("laser_scan_sub shutting down...")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        data = laser_scan_obj.get_laser_data().ranges
        minimum = data[0]
        index = 0
        for i in range(0, 360):
            if minimum > data[i]:
                minimum = data[i]
                index = i
        print("Value = " + str(minimum) + " Index = " + str(index))
        rate.sleep()