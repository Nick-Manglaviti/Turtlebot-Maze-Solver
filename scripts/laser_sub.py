#! /usr/bin/env python 

import rospy
from sensor_msgs.msg import LaserScan
import time

class LaserTopicSub(object):

    def __init__(self):
        rate = rospy.Rate(1)
        self._subscriber = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.__debug_callback)
        self._laser_data = LaserScan()
        self._front = 0.0
        self._left = 0.0
        self._right = 0.0
        while self._subscriber.get_num_connections() == 0:
            rate.sleep()

    def __debug_callback(self, msg):
        self._laser_data = msg
        rospy.logdebug(self._laser_data)

    def get_laser_data(self):
        return self._laser_data


if __name__ == '__main__':
    rospy.init_node("laser_sub_node")
    laser_obj = LaserTopicSub()
    time.sleep(2)
    data = laser_obj.get_laser_data().ranges
    print("Value: " + str(data[0]))
    print("Value: " + str(data[360]))
    print("Value: " + str(data[719]))