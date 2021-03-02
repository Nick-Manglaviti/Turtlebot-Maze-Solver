#! /usr/bin/env python

import rospy
import time
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class OdomTopicSub(object):

    def __init__(self):
        rate = rospy.Rate(1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.__debug_callback)
        self.odom_data = Odometry()
        while self.odom_sub.get_num_connections() == 0:
            rate.sleep()

    def __debug_callback(self, msg):
        self.odom_data = msg
        rospy.logdebug(self.odom_data)
    
    def get_odom(self):
        return self.odom_data

    def get_degrees(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        orientation_list = [x, y, z, w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        yaw = yaw * (180 / 3.1415)
        return yaw

if __name__ == '__main__':
    rospy.init_node('odom_sub_node')
    odom_sub_obj = OdomTopicSub()
    time.sleep(2)
    ctrl_c = False
    rate = rospy.Rate(0.5)

    def shutdownhook():
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        print("Degrees: " + str(odom_sub_obj.get_degrees(odom_sub_obj.get_odom())))
        rate.sleep()
