#! /usr/bin/env python 

import rospy
from geometry_msgs.msg import Twist

class CmdVelPub(object):

    def __init__(self):
        self._publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._move_msg = Twist()
        self._linear_speed = 0.4
        self._angular_speed = 0.5

    def move_robot(self, direction):
        if direction == 'forward':
            self._forward()
        elif direction == 'left':
            self._left()
        elif direction == 'right':
            self._right()
        elif direction == 'stop':
            self._stop()
        else:
            rospy.logerr("No discernable direction passed.")
        self._publisher.publish(self._move_msg)

    def _forward(self):
        self._move_msg.linear.x = self._linear_speed
        self._move_msg.angular.z = 0
    def _left(self):
        self._move_msg.linear.x = 0
        self._move_msg.angular.z = self._angular_speed
    def _right(self):
        self._move_msg.linear.x = 0
        self._move_msg.angular.z = -self._angular_speed
    def _stop(self):
        self._move_msg.linear.x = 0
        self._move_msg.angular.z = 0


if __name__ == '__main__':
    rospy.init_node('cmd_vel_publisher_node')
    cmd_vel_pub_obj = CmdVelPublisher()
    rate = rospy.Rate(1)
    ctrl_c = False

    def shutdownhook():
        rospy.loginfo("cmd_vel_pub shutting down...")
        ctrl_c = True
        cmd_vel_pub_obj.move_robot(direction="stop")

    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        cmd_vel_pub_obj.move_robot(direction="forwards")
        rate.sleep()