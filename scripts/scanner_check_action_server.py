#! /usr/bin/env python

import rospy
import actionlib
from turtlebot_maze_solver.msg import ScannerCheckAction, ScannerCheckFeedback, ScannerCheckResult
from laser_sub import LaserTopicSub
from sensor_msgs.msg import LaserScan
from utility import Actions

class ScannerCheckClass(object):
    def __init__(self):
        self._as = actionlib.SimpleActionServer(Actions.SCANNER_CHECK.value, ScannerCheckAction, self.callback, False)
        self._as.start()
        self._laser_reader_object = LaserTopicSub()
        self._feedback = ScannerCheckFeedback()
        self._result = ScannerCheckResult()

    def callback(self, goal):
        success = True
        rate = rospy.Rate(1)

        while (success):
            if self._as.is_preempt_requested():
                rospy.loginfo("Goal Cancelled.")
                self._as.set_preempted()
                success = False
                break
            
            if not self.check_if_out_maze(self._laser_reader_object.get_laser_data().ranges):
                self._as.publish_feedback(self._feedback)
            else:
                rospy.logwarn('Reached Goal.')
                break
            rate.sleep()
        
        # Goal was either achieved or cancelled
        self._result.success = success
        if success:
            self._as.set_succeeded(self._result)
        self.clean_variables()
    
    def check_if_out_maze(self, laser_scan_array):
        flag = True
        if len(laser_scan_array) < 718:
            flag = False 
        for i in range(len(laser_scan_array)):
            if str(laser_scan_array[i]) != "inf":
                self._feedback.value = i
                flag = False
                break
        return flag

    def clean_variables(self):
        self._feedback = ScannerCheckFeedback()
        self._result = ScannerCheckResult()

if __name__ == '__main__':
    rospy.init_node('scanner_check_action_server_node')
    ScannerCheckClass()
    rospy.spin()