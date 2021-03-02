#! /usr/bin/env python

import rospy
import actionlib
from utility import Actions
from turtlebot_maze_solver.msg import ScannerCheckAction, ScannerCheckGoal

class ScannerCheckClient(object):

    def __init__(self):
        action_name = action_name = Actions.SCANNER_CHECK.value
        self.client = actionlib.SimpleActionClient(action_name, ScannerCheckAction)
        rospy.loginfo('Robot waiting for ' + action_name)
        self.client.wait_for_server()
        rospy.loginfo('Found!')
        self.goal = ScannerCheckGoal()

    def send_goal(self):
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)
    
    def cancel(self):
        self.client.cancel_goal()

    def in_progress(self):
        in_progress = (self.client.get_state() < 2)
        return in_progress

    def feedback_callback(self, feedback):
        rospy.loginfo("Scanning...")

    def get_result(self):
        return self.client.get_result()