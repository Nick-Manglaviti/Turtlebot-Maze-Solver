#! /usr/bin/env python

import rospy
from utility import Services
from turtlebot_maze_solver.srv import AlignmentTarget, AlignmentTargetRequest

class RealignmentClient(object):

    def __init__(self):
        service_name = Services.REALIGNMENT_SERVER.value
        rospy.loginfo('Robot waiting for ' + service_name)
        rospy.wait_for_service(service_name)
        rospy.loginfo('Found!')

        self.request = AlignmentTargetRequest()
        self.connection = rospy.ServiceProxy(service_name, AlignmentTarget)
    
    def send_request(self, target):
        self.request.target = target
        response = self.connection(self.request)
        return response
