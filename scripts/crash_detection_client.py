#! /usr/bin/env python

import rospy
from utility import Services
from std_srvs.srv import Trigger, TriggerRequest

class CrashDetectionClient(object):

    def __init__(self):
        service_name = Services.CRASH_DETECTION_SERVER.value
        rospy.loginfo('Robot waiting for ' + service_name)
        rospy.wait_for_service(service_name)
        rospy.loginfo('Found!')

        self.request = TriggerRequest()
        self.connection = rospy.ServiceProxy(service_name, Trigger)

    def send_request(self):
        response = self.connection(self.request)
        return response