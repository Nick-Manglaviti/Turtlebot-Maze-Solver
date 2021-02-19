#! /usr/bin/env python

import rospy
from srv import AlignmentTarget, AlignmentTargetRequest

rospy.init_node('realignment_service_client', log_level.INFO)
service_name = '/realignment_service_server'
rospy.wait_for_service(service_name)
connection = rospy.ServiceProxy(service_name, AlignmentTarget)
request = AlignmentTargetRequest()

rate = rospy.Rate(5)

ctrl_c = False

def send_request(target):
    request.target = target
    result = connection(request)
    if result.success:
        rospy.logwarn("Success is " + str(result.success))
        rospy.logwarn("Going ==>" + str(result.success))
    else:
        rospy.loginfo("Success is " + str(result.success))
        rospy.loginfo("Going ==>" + str(result.success))

def shutdownhook():
    rospy.loginfo('direction_service_client shutting down...')
    ctrl_c = True

rospy.on_shutdown(shutdownhook)
