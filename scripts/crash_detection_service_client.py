#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerRequest

class CrashDetectionServiceClient(object):
    rospy.init_node('crash_direction_service_client_node')
    service_name = '/crash_direction_service'
    rospy.wait_for_service(service_name)
    connection = rospy.ServiceProxy(service_name, Trigger)
    request = TriggerRequest()



rospy.on_shutdown(shutdownhook)

