#! /usr/bin/env python

import rospy

from std_srvs.srv import Trigger, TriggerResponse
from turtlebot_maze_solver.srv import AlignmentTarget, AlignmentTargetRequest
from utility import get_degrees, Services

from laser_sub import LaserTopicSub
from cmd_vel_pub import CmdVelPub
from odom_sub import OdomTopicSub

class CrashDetectionService(object):

    def __init__(self):
        self._srv_name = Services.CRASH_DETECTION_SERVER.value
        self._service = rospy.Service(self._srv_name, Trigger, self.srv_callback)

        self._laser_reader = LaserTopicSub()
        self._cmd_pub = CmdVelPub()
        self._odom_reader = OdomTopicSub()

        self._realignment_service = None
        self._realignment_request = AlignmentTargetRequest()
        self.init_realignment_service_client()

        self._rate = rospy.Rate(1)
        self.crash_distance = .45

    def srv_callback(self, request):
        laser_scan_size = len(self._laser_reader.get_laser_data().ranges)
        batch = laser_scan_size // 10
        response = TriggerResponse()
        stopped = False
        for i in range(batch + 1):
            index = i * 10
            if index > 719:
                index = 719
            if self._laser_reader.get_laser_data().ranges[index] < self.crash_distance:
                self._cmd_pub.move_robot("stop")
                self._rate.sleep()
                rospy.logwarn('Crash incoming at index: ' + str(index) + ' Value: ' + str(self._laser_reader.get_laser_data().ranges[index]))
                stopped = True
                right_side = laser_scan_size // 4
                left_side = right_side * 3
                old_orientation = get_degrees(self._odom_reader.get_odom())

                if (index > left_side):
                    target = old_orientation - 90
                    self._adjust(old_orientation, target)
                elif (index < right_side):
                    target = old_orientation + 90
                    self._adjust(old_orientation, target)
                elif (self._laser_reader.get_laser_data().ranges[left_side] < self._laser_reader.get_laser_data().ranges[right_side]):
                    target = old_orientation - 90
                    self._adjust(old_orientation, target)
                else:
                    target = old_orientation + 90
                    self._adjust(old_orientation, target)
                break
        if stopped:
            self._cmd_pub.move_robot('forward')
        
        response.success = True
        return response

    def _adjust(self, old_orientation, target):
        self._realignment_request.target = target
        self._realignment_service(self._realignment_request)
        self._cmd_pub.move_robot('forward')
        i = 2
        while (i != 0):
            self._rate.sleep()
            i -= 1
        self._cmd_pub.move_robot("stop")
        self._realignment_request.target = old_orientation
        self._realignment_service(self._realignment_request)
        
    
    def init_realignment_service_client(self, service_name = "/realignment_service_server"):
        rospy.loginfo(self._srv_name + ' waiting for realignment_service_server')
        rospy.wait_for_service(service_name)
        rospy.loginfo('realignment_service_server Found...')
        self._realignment_service = rospy.ServiceProxy(service_name, AlignmentTarget)

if __name__ == "__main__":
    rospy.init_node('crash_detection_service_server_node')
    dir_serv_obj = CrashDetectionService()
    rospy.spin()