#! /usr/bin/env python

import rospy
from utility import get_turn_direction, get_degrees, Services
from turtlebot_maze_solver.srv import AlignmentTarget, AlignmentTargetResponse

from odom_sub import OdomTopicSub
from cmd_vel_pub import CmdVelPub

class RealignmentService(object):

    def __init__(self):
        self._srv_name = Services.REALIGNMENT_SERVER.value
        self._odom_reader = OdomTopicSub()
        self._cmd_pub_obj = CmdVelPub()
        self._service = rospy.Service(self._srv_name, AlignmentTarget, self.srv_callback)
        self._left = "left"
        self._right = "right"
        self._stop = "stop"


    def srv_callback(self, request):
        response = AlignmentTargetResponse()
        flag = True
        while (flag):
            current = get_degrees(self._odom_reader.get_odom())
            result = get_turn_direction(current, request.target)
            if result == 0:
                rospy.logdebug("Turning Done.")
                flag = False
            elif result > 0:
                self._cmd_pub_obj.move_robot(self._right)
                rospy.logdebug("Turning ==> " + self._right)
            elif result < 0:
                self._cmd_pub_obj.move_robot(self._left)
                rospy.logdebug("Turning ==> " + self._left)
        self._cmd_pub_obj.move_robot(self._stop)
        response.success = True

        return response

if __name__ == "__main__":
    rospy.init_node('realignment_service_server')
    align_serv_obj = RealignmentService()
    rospy.spin()