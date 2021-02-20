#! /usr/bin/env python
import rospy
from utility import State
from nav_util import reorient
import time

class BehaviorInHallway(object):

    def __init__(self):
        pass

    def process(self, robot):
        robot.go_forward()
        robot.check_alignment()
        robot.make_crash_detection_request()
        self.check_lfr(robot)
    
    def check_lfr(self, robot):
        if robot.left_open() or robot.right_open() or robot.approching_wall():
            robot.go_forward()
            time.sleep(1)
            robot.stop()
            time.sleep(1)
            
            if robot.left_open():
                if robot.forward_open():
                    if robot.right_open():
                        rospy.logwarn("Left Forward Right Opening.")
                        robot.state = State.IN_INTERSECTION
                    else:
                        rospy.logwarn("Left Forward Opening.")
                        robot.state = State.IN_INTERSECTION
                else:
                    if robot.right_open():
                        rospy.logwarn("Left Right Opening.")
                        robot.state = State.IN_INTERSECTION
                    else:
                        rospy.logwarn("Left Opening.")
                        robot.orientation_directed = reorient(robot.orientation_directed, 90)
                        robot.state = State.GOING_TO_HALLWAY

            elif robot.right_open():
                if robot.forward_open():
                    rospy.logwarn("Right Forward Opening.")
                    robot.state = State.IN_INTERSECTION
                else:
                    rospy.logwarn("Right Opening.")
                    robot.orientation_directed = reorient(robot.orientation_directed, -90)
                    robot.state = State.GOING_TO_HALLWAY

            elif not robot.forward_open():
                rospy.logwarn("Dead End.")
                robot.state = State.AT_DEAD_END   

    
