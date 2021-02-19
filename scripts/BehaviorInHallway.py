#! /usr/bin/env python
import rospy
from utility import State, reorient
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
        if robot.get_laser_scan()[0] > robot.node_size:
            time.sleep(1)
            robot.stop()
            if robot.get_laser_scan()[719] > robot.node_size:
                rospy.logwarn("Right Left Opening.")
                robot.state = State.IN_INTERSECTION
            elif robot.get_laser_scan()[360] > robot.node_size:
                rospy.logwarn("Right Forward Opening.")
                robot.state = State.IN_INTERSECTION
            else:
                robot.orientation_directed = reorient(robot.orientation_directed, -90)
                rospy.logwarn("Right Opening.")
                robot.state = State.GOING_TO_HALLWAY
        if robot.get_laser_scan()[719] > robot.node_size:
            time.sleep(1)
            robot.stop()
            if robot.get_laser_scan()[0] > robot.node_size:
                robot.state = State.IN_INTERSECTION
            elif robot.get_laser_scan()[360] > robot.node_size:
                robot.state = State.IN_INTERSECTION
            else:
                robot.orientation_directed = reorient(robot.orientation_directed, 90)
                rospy.logwarn("Left Opening.")
                robot.state = State.GOING_TO_HALLWAY
        if robot.get_laser_scan()[360] < robot.node_size:
            time.sleep(1)
            robot.stop()
            if robot.get_laser_scan()[0] < robot.node_size:
                if robot.get_laser_scan()[719] < robot.node_size:
                    rospy.logwarn("Dead End.")
                    robot.state = State.AT_DEAD_END