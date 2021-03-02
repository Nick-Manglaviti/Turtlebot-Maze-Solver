#! /usr/bin/env python 

from utility import State

class BehaviorMoveToHallway(object):

    def __init__(self):
        pass

    def process(self, robot):
        robot.check_alignment()
        robot.crash_detection_client.send_request()
        self.check_if_in_hallway(robot)
        robot.go_forward()
    
    def check_if_in_hallway(self, robot):
        if robot.get_laser_scan()[0] < robot.node_size:
            if robot.get_laser_scan()[719] < robot.node_size:
                robot.state = State.IN_HALLWAY
