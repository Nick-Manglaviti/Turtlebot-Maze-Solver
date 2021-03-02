#! /usr/bin/env python

import rospy

from Node import Node
from Graph import Graph

from realignment_client import RealignmentClient
from crash_detection_client import CrashDetectionClient
from scanner_check_client import ScannerCheckClient

from cmd_vel_pub import CmdVelPub
from odom_sub import OdomTopicSub
from laser_sub import LaserTopicSub

from utility import State, get_degrees
from nav_util import get_turn_direction, get_direction_mapping


class Turtlebot(object):

    def __init__(self):
        self._cmd_pub = CmdVelPub()
        self._odom_reader = OdomTopicSub()
        self._laser_reader = LaserTopicSub()
        
        self.previous_node = None
        self.current_node = Node()
        self.destination_node = Node()
        self.path = []
        self.graph = Graph()
        self.node_size = rospy.get_param("/turtlebot_maze_solver/node_size")
        orientation = round(get_degrees(self._odom_reader.get_odom()))
        self.orientation_directed = orientation
        self.previous_orientation = orientation
        self.directed_paths = get_direction_mapping(orientation)
        self.state = None

        self.realignment_client = RealignmentClient()
        self.crash_detection_client = CrashDetectionClient()
        self.scanner_check_client = ScannerCheckClient()
        
    
    def check_alignment(self):
        value = get_turn_direction(self.get_actual_orientation_in_degrees(), self.orientation_directed)
        if value != 0:
            rospy.loginfo("Robot Requesting Realignment")
            self.realignment_client.send_request(self.orientation_directed)

    def go_forward(self):
        self._cmd_pub.move_robot("forward")
    
    def stop(self):
        self._cmd_pub.move_robot("stop")

    def get_actual_orientation_in_degrees(self):
        return get_degrees(self._odom_reader.get_odom())

    def get_coordinates(self):
        msg = self._odom_reader.get_odom()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        position = [x, y]
        rospy.loginfo("Position: [" + str(position[0]) + ", " + str(position[1]) + "]")
        return position

    def get_laser_scan(self):
        return self._laser_reader.get_laser_data().ranges
    
    def is_on_path(self):
        if len(self.path) > 0:
            return True
        else:
            return False
    
    def approching_wall(self):
        if self.get_laser_scan()[360] < (self.node_size / 2):
            return True
        else:
            return False

    def left_open(self):
        if self.get_laser_scan()[719] > self.node_size:
            return True
        else:
            return False

    def forward_open(self):
        if self.get_laser_scan()[360] > self.node_size:
            return True
        else:
            return False

    def right_open(self):
        if self.get_laser_scan()[0] > self.node_size:
            return True
        else:
            return False

if __name__ == "__main__":
    rospy.init_node('turtlebot_node')
    turtle = Turtlebot()