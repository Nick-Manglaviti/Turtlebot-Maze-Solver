#! /usr/bin/env python

import rospy
from Node import Node
from Graph import Graph
import actionlib
import time
from utility import State, Services, Actions, get_degrees, get_turn_direction, get_direction_mapping
from cmd_vel_pub import CmdVelPub
from odom_sub import OdomTopicSub
from laser_sub import LaserTopicSub

from turtlebot_maze_solver.msg import ScannerCheckAction, ScannerCheckGoal
from turtlebot_maze_solver.srv import AlignmentTarget, AlignmentTargetRequest
from std_srvs.srv import Trigger, TriggerRequest

class Turtlebot(object):

    def __init__(self, node_size = 1.45):
        rate = rospy.Rate(5)
        self._cmd_pub = CmdVelPub()
        rate.sleep()
        self._odom_reader = OdomTopicSub()
        rate.sleep()
        self._laser_reader = LaserTopicSub()
        while len(self.get_laser_scan()) != 720:
            rate.sleep()
        rate.sleep()
        
        self.previous_node = None
        self.current_node = Node()
        self.destination_node = Node()
        self.path = []
        self.graph = Graph()
        self.node_size = node_size
        orientation = round(get_degrees(self._odom_reader.get_odom()))
        self.orientation_directed = orientation
        self.previous_orientation = orientation
        self.directed_paths = get_direction_mapping(orientation)
        self.state = None

        self._realignment_service_connection = None
        self._realignment_request = AlignmentTargetRequest()

        self._crash_detection_service_connection = None
        self._crash_detection_request = TriggerRequest()

        self._scanner_check_action_client = None
        self._scanner_check_goal = ScannerCheckGoal()
        
        self._init_realignment_service_client()
        self._init_crash_detection_service_client()
        self._init_scanner_check_client()
    
    def _init_realignment_service_client(self, service_name = Services.REALIGNMENT_SERVER.value):
        rospy.loginfo('Robot waiting for ' + service_name)
        rospy.wait_for_service(service_name)
        rospy.loginfo('Found!')
        self._realignment_service_connection = rospy.ServiceProxy(service_name, AlignmentTarget)
    
    def _init_crash_detection_service_client(self, service_name = Services.CRASH_DETECTION_SERVER.value):
        rospy.loginfo('Robot waiting for ' + service_name)
        rospy.wait_for_service(service_name)
        rospy.loginfo('Found!')
        self._crash_detection_service_connection = rospy.ServiceProxy(service_name, Trigger)

    def _init_scanner_check_client(self, action_name = Actions.SCANNER_CHECK.value):
        self._scanner_check_action_client = actionlib.SimpleActionClient(action_name, ScannerCheckAction)
        rospy.loginfo('Robot waiting for ' + action_name)
        self._scanner_check_action_client.wait_for_server()
        rospy.loginfo('Found!')
    
    def send_goal_to_scanner_check_action_server(self):
        self._scanner_check_action_client.send_goal(self._scanner_check_goal, feedback_cb=self.scanner_check_feedback_callback)
    
    def cancel_scanner_check_action(self):
        self._scanner_check_action_client.cancel_goal()

    def scanner_check_in_progress(self):
        in_progress = (self._scanner_check_action_client.get_state() < 2)
        return in_progress

    def scanner_check_feedback_callback(self,feedback):
        rospy.loginfo("Scanner Check Feedback ==> " + str(feedback))

    def get_result_scanner_check(self):
        return self._scanner_check_action_client.get_result()

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

    def make_realignment_request(self, target):
        self._realignment_request.target = target
        response = self._realignment_service_connection(self._realignment_request)
        return response

    def make_crash_detection_request(self):
        response = self._crash_detection_service_connection(self._crash_detection_request)
        return response
    
    def is_on_path(self):
        if len(self.path) > 0:
            return True
        else:
            return False

    def check_alignment(self):
        value = get_turn_direction(self.get_actual_orientation_in_degrees(), self.orientation_directed)
        if value != 0:
            rospy.loginfo("Robot Requesting Realignment")
            self.make_realignment_request(self.orientation_directed)
    
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