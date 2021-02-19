#! /usr/bin/env python

import rospy
import actionlib
import time
from Node import Node
from BehaviorInHallway import BehaviorInHallway
from BehaviorMoveToHallway import BehaviorMoveToHallway
from BehaviorInIntersection import BehaviorInIntersection
from BehaviorDeadEnd import BehaviorDeadEnd
from utility import State
from Turtlebot import Turtlebot
from turtlebot_maze_escape.srv import AlignmentTarget, AlignmentTargetRequest



if __name__ == "__main__":
    rospy.init_node("turtlebot_controller_node")
    rate = rospy.Rate(10)

    robot = Turtlebot()
    robot.send_goal_to_scanner_check_action_server()

    # Init Starting Node/Graph
    start_node = Node()
    start_node.set_directions(None, None, None, None)
    start_node.set_visited(True)
    robot.current_node = start_node
    robot.graph.add_node(robot.current_node)
    robot.state = State.IN_HALLWAY

    in_hallway = BehaviorInHallway()
    move_to_hallway = BehaviorMoveToHallway()
    in_intersection = BehaviorInIntersection()
    at_dead_end = BehaviorDeadEnd()

    while robot.scanner_check_in_progress():
        while (robot.state == State.IN_HALLWAY):
            in_hallway.process(robot)
        while (robot.state == State.GOING_TO_HALLWAY):
            move_to_hallway.process(robot)
        while (robot.state == State.IN_INTERSECTION):
            in_intersection.process(robot)
        while (robot.state == State.AT_DEAD_END):
            at_dead_end.process(robot)
    
    robot.graph.display()
    rospy.loginfo("Turtlebot Maze test Finished")