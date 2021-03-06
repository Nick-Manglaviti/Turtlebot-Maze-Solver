#! /usr/bin/env python

import rospy
from Node import Node
from BehaviorInHallway import BehaviorInHallway
from BehaviorMoveToHallway import BehaviorMoveToHallway
from BehaviorInIntersection import BehaviorInIntersection
from BehaviorDeadEnd import BehaviorDeadEnd
from utility import State
from Turtlebot import Turtlebot



if __name__ == "__main__":
    rospy.init_node("turtlebot_controller_node")
    rate = rospy.Rate(1)

    robot = Turtlebot()
    robot.scanner_check_client.send_goal()

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
    rate.sleep()

    while robot.scanner_check_client.in_progress():
        if (robot.state == State.IN_INTERSECTION):
            in_intersection.process(robot)
        elif (robot.state == State.IN_HALLWAY):
            in_hallway.process(robot)
        elif (robot.state == State.GOING_TO_HALLWAY):
            move_to_hallway.process(robot)
        elif (robot.state == State.AT_DEAD_END):
            at_dead_end.process(robot)
    robot.stop()
    robot.graph.display()
    rospy.loginfo("Turtlebot Maze Test Finished")