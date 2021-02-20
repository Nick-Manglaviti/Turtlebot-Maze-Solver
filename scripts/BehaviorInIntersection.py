#! /usr/bin/env python
import rospy
from Node import Node
from utility import State, Intersection, Directions
from graph_util import create_node
from nav_util import update_nodes, set_destination_to_path, set_destination_to_unvisited_child, lookup_node, create_and_set_path
import math

class BehaviorInIntersection(object):
    
    def __init__(self):
        pass

    def process(self, robot):
        rospy.logwarn("In Intersection")
        update_nodes(robot)
        if robot.is_on_path():
            set_destination_to_path(robot)
        else:
            if not robot.current_node.is_visited:
                searched_node = lookup_node(robot.get_coordinates(), robot.graph)
                if searched_node != None:
                    robot.current_node = searched_node
                if not robot.current_node.is_visited:
                    create_node(robot)
                set_destination_to_unvisited_child(robot)
            if robot.destination_node == None:
                create_and_set_path(robot)
        robot.graph.display()
        rospy.logwarn("Intersection processing done.")
        robot.state = State.GOING_TO_HALLWAY