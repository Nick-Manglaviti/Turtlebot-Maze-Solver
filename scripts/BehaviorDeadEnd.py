#! /usr/bin/env python

from utility import State, Intersection
from nav_util import update_nodes, set_destination_to_path, create_and_set_path
from graph_util import create_node

class BehaviorDeadEnd(object):

    def __init__(self):
        pass

    def process(self, robot):
        update_nodes(robot)
        create_node(robot)
        create_and_set_path(robot)
        robot.graph.display()
        robot.state = State.IN_HALLWAY