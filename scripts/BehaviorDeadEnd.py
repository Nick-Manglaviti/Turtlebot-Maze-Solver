#! /usr/bin/env python

from utility import State, Intersection
from graph_util import update_nodes, update_connections, set_destination_to_path, set_destination_to_unvisited_child


class BehaviorDeadEnd(object):

    def __init__(self):
        pass

    def process(self, robot):
        update_nodes(robot)
        # create dead node
        update_connections(robot)
        set_destination_to_path(robot)