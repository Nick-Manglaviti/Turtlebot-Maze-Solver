#! /usr/bin/env python

class Node(object):

    def __init__(self):
        self.node_id = None
        self.x_lower = None
        self.x_upper = None
        self.y_lower = None
        self.y_upper = None
        self.directions = [None, None, None, None]
        self.is_visited = False

    def set_ranges(self, x_lower, x_upper, y_lower, y_upper):
        self.x_lower = x_lower
        self.x_upper = x_upper
        self.y_lower = y_lower
        self.y_upper = y_upper

    def set_directions(self, north, south, east, west):
        self.directions = [north, south, east, west]

    def set_visited(self, value):
        self.is_visited = value

    def set_id(self, x):
        self.node_id = x

    def get_id(self):
        if self.node_id is not None:
            return self.node_id
        else:
            return None

    def get_connections(self):
        if self.node_id is not None:
            s = "Connections: "
            for i in range(0, 4):
                if self.directions[i] is not None:
                    if i == 0:
                        s += "North->"
                    if i == 1:
                        s += "South->"
                    if i == 2:
                        s += "East->"
                    if i == 3:
                        s += "West->"
                    if self.directions[i].get_id() is not None:
                        s += str(self.directions[i].get_id()) + " "
                    else:
                        s += "Unvisited "
            return s
        else:
            s = "Unvisited"
        return s

    def get_ranges(self):
        s = "Min X: " + str(self.x_lower ) + " Max X: " + str(self.x_upper )
        s += "Min Y: " + str(self.y_lower ) + " Max Y: " + str(self.y_upper )
        return s