#! /usr/bin/env python

from Node import Node

class Graph(object):

    def __init__(self):
        self.nodes = {}

    def add_node(self, node):
        node.set_id(len(self.nodes))
        node.set_visited(True)
        self.nodes.update({node.get_id(): node})
        return node

    def search_by_position(self, x, y):
        for i in self.nodes:
            if self.nodes[i].x_lower == None:
                continue
            if (self.nodes[i].x_lower < x < self.nodes[i].x_upper) and (self.nodes[i].y_lower < y < self.nodes[i].y_upper):
                return self.nodes[i]
        return None

    @staticmethod
    def search_for_unvisited(node):
        source = node
        level = {node: None}
        parent = {node: None}
        frontier = [node]
        i = 1
        destination = None
        while (len(frontier) > 0) and (destination is None):
            externals = []
            for u in frontier:
                for v in u.directions:
                    if (v not in level) and (v is not None):
                        level[v] = i
                        parent[v] = u
                        externals.append(v)
                        if not v.is_visited:
                            destination = u
                if destination is not None:
                    break
            frontier = externals
            i += 1
        stack = []
        if destination != None:
            while destination is not source:
                stack.append(destination)
                destination = parent.get(destination)
        return stack

    def display(self):
        for i in self.nodes:
            print("Node " + str(self.nodes[i].get_id()))
            print(str(self.nodes[i].get_connections()))
            print(str(self.nodes[i].get_ranges()))

if __name__ == "__main__":
    graph = Graph()
    node_zero = Node()
    node_one = Node()
    node_two = Node()
    node_three = Node()

    node_zero.is_visited = True
    node_one.is_visited = True
    node_two.is_visited = True

    node_zero.set_directions(node_one, None, None, None)
    node_one.set_directions(node_two, node_zero, node_two, node_two)
    node_two.set_directions(node_three, node_one, node_one, node_one)

    graph.add_node(node_zero)
    graph.add_node(node_one)
    graph.add_node(node_two)

    path = graph.search_for_unvisited(node_one)
    
    
