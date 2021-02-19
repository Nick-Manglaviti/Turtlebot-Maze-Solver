#! /usr/bin/env python
import rospy
from Node import Node
from utility import State, Intersection, Directions, reorient, generate_corners, is_inf
import math

class BehaviorInIntersection(object):
    
    def __init__(self):
        pass

    def process(self, robot):
        rospy.logwarn("In Intersection")
        self.update_nodes(robot)
        if robot.is_on_path():
            rospy.logwarn("Continuing Path...")
            self.set_destination_to_path(robot)
        else:
            if not robot.current_node.is_visited:
                robot.current_node = self.lookup_node(robot.get_coordinates(), robot.graph)
                if not robot.current_node.is_visited:
                    rospy.logwarn("Creating Node...")
                    self.create_node(robot)
                self.update_connections(robot)
                self.set_destination_to_unvisited_child(robot)
            if robot.destination_node == None:
                robot.path = robot.graph.search_for_unvisited(robot.current_node)
                self.update_connections(robot)
                self.set_destination_to_path(robot)
        rospy.logwarn("Intersection processing done.")
        robot.graph.display()
        robot.state = State.GOING_TO_HALLWAY
    
    """
    Search the graph for the node based on the x y values.
    Return the found node or a placeholder node.
    """
    def lookup_node(self, coordinates, graph):
        rospy.logwarn("Performing Lookup with x=" + str(coordinates[0]) + " y=" + str(coordinates[1]))
        searched_node = graph.search_by_position(coordinates[0], coordinates[1])
        if searched_node == None:
            searched_node = Node()
        return searched_node
    
    """
    Pop from the robot's path
    and set it to destination
    """
    def set_destination_to_path(self, robot):
        rospy.logwarn("Creating Path.")
        robot.destination_node = robot.path.pop()
        for i in range(len(robot.current_node.directions)):
            if robot.current_node.directions[i] == None:
                pass
            elif robot.current_node.directions[i] == robot.destination_node:
                robot.orientation_directed = robot.directed_paths[i]
        robot.previous_orientation = robot.orientation_directed
    
    """
    If there is an unvisited child at current node
    set path and destination to it.
    """
    def set_destination_to_unvisited_child(self, robot):
        rospy.logwarn("Looking for nearest unvisited Node.")
        rospy.logwarn("Orientation =" + str(robot.orientation_directed))
        for i in range(len(robot.directed_paths)):
            rospy.logwarn("Directed Paths:" + str(robot.directed_paths[i]))
            if robot.current_node.directions[i] == None:
                continue
            elif not robot.current_node.directions[i].is_visited:
                robot.orientation_directed = robot.directed_paths[i]
                robot.destination_node = robot.current_node.directions[i]
        robot.previous_orientation = robot.orientation_directed

    """
    Generate a new node based on the current orientation, 
    set it to visited, add the possible directional paths,
    calculate x and y ranges. Add/Set node to graph and current.
    """
    def create_node(self, robot):
        new_node = Node()
        new_node.set_visited(True)
        front = robot.orientation_directed
        right = reorient(front, -90)
        left = reorient(front, 90)
        back = reorient(front, 180)

        for i in range(len(robot.directed_paths)):
            if robot.directed_paths[i] == front:
                if robot.get_laser_scan()[360] > robot.node_size or is_inf(robot.get_laser_scan()[360]):
                    new_node.directions[i] = Node()
                    rospy.logwarn("Added: Forward Path.")
            if robot.directed_paths[i] == left:
                if robot.get_laser_scan()[719] > robot.node_size or is_inf(robot.get_laser_scan()[719]):
                    new_node.directions[i] = Node()
                    rospy.logwarn("Added: Left Path.")
            if robot.directed_paths[i] == right:
                if robot.get_laser_scan()[0] > robot.node_size or is_inf(robot.get_laser_scan()[0]):
                    new_node.directions[i] = Node()
                    rospy.logwarn("Added: Right Path.")
        rospy.logwarn("Finished Setting Pathways.")
        self.set_xy_ranges(robot, self.get_intersection_type(robot), new_node)
        
        robot.graph.add_node(new_node)
        robot.current_node = new_node

 
    """
    Shift the nodes: current -> previous
    destination -> current and empty out destination
    """
    def update_nodes(self, robot):
        robot.previous_node = robot.current_node
        robot.current_node = robot.destination_node
        robot.destination_node = None

    """
    Add the connection from the previous to the current
    and vice versa. Change previous orientation
    """
    def update_connections(self, robot):
        from_direction = reorient(robot.orientation_directed, 180)
        for i in range(len(robot.directed_paths)):
            if robot.directed_paths[i] == from_direction:
                robot.current_node.directions[i] = robot.previous_node
            if robot.directed_paths[i] == robot.previous_orientation:
                robot.previous_node.directions[i] = robot.current_node

    """
    Depending on the intersection type get the top left or right
    corner and generate all corners within it. 
    """
    def set_xy_ranges(self, robot, intersection_type, node):
        orientation = robot.orientation_directed
        if intersection_type == Intersection.LFR:
            side = Directions.RIGHT
        if intersection_type == Intersection.LF:
            side = Directions.LEFT
        if intersection_type == Intersection.FR:
            side = Directions.RIGHT
        if intersection_type == Intersection.LR:
            orientation = reorient(robot.orientation_directed, 180)
            side = Directions.RIGHT
            robot.make_realignment_request(turned_around)
        corner = self.calculate_top_corner(robot, side)
        robot.make_realignment_request(robot.orientation_directed)
        corners = generate_corners(corner, side, orientation, robot.node_size)
        
        node.x_lower = corners[0][0]
        node.x_upper = corners[0][0]
        node.y_lower = corners[0][1]
        node.y_upper = corners[0][1]
        for i in corners:
            if i[0] < node.x_lower:
                node.x_lower = i[0]
            if i[0] > node.x_upper:
                node.x_upper = i[0]
            if i[1] < node.y_lower:
                node.y_lower = i[1]
            if i[1] > node.y_upper:
                node.y_upper = i[1]

    """
    Calculate a top corner of the node
    either on the left (0) or right (1)
    side. Return the corner.
    """
    def calculate_top_corner(self, robot, side):
        data = robot.get_laser_scan()
        index = 0
        theta = 0
        if side == Directions.RIGHT:
            diagonal = data[180]
            for i in range(1, 360):
                if is_inf(data[i]):
                    continue
                if diagonal > data[i]:
                    diagonal = data[i]
                    index = i
            index = index / 4
            theta = robot.get_actual_orientation_in_degrees() - (90 - index)
        else:
            diagonal = data[540]
            for i in range(360, 719):
                if is_inf(data[i]):
                    continue
                if diagonal > data[i]:
                    diagonal = data[i]
                    index = i
            index = index / 4
            theta = robot.get_actual_orientation_in_degrees() + (90 - index)
        position = robot.get_coordinates()
        x1 = position[0]
        y1 = position[1]

        delta_x = diagonal * math.cos(math.radians(theta))
        delta_y = diagonal * math.sin(math.radians(theta))

        x2 = x1 + delta_x
        y2 = y1 + delta_y

        return [x2, y2]

    """
    At the current orientation,
    get the shape of the intersection
    assuming behind is a pathway.
    Return the type.
    """
    def get_intersection_type(self, robot):
        intersection_type = None
        if robot.get_laser_scan()[0] > robot.node_size:
            if robot.get_laser_scan()[719] > robot.node_size:
                if robot.get_laser_scan()[360] > robot.node_size:
                    intersection_type = Intersection.LFR
                else:
                    intersection_type = Intersection.LR
            else: 
                intersection_type = Intersection.FR
        else:
            intersection_type = Intersection.LF

        return intersection_type