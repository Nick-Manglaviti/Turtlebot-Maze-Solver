#! /usr/bin/env python
import rospy
from Node import Node
from utility import State, Intersection, Directions, reorient, generate_corners, is_inf
from graph_util import update_nodes, set_destination_to_path, update_connections, set_destination_to_unvisited_child
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
                robot.current_node = self.lookup_node(robot.get_coordinates(), robot.graph)
                if not robot.current_node.is_visited:
                    self.create_node(robot)
                update_connections(robot)
                set_destination_to_unvisited_child(robot)
            if robot.destination_node == None:
                robot.path = robot.graph.search_for_unvisited(robot.current_node)
                if (len(robot.path) == 0):
                    rospy.logerr("No Valid Nodes could be found.")
                    robot.cancel_scanner_check_action()
                else:
                    update_connections(robot)
                    set_destination_to_path(robot)
        robot.graph.display()
        rospy.logwarn("Intersection processing done.")
        robot.state = State.GOING_TO_HALLWAY
    
    """
    Search the graph for the node based on the x y values.
    Return the found node or a placeholder node.
    """
    def lookup_node(self, coordinates, graph):
        rospy.logwarn("Performing Lookup with x: " + str(coordinates[0]) + " y: " + str(coordinates[1]))
        searched_node = graph.search_by_position(coordinates[0], coordinates[1])
        if searched_node == None:
            searched_node = Node()
        return searched_node

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
        rospy.logwarn("Front: " + str(front))
        rospy.logwarn("Right: " + str(right))
        rospy.logwarn("Left: " + str(left))
        rospy.logwarn("Back: " + str(back))


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
            robot.make_realignment_request(orientation)
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
            index = (index - 360) / 4
            theta = robot.get_actual_orientation_in_degrees() + index
        position = robot.get_coordinates()
        x1 = position[0]
        y1 = position[1]

        delta_x = diagonal * math.cos(math.radians(theta))
        delta_y = diagonal * math.sin(math.radians(theta))

        x2 = x1 + delta_x
        y2 = y1 + delta_y

        rospy.logwarn("Top Corner is: " + str(x2) + str(y2))

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