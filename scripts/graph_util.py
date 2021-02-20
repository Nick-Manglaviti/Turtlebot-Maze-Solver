#! /usr/bin/env python
import rospy
import math
from Node import Node
from utility import Intersection, Directions, is_inf
from nav_util import reorient

"""
Generate a new node based on the current orientation, 
set it to visited, add the possible directional paths,
calculate x and y ranges. Add/Set node to graph and current.
"""
def create_node(robot):
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
    set_xy_ranges(robot, get_intersection_type(robot), new_node)
        
    robot.graph.add_node(new_node)
    robot.current_node = new_node

"""
Depending on the intersection type get the top left or right
corner and generate all corners within it. 
"""
def set_xy_ranges(robot, intersection_type, node):
    side = Directions.RIGHT
    if intersection_type == Intersection.LF:
        side = Directions.LEFT
    orientation = robot.orientation_directed
    if intersection_type == Intersection.LR:
        orientation = reorient(robot.orientation_directed, 180)
        robot.make_realignment_request(orientation)
    corner = calculate_top_corner(robot, intersection_type)
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
def calculate_top_corner(robot, intersection_type):
    side = Directions.RIGHT
    if intersection_type == Intersection.LF:
        side = Directions.LEFT
    data = robot.get_laser_scan()
    index = 0
    theta = 0
    if intersection_type == Intersection.DE:
        diagonal = data[180]
        for i in range(1, 360):
            if is_inf(data[i]):
                continue
            if diagonal < data[i]:
                diagonal = data[i]
                index = i
        index = index / 4
        theta = robot.get_actual_orientation_in_degrees() - (90 - index)
    elif side == Directions.RIGHT:
        diagonal = data[180]
        for i in range(1, 360):
            if is_inf(data[i]):
                continue
            if diagonal > data[i]:
                diagonal = data[i]
                index = i
        index = index / 4
        theta = robot.get_actual_orientation_in_degrees() - (90 - index)
    elif side == Directions.LEFT:
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
Generate 4 points of a square given a corner and directed angle.
Returns an array of the 4 points in order: 
top_left, top_right, bottom_left, bottom_right
"""
def generate_corners(corner, point_side, directed_angle, side_length):
    rospy.logwarn("Side: " + str(point_side) + " Angle: " + str(directed_angle) + " length: " + str(side_length))
    # Normalized the angles to generate the points
    vertical_angle = directed_angle + 180
    delta_x_vertical = math.cos(math.radians(vertical_angle)) * side_length
    delta_y_vertical = math.sin(math.radians(vertical_angle)) * side_length
    horizontal_angle = directed_angle + 90
    delta_x_horizontal = math.cos(math.radians(horizontal_angle)) * side_length
    delta_y_horizontal = math.sin(math.radians(horizontal_angle)) * side_length

    if point_side == Directions.LEFT:
        top_left = corner
        top_right = [top_left[0] - delta_x_horizontal, top_left[1] - delta_y_horizontal]
    else:
        top_right = corner
        top_left = [top_right[0] + delta_x_horizontal, top_right[1] + delta_y_horizontal]
    bottom_left = [top_left[0] + delta_x_vertical, top_left[1] + delta_y_vertical]
    bottom_right = [top_right[0] + delta_x_vertical, top_right[1] + delta_y_vertical]

    top_left = [round(top_left[0], 4), round(top_left[1], 4)]
    top_right = [round(top_right[0], 4), round(top_right[1], 4)]
    bottom_left = [round(bottom_left[0], 4), round(bottom_left[1], 4)]
    bottom_right = [round(bottom_right[0], 4), round(bottom_right[1], 4)]
    
    return [top_left, top_right, bottom_left, bottom_right]

"""
At the current orientation,
get the shape of the intersection
assuming behind is a pathway.
Return the type.
"""
def get_intersection_type(robot):
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