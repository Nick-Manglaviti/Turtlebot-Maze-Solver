#! /usr/bin/env python

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from enum import Enum
import math
import rospy

"""
Enum used for States 
"""
class State(Enum):
    IN_HALLWAY = 0
    GOING_TO_HALLWAY = 1
    IN_INTERSECTION = 2
    AT_DEAD_END = 3

"""
Enum used for States 
"""
class Directions(Enum):
    LEFT = 0
    RIGHT = 1

"""
Enum used for Intersection Types 
"""
class Intersection(Enum):
    LFR = "Left Forward Right"
    LR = "Left Right"
    LF = "Left Forward"
    FR = "Forward Right"
    DE = "Dead End"


"""
Enum used for Service Names
"""
class Services(Enum):
    REALIGNMENT_SERVER = '/realignment_service_server'
    CRASH_DETECTION_SERVER = '/crash_detection_service_server'

"""
Enum used for Action Names
"""
class Actions(Enum):
    SCANNER_CHECK = '/scanner_check_action_server'

""" 
Compares two directions by their rotational degrees.
+ return means it needs to turn left
- to turn right.
0 is within acceptable range
"""
def get_turn_direction(current, target):
    result = target - current
    while (result > 180) or (result < -180):
        if result > 180:
            result -= 360
        elif result < -180:
            result += 360
    if -3 <= result <= 3:
        return 0
    else:
        return result

"""
Returns Euler angle for rotation about the z axis
"""
def get_degrees(msg):
    x = msg.pose.pose.orientation.x
    y = msg.pose.pose.orientation.y
    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    orientation_list = [x, y, z, w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    yaw = yaw * (180 / 3.1415)
    return yaw

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
Given the current orientation, add +-degrees 
and return the targetted orientation 
"""
def reorient(current, value):
    assert(abs(current) <= 180)
    assert(abs(value) <= 360)
    result = current + value
    if result > 180:
        result -= 360
    if result < -180:
        result += 360
    if result == -180:
        result = 180
    return result
    
"""
Given the robot's starting orientation,
generate an array of the 4 possible orientations
that the robot will use for navigating the maze.
"""
def get_direction_mapping(starting_orientation):
    north = starting_orientation
    south = reorient(north, 180)
    east = reorient(north, -90)
    west = reorient(north, 90)
    directed_paths = [north, south, east, west]
    return directed_paths

def is_inf(value):
    if value == "inf":
        return True
    else:
        return False


if __name__ == "__main__":
    corner = [2, 2]
    point_side = "right"
    directed_angle = 90
    side_length = 2
    generate_corners(corner, point_side, directed_angle, side_length)
    print(str(Intersection.LFR.name))