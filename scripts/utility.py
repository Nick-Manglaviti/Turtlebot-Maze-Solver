#! /usr/bin/env python

from tf.transformations import euler_from_quaternion
from enum import Enum
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
    

def is_inf(value):
    if value == "inf":
        return True
    else:
        return False