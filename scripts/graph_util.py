#! /usr/bin/env python
import rospy
from utility import reorient


"""
Shift the nodes: current -> previous
destination -> current and empty out destination
"""
def update_nodes(robot):
    robot.previous_node = robot.current_node
    robot.current_node = robot.destination_node
    robot.destination_node = None

"""
Add the connection from the previous to the current
and vice versa. Change previous orientation
"""
def update_connections(robot):
    from_direction = reorient(robot.orientation_directed, 180)
    for i in range(len(robot.directed_paths)):
        if robot.directed_paths[i] == from_direction:
            robot.current_node.directions[i] = robot.previous_node
        if robot.directed_paths[i] == robot.previous_orientation:
            robot.previous_node.directions[i] = robot.current_node

"""
Pop from the robot's path and set it to destination
"""
def set_destination_to_path(robot):
    robot.destination_node = robot.path.pop()
    for i in range(len(robot.current_node.directions)):
        if robot.current_node.directions[i] == None:
            pass
        elif robot.current_node.directions[i] == robot.destination_node:
            robot.orientation_directed = robot.directed_paths[i]
    robot.previous_orientation = robot.orientation_directed
    rospy.logwarn("Destination from Path: " + str(robot.orientation_directed))

"""
If there is an unvisited child at current node
set path and destination to it.
"""
def set_destination_to_unvisited_child(robot):
    for i in range(len(robot.directed_paths)):
        if robot.current_node.directions[i] == None:
            continue
        elif not robot.current_node.directions[i].is_visited:
            rospy.logwarn("Set Path: " + str(robot.directed_paths[i]))
            robot.orientation_directed = robot.directed_paths[i]
            robot.destination_node = robot.current_node.directions[i]
            break
    robot.previous_orientation = robot.orientation_directed