#! /usr/bin/env python
import rospy

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
    update_connections(robot)
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
    update_connections(robot)
    for i in range(len(robot.directed_paths)):
        if robot.current_node.directions[i] == None:
            continue
        elif not robot.current_node.directions[i].is_visited:
            rospy.logwarn("Set Path: " + str(robot.directed_paths[i]))
            robot.orientation_directed = robot.directed_paths[i]
            robot.destination_node = robot.current_node.directions[i]
            break
    robot.previous_orientation = robot.orientation_directed

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

"""
Search the graph for the node based on the x y values.
Return the found node or a placeholder node.
"""
def lookup_node(coordinates, graph):
    rospy.logwarn("Performing Lookup with x: " + str(coordinates[0]) + " y: " + str(coordinates[1]))
    searched_node = graph.search_by_position(coordinates[0], coordinates[1])
    return searched_node

"""
Generate a path using the graph's BFS and if it exists, 
set the destination to the robot's newpath.
"""
def create_and_set_path(robot):
    robot.path = robot.graph.search_for_unvisited(robot.current_node)
    if (len(robot.path) == 0):
        rospy.logerr("No Valid Nodes could be found.")
        robot.cancel_scanner_check_action()
    else:
        set_destination_to_path(robot)