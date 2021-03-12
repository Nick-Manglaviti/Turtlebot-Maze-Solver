#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan


class LaserScanAnalysis(object):
    def __init__(self):
        pass

'''
Constantly checks for all values to see
if there are all "inf" then robot is out of maze
'''
def check_if_out_maze(laser_scan_result_array):
    laser_scan_analysis_object = LaserScanAnalysis()
    flag = True
    for i in range(len(laser_scan_result_array)):
        rospy.loginfo("Index: "+ str(i) + ", Value: " + str(laser_scan_result_array[i]))
        if str(laser_scan_result_array[i]) != "inf":
            flag = False
            break
    rospy.loginfo("Out of Maze: "+str(flag))
    return flag