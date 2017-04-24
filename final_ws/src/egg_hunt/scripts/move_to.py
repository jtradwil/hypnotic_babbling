#!/usr/bin/env python

# imports
import rospy
from geometry_msgs.msg import Twist, Vector3, Point, PoseWithCovarianceStamped, Quaternion
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import tf
import time
import roslaunch
import os
import sys
import rospkg
import rosparam
import cv2
import copy
import Queue
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from nav_msgs.msg import Path
  
class move_to(object):

    def __init__(self):
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def _move_to_goal(self, xGoal, yGoal, yawGoal):
    
        while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            pass

        goal = MoveBaseGoal()

        # set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal
        goal.target_pose.pose.position = Point(xGoal , yGoal, 0)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yawGoal)
        goal.target_pose.pose.orientation.x=quaternion[0]
        goal.target_pose.pose.orientation.y=quaternion[1]
        goal.target_pose.pose.orientation.z=quaternion[2]
        goal.target_pose.pose.orientation.w=quaternion[3]

        self.goal_start_time = time.time()

        self.ac.send_goal(goal)

    def _move_to_pose(self, x, y, rx, ry, rz, rw):
    
        while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            pass

        goal = MoveBaseGoal()

        # set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal
        goal.target_pose.pose.position = Point(x, y, 0)
        goal.target_pose.pose.orientation.x=rx
        goal.target_pose.pose.orientation.y=ry
        goal.target_pose.pose.orientation.z=rz
        goal.target_pose.pose.orientation.w=rw

        self.goal_start_time = time.time()

        self.ac.send_goal(goal)
		    
    def _check_goal_status(self, time_limit):
        ret_val = 0

        time_elapsed = time.time() - self.goal_start_time

        if(self.ac.get_state() ==  GoalStatus.SUCCEEDED):
            self.ac.cancel_goal()
            ret_val = 1

        else:
            if(time_elapsed > time_limit):
                self.ac.cancel_goal()
                ret_val = -1
            else:
                ret_val = 0
                
        return ret_val
		        
		        
		        
		        
		        
		        
