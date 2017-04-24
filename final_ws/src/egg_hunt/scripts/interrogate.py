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
  
class interrogator(object):
    # FUNCTION: init()
    # Initializes this node and sets up subsciber for Path messages.
    # Then spawns a new Jackal using the random spawn function
    def __init__(self, queue, points):
        self.queue = queue
        self.points = points
        self.point_index = 0
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
    def _run(self):
        run = 1
        
        interrogating = 0
        interrogate_stage = 0
        
        # dummy target values
        tar_x = 0
        tar_y = 0
        tar_y = 0
        tar_limit = 10
        
        while not(rospy.is_shutdown()) and (run == 1):
        
            if(interrogate_stage == 0):
                if(self.point_index <  len(self.points)):
                    interrogate_stage = 1
                    
                    tar_x = self.points[self.point_index][0]
                    tar_y = self.points[self.point_index][1]
                    tar_r = self.points[self.point_index][2]
                    
                    self.point_index = self.point_index + 1
                
                else:
                    run = 0
                    self.queue.put("DONE")
                    
            else:
                if(interrogating == 0):
                    if(interrogate_stage == 1):
                        moveToGoal(tar_x, tar_y - tar_r - 0.5, math.pi/2, 30)
                        interrogating = 1
                        
                    elif(interrogate_stage == 2):
                        moveToGoal(tar_x - tar_r - 0.5, tar_y, 0, 30)
                        interrogating = 1
                        
                    elif(interrogate_stage == 3):
                        moveToGoal(tar_x, tar_y + tar_r + 0.5, 3 * math.pi/2, 30)
                        interrogating = 1
                        
                    elif(interrogate_stage == 4):
                        moveToGoal(tar_x + tar_r + 0.5, tar_y, math.pi, 30)
                        interrogating = 1
                        
                    else:
                        interrogate_stage = 0
                                   
                else:
                    status = self._check_goal_status(tar_limit)
                    
                    if(status <> 0)
                        interrogating = 0
                        interrogate_stage = interrogate_stage + 1
                    
            if(not(self.queue.empty())):
                run = 0		

    def _move_to_goal(self, xGoal, yGoal, yawGoal):
	    while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
		    pass

	    goal = MoveBaseGoal()

	    # set up the frame parameters
	    goal.target_pose.header.frame_id = "map"
	    goal.target_pose.header.stamp = rospy.Time.now()

       	# moving towards the goal
       	goal.target_pose.pose.position = Point(xGoal,yGoal,0)
	    quaternion = tf.transformations.quaternion_from_euler(0, 0, yawGoal)
	    goal.target_pose.pose.orientation.x=quaternion[0]
     	goal.target_pose.pose.orientation.y=quaternion[1]
       	goal.target_pose.pose.orientation.z=quaternion[2]
       	goal.target_pose.pose.orientation.w=quaternion[3]
       	
       	self.goal_start_time = time.time()
       	
       	self.ac.send_goal(goal)
		    
    def _check_goal_status(self, time_limit):
        ret_val = 0
    
        time_elapsed = time.time() - self.goal_start_time
        
	    if(self.ac.get_state() ==  GoalStatus.SUCCEEDED):
		    ret_val = 1

	    else:
	        if(time_elapsed > time_limit):
		        self.ac.cancel_goal()
		        ret_val = -1
		    else:
		        ret_val = 0
        


if __name__ == '__main__':
	try:
		init()
		interrogate()

	except rospy.ROSInterruptException:
		pass
