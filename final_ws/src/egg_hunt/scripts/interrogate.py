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


circle_x = [-4.3806, 6.515, 3.8655, -4.1195, -7.9609, 5.9347]
circle_y = [-1.2384, 6.6707, -2.8412, 5.2126,-4.1242, -8.4489]
circle_r = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
  
def interrogate():
	for i in range(0, len(circle_x)):
		rospy.loginfo("Going below possible rabbit "+str(i))
		moveToGoal(circle_x[i], circle_y[i]-circle_r[i]-0.5, math.pi/2, 30)
		time.sleep(5)
		rospy.loginfo("Going left of possible rabbit "+str(i))
		moveToGoal(circle_x[i]-circle_r[i]-0.5, circle_y[i], 0, 30)
		time.sleep(5)
		rospy.loginfo("Going above possible rabbit "+str(i))
		moveToGoal(circle_x[i], circle_y[i]+circle_r[i]+0.5, 3*math.pi/2, 30)
		time.sleep(5)
		rospy.loginfo("Going right of possible rabbit "+str(i))
		moveToGoal(circle_x[i]+circle_r[i]+0.5, circle_y[i], math.pi, 30)
		time.sleep(5)		
		

def moveToGoal(xGoal,yGoal,yawGoal,tLimit):
	#define a client for to send goal requests to the move_base server through a SimpleActionClient
	ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

	#wait for the action server to come up
	while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
		rospy.loginfo("Waiting for the move_base action server to come up")
		time.sleep(1) 

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

   	rospy.loginfo("Sending goal location ...")
   	ac.send_goal(goal)
    
   	ac.wait_for_result(rospy.Duration(tLimit))

	if(ac.get_state() ==  GoalStatus.SUCCEEDED):
		rospy.loginfo("You have reached the destination")
		return True

	else:
		ac.cancel_goal() # Remove old goal that wasnt reached
		rospy.loginfo("The robot failed to reach the destination")
		return False
        

# FUNCTION: init()
# Initializes this node and sets up subsciber for Path messages.
# Then spawns a new Jackal using the random spawn function
def init():
    rospy.init_node('interrogator', anonymous=False)

if __name__ == '__main__':
	try:
		init()
		interrogate()

	except rospy.ROSInterruptException:
		pass
