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
import random
  
def explore():
    pos_x=[]  
    pos_y=[]
    #import image
    img = cv2.imread(str(rospkg.RosPack().get_path('egg_hunt')) + "/map/gmap" + ".pgm",0)
    color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    height, width, channels = color.shape
    mask = cv2.inRange(img, 250, 255)
    mask = cv2.erode(mask, None, iterations=30)

    #get the origin and resolution
    paramlist=rosparam.load_file(str(rospkg.RosPack().get_path('egg_hunt')) + "/map/gmap" + ".yaml",default_namespace="map_params")   
    for params, ns in paramlist:
        rosparam.upload_params(ns,params)
    origin = rospy.get_param('/map_params/origin')
    resolution = rospy.get_param('/map_params/resolution')

    count=0
    
    #find a point in the  first quadrant
    temp_x=random.randrange(0, width/2,1)
    temp_y=random.randrange(0, height/2,1)
    while(mask[temp_y, temp_x] != 255) and (count<width*height/10):
        temp_x=random.randrange(0, width/2,1)
        temp_y=random.randrange(0, height/2,1)
        count=count+1
    if (count<width*height/10):
        pos_x.append(temp_x*resolution+origin[0])
        pos_y.append(temp_y*resolution+origin[1])
    count = 0    
    
    #find a point in the second quadrant
    temp_x=random.randrange(width/2, width-1,1)
    temp_y=random.randrange(0, height/2,1)
    while(mask[temp_y, temp_x] != 255) and (count<width*height/10):
        temp_x=random.randrange(width/2, width-1,1)
        temp_y=random.randrange(0, height/2,1)
        count=count+1
    if (count<width*height/10):
        pos_x.append(temp_x*resolution+origin[0])
        pos_y.append(temp_y*resolution+origin[1])
    count = 0 

    #find a point in the third quadrant
    temp_x=random.randrange(width/2, width-1,1)
    temp_y=random.randrange(height/2, height-1,1)
    while(mask[temp_y, temp_x] != 255) and (count<width*height/10):
        temp_x=random.randrange(width/2, width-1,1)
        temp_y=random.randrange(height/2, height-1,1)
        count=count+1
    if (count<width*height/10):
        pos_x.append(temp_x*resolution+origin[0])
        pos_y.append(temp_y*resolution+origin[1])
    count = 0 

    #find a point in the fourth quadrant
    temp_x=random.randrange(0, width/2,1)
    temp_y=random.randrange(height/2, height-1,1)
    while(mask[temp_y, temp_x] != 255) and (count<width*height/10):
        temp_x=random.randrange(0, width/2,1)
        temp_y=random.randrange(height/2, height-1,1)
        count=count+1
    if (count<width*height/10):
        pos_x.append(temp_x*resolution+origin[0])
        pos_y.append(temp_y*resolution+origin[1])
    count = 0 

    #go to each point and pose in all four orientations. 
    for i in range(0, len(pos_x)):
        rospy.loginfo("Going to random point in quadrant "+str(i+1))
        moveToGoal(pos_x[i], pos_y[i], math.pi/2, 30)
        time.sleep(5)
        moveToGoal(pos_x[i], pos_y[i], 0, 30)
        time.sleep(5)
        moveToGoal(pos_x[i], pos_y[i], 3*math.pi/2, 30)
        time.sleep(5)
        moveToGoal(pos_x[i], pos_y[i], math.pi, 30)
        time.sleep(5)		
		

#same movebase function I keep using.
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
        

def init():
    rospy.init_node('random_explorer', anonymous=False)

if __name__ == '__main__':
    try:
        init()
        explore()

    except rospy.ROSInterruptException:
        pass
