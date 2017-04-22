#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Twist, Vector3, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Path
import roslaunch
from random import randint
import time
import math

def path_callback(myPath):
    global target
def move():
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
    
    # moving towards target position
    if(target = 1):
    	goal = rabbit_one_pose
    if(target =2):
    	goal = rabbit_two_pose
    if(target = 3):
    	goal = rabbit_three_pose
	
    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)
    
    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)
    
    ac.wait_for_result(rospy.Duration(5))
    
    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")
        return True
    else:
        ac.cancel_goal() # Remove old goal that wasnt reached
        rospy.loginfo("The robot failed to reach the destination yet")
        return False

def move_to_zero():
    global success
    while(1):
	if move()==True:
	    rospy.loginfo("break from posting. Goal reached")
	    break
	else:
	    rospy.loginfo("continue one more goal")
	    continue
    rospy.loginfo("reached")    
	

def init():
    rospy.init_node('NtoT', anonymous=False)
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, path_callback) 

# standard ros boilerplate
if __name__ == "__main__":
    try:
        time.sleep(10)
	init()
	move_to_zero()
    except rospy.ROSInterruptException:
        pass

