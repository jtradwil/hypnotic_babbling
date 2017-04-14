#!/usr/bin/env python

# imports
import rospy
import random
import sys
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import time
import roslaunch
import os
import math

# Global variables for random bounds
wall_distance = 1
error = 0
diff_e = 0
max_speed = 0.5
pid_kp = 2
pid_kd = 2
angle_coef = 1
direction = 1
angle_min = 0
dist_front = 0

cmd_x = 0
cmd_z = 0


# define callback for twist
def Callback(data):
    global angle_min, dist_front, diff_e, error
    
    size = len(data.ranges)
    minIndex = size/2
    maxIndex = size
    
    for i in range(minIndex,maxIndex):
        if (data.ranges[i] < data.ranges[minIndex]) and (data.ranges[i] > 0.0):
            minIndex = i
            
    angle_min = (minIndex - size/2) * data.angle_increment
    dist_min = data.ranges[minIndex]
    dist_front = data.ranges[size/2]
    diff_e = (dist_min - wall_distance) - error
    error = dist_min - wall_distance
    
    Update()
    
def Update():
    global cmd_x, cmd_z
    
    
    cmd_z = direction * (pid_kp * error + pid_kd * diff_e) + angle_coef * (angle_min - math.pi * direction / 2)
    
    if(dist_front < wall_distance):
        cmd_x = 0
    elif(dist_front < (wall_distance * 2)):
        cmd_x = 0.5 * max_speed
    elif(math.fabs(angle_min) > 1.75):
        cmd_x = 0.3 * max_speed
    else:
        cmd_x = max_speed
        


# define setup and run routine
def run():
    rospy.Subscriber("/scan", LaserScan, Callback)
    rate = rospy.Rate(50)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    count = 0
    countLimit = 5
    
    while not rospy.is_shutdown():
        if count < countLimit :
            count = count + 1
        else :
            count = 0

            linear_msg = Vector3(x=cmd_x, y=float(0.0), z=float(0.0))
            angular_msg = Vector3(x=float(0.0), y=float(0.0), z=cmd_z)
            publish_msg = Twist(linear=linear_msg, angular=angular_msg)
            
            pub.publish(publish_msg)
            
            rospy.loginfo('\tX: %2.4f  -  Z: %2.4f', cmd_x, cmd_z)
            
            


# standard ros boilerplate
if __name__ == "__main__":
    try:
        rospy.init_node("jackal_explore")
        run()
    except rospy.ROSInterruptException:
        pass
