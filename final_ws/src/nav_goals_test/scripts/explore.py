#!/usr/bin/env python

# imports
import rospy
from geometry_msgs.msg import Twist, Vector3, TransformStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import tf
import time
from tf import transformations as t
import roslaunch
import os
import sys
import rospkg
import rosparam


# Global variables for random bounds
blind_threshold = 3.5
run_rate = 10
linear_acc  =  0.005 / run_rate
angular_acc =  0.00875 / run_rate
angle_threshold = 60 * 3.14159 / 180

wall_distance = 1
max_speed = 0.75
angle_coef = 1.25
direction = 1
angle_min = 0
dist_min = 0
dist_front = 0

# PID Coefficients
k_p = 2
k_i = 0
k_d = 2

# PID Error
e_p = 0
e_i = 0
e_d = 0

# Output Commands
cmd_x = 0
cmd_z = 0


first_wall = 0

# 0 - Looking for first Wall
# 1 - Found first wall / following wall
# 2 - Back at start
stage = 0



# define callback for twist
def Callback(data):
    global angle_min, dist_min, dist_front, first_wall
    
    size = len(data.ranges)
    minIndex = size/2
    maxIndex = size
    
    for i in range(minIndex,maxIndex):
        if (data.ranges[i] < data.ranges[minIndex]) and (data.ranges[i] > 0.0):
            minIndex = i
            
    angle_min = (minIndex - size/2) * data.angle_increment
    dist_min = data.ranges[minIndex]
    dist_front = data.ranges[size/2]
    
    update_path()
    

def update_path():
    global e_p, e_i, e_d, stage, cmd_x, cmd_z
    
    if(stage == 0):
        if(dist_min > (wall_distance + 0.25)):
            cmd_x = 0.25
            cmd_z = 0.125
        else:
            cmd_x = 0
            cmd_z = 0
            stage = 1
        
    elif(stage == 1):
        dt = 0
        e_prev = e_p
        
        # Update errors
        e_p = dist_min - wall_distance
        e_i = e_i + e_p * dt
        e_d = e_p - e_prev
          
        # run the PID
        cmd_z = (k_p * e_p) + (k_i * e_i) + (k_d * e_d) + angle_coef * (angle_min - math.pi * direction / 2)
        
        if(cmd_z > 1.5):
            cmd_z = 1.5
        
        # Do some speed regulation based on how far open space we have
        speed = dist_front * max_speed / blind_threshold
        
        if(dist_min < (wall_distance-0.25)):
            cmd_x = 0
            
        elif(dist_front < wall_distance):
            cmd_x = 0
            
        elif(math.fabs(angle_min) < angle_threshold):
            cmd_x = 0.25 * max_speed
        else:
            if(speed > max_speed):
                cmd_x = max_speed
            else:
                cmd_x = speed
                
    else:
        cmd_x = 0
        cmd_z = 0
            
            
            
            
            
            
# define setup and run routine
def run():
    global cmd_x, cmd_z, stage
    
    time.sleep(10)

    rospy.Subscriber("/scan", LaserScan, Callback)
    rate = rospy.Rate(run_rate)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    m = TransformStamped()
    
    set_x = 0
    set_z = 0
    
    startZone = 1
    start_x = 0
    start_y = 0
    valid = 0
    
    my_x = 0
    my_y = 0
    
    start_trans = np.zeros(3)
    start_rot = np.zeros(4)
    dist_home = 0
    
    
    rospack = rospkg.RosPack()
    saving = 0
    run = 1
    map_path = str(rospack.get_path('map_sim')) + "/map/gmap"
    
    while not(rospy.is_shutdown()) and run:    
        if(stage == 0):
            if listener.frameExists("/base_link") and listener.frameExists("/map"):
                try:
                    now = rospy.Time(0)
                    listener.waitForTransform("/map", "/base_link", now, rospy.Duration(2.0))
                    start_trans, start_rot  = listener.lookupTransform("/map", "/base_link", now)
                    
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                    
        elif(stage == 1):
            if listener.frameExists("/base_link") and listener.frameExists("/start_point"):
                try:
                    now = rospy.Time(0)
                    listener.waitForTransform("/base_link", "/start_point", now, rospy.Duration(2.0))
                    trans, rot  = listener.lookupTransform("/base_link", "/start_point", now)
                    
                    dist_home = np.linalg.norm(trans)
                    
                    if((dist_home > 4) and (startZone == 1)):
                        startZone = 0
                    elif((dist_home < 1) and (startZone == 0)):
                        stage = 2
                    
                    
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                
        elif(stage == 2):
            
            if(saving == 0):
                saving = 1
                
                #start save map
                package='map_server'
                executable ='map_saver'
                node = roslaunch.core.Node(package, executable, args="-f " + map_path)
                
	
                launch = roslaunch.scriptapi.ROSLaunch()
                launch.start()

                process = launch.launch(node)
                
                
            if(not(process.is_alive())):
                stage = 3
                rospy.loginfo("-f " + map_path)
        elif(stage == 3):
            run = 0
                
        rospy.loginfo('\tX: %2.4f  -  Y: %2.4f  -  D: %3.2f', start_trans[0], start_trans[1], dist_home)
        
        br.sendTransform((start_trans[0], start_trans[1], start_trans[2]),
                         (start_rot[0], start_rot[1], start_rot[2], start_rot[3]),
                         rospy.Time.now(),
                         "start_point",
                         "map")


        


        # Basic acceleration code
        if (cmd_x > set_x):
            if (cmd_x > (set_x + linear_acc)):
                set_x = set_x + linear_acc
            else :
                set_x = cmd_x
        else :
            if (cmd_x < (set_x - linear_acc)):
                set_x = set_x - linear_acc
            else :
                set_x = cmd_x
                    
        if (cmd_z > set_z):
            if (cmd_z > (set_z + angular_acc)):
                set_z = set_z + angular_acc
            else :
                set_z = cmd_z
        else :
            if (cmd_z < (set_z - angular_acc)):
                set_z = set_z - angular_acc
            else :
                set_z = cmd_z

        linear_msg = Vector3(x=set_x, y=float(0.0), z=float(0.0))
        angular_msg = Vector3(x=float(0.0), y=float(0.0), z=set_z)
        publish_msg = Twist(linear=linear_msg, angular=angular_msg)
        
        #pub.publish(publish_msg)            
            
            
    # load parameters for map
    paramlist=rosparam.load_file(map_path + ".yaml",default_namespace="map_params")
    
    for params, ns in paramlist:
        rosparam.upload_params(ns,params)
        
    print rospy.get_param('/map_params/origin')[0]
    
    while not(rospy.is_shutdown()):
        rate.sleep()


# standard ros boilerplate
if __name__ == "__main__":
    try:
        rospy.init_node("jackal_explore")
        run()
    except rospy.ROSInterruptException:
        pass
