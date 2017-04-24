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
from math import radians, degrees
from actionlib_msgs.msg import *
from nav_msgs.msg import Path
import random

import move_to

class explore_random(object):
    pos_x=[]  
    pos_y=[]

    pos_r = [0, math.pi/2, math.pi, 3*math.pi/2]

    def __init__(self, queue):
        self.queue = queue
        
        del self.pos_x[:] 
        del self.pos_y[:] 
        
        self._find_points()

    def _run(self):
        rand_point_mover = move_to.move_to()

        run = 1

        pos_index = 0
        rot_index = 0

        status = 0

        rand_point_mover._move_to_goal(self.pos_x[pos_index], self.pos_y[pos_index], self.pos_r[rot_index])
        
        while(not(rospy.is_shutdown()) and (run == 1)):
            if(not(self.queue.empty())):
                run = 0

            if(status == 1):
                status = 0
                
                if(rot_index < 3):
                    rot_index = rot_index + 1
                    rand_point_mover._move_to_goal(self.pos_x[pos_index], self.pos_y[pos_index], self.pos_r[rot_index])
                    
                else:
                    rot_index = 0
                    
                    if(pos_index < 3):
                        pos_index = pos_index + 1
                        rand_point_mover._move_to_goal(self.pos_x[pos_index], self.pos_y[pos_index], self.pos_r[rot_index])
                        status = 0
                    else:
                        run = 0
                        
                rospy.loginfo('Go To Position: %d, Rotation: %d', pos_index, rot_index)

            elif(status == -1):
                rospy.loginfo('Retry Position: %d, Rotation: %d', pos_index, rot_index)
                rand_point_mover._move_to_goal(self.pos_x[pos_index], self.pos_y[pos_index], self.pos_r[rot_index])
                status = 0
            else:
                status = rand_point_mover._check_goal_status(30)

        rospy.loginfo('Random Mover Finished')
        
        del rand_point_mover


    def _find_points(self):
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
        count_limit = (width*height/10)

        #find a point in the  first quadrant
        temp_x=random.randrange(0, width/2,1)
        temp_y=random.randrange(0, height/2,1)
        while((mask[temp_y, temp_x] != 255) and (count<count_limit)):
            temp_x=random.randrange(0, width/2,1)
            temp_y=random.randrange(0, height/2,1)
            count=count+1
        if (count<count_limit):
            self.pos_x.append(temp_x*resolution+origin[0])
            self.pos_y.append((height-temp_y)*resolution+origin[1])
        count = 0    

        #find a point in the second quadrant
        temp_x=random.randrange(width/2, width-1,1)
        temp_y=random.randrange(0, height/2,1)
        
        
        while((mask[temp_y, temp_x] != 255) and (count<count_limit)):
            temp_x=random.randrange(width/2, width-1,1)
            temp_y=random.randrange(0, height/2,1)
            count=count+1
        if (count<count_limit):
            self.pos_x.append(temp_x*resolution+origin[0])
            self.pos_y.append((height-temp_y)*resolution+origin[1])
        count = 0 

        #find a point in the third quadrant
        temp_x=random.randrange(width/2, width-1,1)
        temp_y=random.randrange(height/2, height-1,1)
        while((mask[temp_y, temp_x] != 255) and (count<count_limit)):
            temp_x=random.randrange(width/2, width-1,1)
            temp_y=random.randrange(height/2, height-1,1)
            count=count+1
        if (count<count_limit):
            self.pos_x.append(temp_x*resolution+origin[0])
            self.pos_y.append((height-temp_y)*resolution+origin[1])
        count = 0 

        #find a point in the fourth quadrant
        temp_x=random.randrange(0, width/2,1)
        temp_y=random.randrange(height/2, height-1,1)
        while((mask[temp_y, temp_x] != 255) and (count<count_limit)):
            temp_x=random.randrange(0, width/2,1)
            temp_y=random.randrange(height/2, height-1,1)
            count=count+1
        if (count<count_limit):
            self.pos_x.append(temp_x*resolution+origin[0])
            self.pos_y.append((height-temp_y)*resolution+origin[1])
        count = 0 	


def init():
	rospy.init_node('random_explorer', anonymous=False)

if __name__ == '__main__':
	try:
		init()
		explore()

	except rospy.ROSInterruptException:
		pass
