#!/usr/bin/env python

# imports
import rospy
import random
import sys
from geometry_msgs.msg import Twist, Vector3, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np
import time
import roslaunch
import os
import math

# Global variables for random bounds


cmd_x = 0
cmd_z = 0

class wall_follower(object):
    num_zones = 5
    zones_window_start = -80.0
    zones_window_end = 20.0
    zone_delta = 1.0
    zone_boundries = np.zeros(num_zones + 1)
    zone_coef = np.zeros(num_zones-1)
    zone_angle = (zones_window_end - zones_window_start) / num_zones
    markerArray = MarkerArray()

    def __init__(self):
        # Initialize the ros stuff
        rospy.init_node("jackal_explore")
        rospy.Subscriber("/scan", LaserScan, self._laser_cb)
        rate = rospy.Rate(50)
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.marker_pub = rospy.Publisher("/explore_markers", MarkerArray)
        
        # Calculate the centers for the zones being used
        for i in range(0,self.num_zones+1):
            boundry = (self.zone_angle * i) + self.zones_window_start
            self.zone_boundries[i] = boundry
            rospy.loginfo('\tzone[%d] boundries =  %f', i, self.zone_boundries[i])
            
        whole_coef = 1.0
        # Calculate the centers for the zones being used
        #for i in range(0,self.num_zones-2):
        #    whole_coef = whole_coef / 2.0
        #    self.zone_coef[i] = whole_coef  
        #self.zone_coef[self.num_zones-2] = whole_coef
        
        for i in range(0,self.num_zones-2):
            self.zone_coef[i] = whole_coef / self.num_zones
        
            
        rospy.loginfo('\tCoefficient Sum = %f', sum(self.zone_coef))
            
        # Send the updated data at 10 Hz
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
                
                #pub.publish(publish_msg)
                
                
                #rospy.loginfo('\tX: %2.4f  -  Z: %2.4f', cmd_x, cmd_z)
            
        
    def _laser_cb(self, data):
        side = 0
        zone_dist = np.zeros(self.num_zones+1)
        zone_ang = np.zeros(self.num_zones+1)
        
        start_ang = self._to_ang(data.angle_min)
        end_ang = self._to_ang(data.angle_max)
        increment = self._to_ang(data.angle_increment)
        
        #Catch anything out of range
        if((start_ang > self.zones_window_start) or (end_ang < self.zones_window_end)):
            rospy.loginfo('Window bad %2.5f = %2.5f  |  %2.5f = %2.5f', start_ang, self.zones_window_start, end_ang, self.zones_window_end)
        else:
            
            path_points = []
            
            # Calculate the centers for the zones being used
            for i in range(0,self.num_zones+1):
                start_index = int((self.zone_boundries[i] - start_ang - (self.zone_delta / 2)) / increment)
                end_index = int((self.zone_boundries[i] - start_ang + (self.zone_delta / 2)) / increment)
                
                zone_ang[i] = self.zone_boundries[i]
                zone_dist[i] = self._get_ave(start_index, end_index, data)
                
                x = zone_dist[i] * math.cos(self._to_rad(zone_ang[i]))
                y = zone_dist[i] * math.sin(self._to_rad(zone_ang[i]))
                
                vertex_marker = self._make_vertex(x , y, i)
                self.markerArray.markers.append(vertex_marker)
                
                p = Point()
                p.x = x
                p.y = y
                p.z = 0
                path_points.append(p)
            
            # Mark the path
            path_marker = self._make_path(path_points, i)
            self.markerArray.markers.append(path_marker)
                
            self.marker_pub.publish(self.markerArray)
            
            del self.markerArray.markers[:]
            
            error = np.zeros(self.num_zones)
            
            # Calculate error
            for i in range(0,self.num_zones):
            
                error[i] = self._do_shit(zone_dist[i], zone_dist[i+1], zone_ang[i], zone_ang[i+1])
                
                #rospy.loginfo('[%d]  -  M1: %2.3f,%2.3f  -  M2: %2.3f,%2.3f  -  Tr: %2.3f  -  Tw: %2.3f', i, dist_1,zone_ang[i], dist_2, zone_ang[i+1], theta_r, theta_w)
            
            
            total_error = 0
            
            for i in range(0,self.num_zones-1):
                total_error = total_error + error[i] * self.zone_coef[i]
                #rospy.loginfo('[%d]  -  Theta Error %3.3f', i, error[i])
            
            #rospy.loginfo('\tTheta Error %3.3f', total_error)
            rospy.loginfo('\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f', error[0], error[1], error[2], error[3], error[4])
                

    def _do_shit(self, d1, d2, t1, t2):
        
        direction = 1
        compensation = 0
        
        theta_z = abs(t2 - t1)
        
        if(d1 > d2): # Right Front or Left Wall
            dist_long = d1
            dist_short = d2
            
            if(t2 < 0.0): # Right Front
                theta_r = 90.0 - t1
                direction = -1
                compensation = -90
            else: # Left Wall
                theta_r = t1
                compensation = -180
                
            
                
            com = dist_short * math.sin(self._to_rad(theta_z))
            dist_long_p1 = dist_short * math.cos(self._to_rad(theta_z))
            dist_long_p2 = dist_long - dist_long_p1   
            theta_w = (self._to_ang(math.atan(com / dist_long_p2)) + compensation) * direction
            wall_length = math.sqrt(com*com + dist_long_p2*dist_long_p2)
            
            error = abs(theta_w) - abs(theta_r)
            
        elif(d1 < d2): # Left Front or Right Wall
            dist_long = d2
            dist_short = d1
            
            if(t2 < 0.0): # Right Wall
                theta_r = t2
                direction = -1
                compensation = 0
            else: # Left Front
                theta_r = 90.0 - t2
                compensation = -90
                
            com = dist_short * math.sin(self._to_rad(theta_z))
            dist_long_p1 = dist_short * math.cos(self._to_rad(theta_z))
            dist_long_p2 = dist_long - dist_long_p1   
            theta_w = (self._to_ang(math.atan(com / dist_long_p2)) + compensation) * direction
            wall_length = math.sqrt(com*com + dist_long_p2*dist_long_p2)
            
            error = abs(theta_w) - abs(theta_r)
        else:
            theta_tmp1 = abs(90.0 + t1)
            theta_tmp2 = (180.0 - theta_z) / 2
            error = 90 + theta_tmp1 - theta_tmp2
            
        return error
        

    def _make_vertex(self, x , y, i):
        vertex_marker = Marker()
        vertex_marker.header.frame_id = "/lms291"
        vertex_marker.header.stamp = rospy.get_rostime()
        vertex_marker.ns = "explorer_vis";
        vertex_marker.id = i;
        vertex_marker.type = vertex_marker.SPHERE
        vertex_marker.action = vertex_marker.ADD
        vertex_marker.scale.x = 0.2
        vertex_marker.scale.y = 0.2
        vertex_marker.scale.z = 0.2
        vertex_marker.color.a = 1.0
        vertex_marker.color.r = 1.0
        vertex_marker.pose.orientation.w = 1.0
        vertex_marker.pose.position.x = x
        vertex_marker.pose.position.y = y
        vertex_marker.pose.position.z = 0
        
        return vertex_marker
        
    def _make_path(self, points, i):
        path_marker = Marker()
        path_marker.header.frame_id = "/lms291"
        path_marker.header.stamp = rospy.get_rostime()
        path_marker.ns = "explorer_vis";
        path_marker.id = self.num_zones + i;
        path_marker.type = path_marker.LINE_STRIP
        path_marker.action = path_marker.ADD
        path_marker.scale.x = 0.1
        path_marker.scale.y = 0.1
        path_marker.scale.z = 0.1
        path_marker.color.a = 1.0
        path_marker.color.b = 1.0            
        path_marker.points = points
        
        return path_marker

    def _get_ave(self, start, end, data):
        angSum = float(0.0)
        index = start
        while index < end :
            angSum = angSum + data.ranges[index]
            index = index + 1
            
        angSum = float(angSum) / float(end-start)
        
        return angSum
        
    def _to_ang(self, rad):
        ang = rad * 180.0 / 3.14159
        return ang
        
    def _to_rad(self, ang):
        rad = ang * 3.14159 / 180.0
        return rad

# standard ros boilerplate
if __name__ == "__main__":
    try:
        
        explorer = wall_follower()
    except rospy.ROSInterruptException:
        pass
