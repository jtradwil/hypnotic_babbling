#!/usr/bin/env python

# imports
import rospy
from geometry_msgs.msg import Twist, Vector3
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

class explorer(object):
    # Global variables for random bounds
    blind_threshold = 3.5
    run_rate = 50
    linear_acc  =  1.0 / run_rate
    angular_acc =  1.0 / run_rate
    angle_threshold = 60 * 3.14159 / 180
    
    # Frame definitions
    map_frame = "map"
    start_frame = "start_point"

    # Run setpoints
    wall_distance = 1
    max_speed = 0.75
    
    # Laser Information
    angle_min = 0
    dist_min = 0
    dist_front = 0

    # PID Coefficients
    angle_coef = 1.25
    k_p = 2
    k_i = 0
    k_d = 2

    # PID Error
    e_p = 0
    e_i = 0
    e_d = 0

    # Current set drive setpoints
    set_x = 0
    set_z = 0
    
    # Map Params
    map_path = ""
    
    stage = 0
    laser_valid = 0

    def __init__(self):
        rospy.Subscriber("/scan", LaserScan, self._laser_cb)
        self.rate = rospy.Rate(self.run_rate)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.tf_lsnr = tf.TransformListener()
        self.start_bcst = tf.TransformBroadcaster()
        rospack = rospkg.RosPack()
        self.map_path = str(rospack.get_path('map_sim')) + "/map/gmap"
        self._run_states()
        
        
    def _run_states(self):
        linear = 0
        angular = 0
        
        #Start zone variables
        start_zone = 1
        start_trans = np.zeros(3)
        start_rot = np.zeros(4)
    
        while not(rospy.is_shutdown()):
            if(self.stage == 0):
                (success,start_trans, start_rot) = self._get_transform("map", "base_link")
                if(self.laser_valid == 1):
                    self.laser_valid == 0
                    if(self.dist_min > (self.wall_distance + 0.25)):
                        linear = 0.5
                        angular = 0.125
                    else:
                        linear = 0
                        angular = 0
                        self.stage = self.stage + 1
                        
            elif(self.stage == 1):
                linear, angular = self._update_pid(self.angle_min, self.dist_min, self.dist_front)
                
                (success,trans, rot) = self._get_transform("start_point", "base_link")
                distance = np.linalg.norm(trans)
                
                if(start_zone == 1):
                    if(distance > 4):
                        start_zone = 0
                else:
                    if(distance < 1):
                        linear = 0
                        angular = 0
                        
                        self.stage = self.stage + 1
                
            elif(self.stage == 2):
                linear = 0
                angular = 0
                
                # Wait to stop moving
                if((self.set_x == 0) and (self.set_z == 0) ):
                    self._save_map()
                    self.stage = self.stage + 1
                
            elif(self.stage == 3):
                self._open_cv_map()
                self.stage = self.stage + 1
                
            elif(self.stage == 4):
                linear = 0
                angular = 0 
                cv2.waitKey(30)
                
        
            self._update_cmd(linear, angular)
            self._publish_start_tf(start_trans, start_rot)
            rospy.loginfo('%d: X: %2.4f  -   Z: %2.4f', self.stage,linear, angular)
            self.rate.sleep()
            
        
    # Callback for laser data
    def _laser_cb(self,data):
        global angle_min, dist_min, dist_front, first_wall
        
        size = len(data.ranges)
        minIndex = size/2
        maxIndex = size
        
        for i in range(minIndex,maxIndex):
            if (data.ranges[i] < data.ranges[minIndex]) and (data.ranges[i] > 0.0):
                minIndex = i
                
        self.angle_min = (minIndex - size/2) * data.angle_increment
        self.dist_min = data.ranges[minIndex]
        self.dist_front = data.ranges[size/2]
        
        self.laser_valid = 1
        
    # Update and pubish the drive commands
    def _update_cmd(self, cmd_x, cmd_z):
        if (cmd_x > self.set_x):
            if (cmd_x > (self.set_x + self.linear_acc)):
                self.set_x = self.set_x + self.linear_acc
            else :
                self.set_x = cmd_x
        else :
            if (cmd_x < (self.set_x - self.linear_acc)):
                self.set_x = self.set_x - self.linear_acc
            else :
                self.set_x = cmd_x
                    
        if (cmd_z > self.set_z):
            if (cmd_z > (self.set_z + self.angular_acc)):
                self.set_z = self.set_z + self.angular_acc
            else :
                self.set_z = cmd_z
        else :
            if (cmd_z < (self.set_z - self.angular_acc)):
                self.set_z = self.set_z - self.angular_acc
            else :
                self.set_z = cmd_z

        linear_msg = Vector3(x=self.set_x, y=float(0.0), z=float(0.0))
        angular_msg = Vector3(x=float(0.0), y=float(0.0), z=self.set_z)
        publish_msg = Twist(linear=linear_msg, angular=angular_msg)
        
        self.cmd_pub.publish(publish_msg)


    # Update the PID
    def _update_pid(self, angle_min, dist_min, dist_front):
        global e_p, e_i, e_d
        
        dt = 0
        e_prev = self.e_p
        
        # Update errors
        self.e_p = dist_min - self.wall_distance
        self.e_i = self.e_i + self.e_p * dt
        self.e_d = self.e_p - e_prev
          
        # run the PID
        cmd_angular = (self.k_p * self.e_p) + (self.k_i * self.e_i) + (self.k_d * self.e_d) + self.angle_coef * (angle_min - math.pi / 2)
        
        # Limit max turn speed
        if(cmd_angular > 1.5):
            cmd_angular = 1.5
        
        # Do some speed regulation based on how far open space we have
        speed = self.dist_front * self.max_speed / self.blind_threshold
        
        if(self.dist_min < (self.wall_distance-0.25)):
            cmd_linear = 0
            
        elif(self.dist_front < self.wall_distance):
            cmd_linear = 0
            
        elif(math.fabs(self.angle_min) < self.angle_threshold):
            cmd_linear = 0.25 * self.max_speed
        else:
            if(speed > self.max_speed):
                cmd_linear = self.max_speed
            else:
                cmd_linear = speed

        return (cmd_linear, cmd_angular)

    # Get translation and rotation from source frame to target_frame
    def _get_transform(self, source_frame, target_frame):         
        trans = np.zeros(3)
        rot = np.zeros(4)
        
        success = -1
         
        if self.tf_lsnr.frameExists(source_frame) and self.tf_lsnr.frameExists(target_frame):
            try:
                now = rospy.Time(0)
                self.tf_lsnr.waitForTransform(source_frame, target_frame, now, rospy.Duration(2.0))
                trans, rot  = self.tf_lsnr.lookupTransform(source_frame, target_frame, now)
                
                success = 1
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
                
        return (success, trans, rot)


    # Publish the start point transform
    def _publish_start_tf(self, trans, rot):
        self.start_bcst.sendTransform((trans[0], trans[1], trans[2]),
                                      (rot[0], rot[1], rot[2], rot[3]),
                                      rospy.Time.now(), self.start_frame, self.map_frame)

    # Launch the map saver to save the gmapping map
    def _save_map(self):
        #start save map
        package='map_server'
        executable ='map_saver'
        node = roslaunch.core.Node(package, executable, args="-f " + self.map_path)
        
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        process = launch.launch(node)
            
            
        while(process.is_alive()):
            pass
            
            
        rospy.loginfo("-f " + self.map_path) 
        
    # Load map params from yaml, publish to param server, return useful params
    def _get_map_params(self):
        paramlist=rosparam.load_file(self.map_path + ".yaml",default_namespace="map_params")
        
        for params, ns in paramlist:
            rosparam.upload_params(ns,params)
            
        origin = rospy.get_param('/map_params/origin')
        resolution = rospy.get_param('/map_params/resolution')
       
        return (origin, resolution)

    def _open_cv_map(self):
        (origin,resolution) = self._get_map_params()
        
        
    
        img = cv2.imread(self.map_path + ".pgm",0)
        
        height, width = img.shape[:2]
        
        color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        cv2.circle(color, (abs(int(origin[0] / resolution)), height - abs(int(origin[1] / resolution))), 5, (0, 0, 255), 2)
        
        cv2.imshow('Result', cv2.resize(color, (0,0), fx=0.5, fy=0.5))
        
        cv2.waitKey(30)

# standard ros boilerplate
if __name__ == "__main__":
    try:
        rospy.init_node("jackal_explore")
        time.sleep(10)
        explorer = explorer()
    except rospy.ROSInterruptException:
        pass
