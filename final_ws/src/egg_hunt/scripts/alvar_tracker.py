#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
import time
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
import tf
import math

class alvar_tracker(object):
    run_rate = 20
    seen_markers = []
    marker_delta = 0.75
    
    map_frame = "map"

    def __init__(self, queue, previous_markers, num_targets):
        self.queue = queue
        self.num_targets = num_targets
        self.tf_lsnr = tf.TransformListener()
        self.start_bcst = tf.TransformBroadcaster()
        self.rate = rospy.Rate(self.run_rate)
        
        del self.seen_markers[:] 
        
        rospy.loginfo('Populatig %d previous markers', len(previous_markers))
        
        for prev in previous_markers:
            self.seen_markers.append(prev)
        
    
    def _marker_cb(self, data):
        
        for marker in data.markers:
            if((marker.id > 0) and (marker.id < 4)):
                ar_frame = "ar_marker_" + str(marker.id)
                
                (success, trans, rot) = self._get_transform(self.map_frame, ar_frame)
                (r_success, r_trans, r_rot) = self._get_transform("base_link", ar_frame)
                
                if((success == 1) and (r_success == 1)):
                    distance = np.linalg.norm(r_trans)
                    
                    new_marker = (marker.id, trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], distance)   
                
                    prev_marker = 0
                
                    index = 0
                    
                    while(index < len(self.seen_markers)):
                        if( (new_marker[1] < (self.seen_markers[index][1] + self.marker_delta)) and (new_marker[1] > (self.seen_markers[index][1] - self.marker_delta)) and
                            (new_marker[2] < (self.seen_markers[index][2] + self.marker_delta)) and (new_marker[2] > (self.seen_markers[index][2] - self.marker_delta)) or
                            (self.seen_markers[index][0] == new_marker[0]) ):
                            
                            rospy.loginfo('Copy Marker %d at X: %f, Y: %f', new_marker[0], new_marker[1], new_marker[2])
                            
                            
                            
                            # Previous marker, but we are closer / more accurate
                            if((new_marker[8] < self.seen_markers[index][8]) and (new_marker[8] > 1.0)):
                                rospy.loginfo('\t Using new location for marker %d', new_marker[0])
                                self.seen_markers[index] = new_marker
                                
                            prev_marker = 1
                            
                        index = index + 1
                            
                    
                    if(prev_marker == 0):
                        
                        self.seen_markers.append(new_marker)
                        rospy.loginfo('Found Marker %d at X: %f, Y: %f', new_marker[0], new_marker[1], new_marker[2])
                

    def _run(self):
        self.ar_map_sb = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self._marker_cb)
        
        run = 1
        
        while not(rospy.is_shutdown()) and (run == 1):
            if(len(self.seen_markers) >= self.num_targets):
                run = 0
                self.queue.put("DONE")
                
            if(not(self.queue.empty())):
                run = 0        
            
            self._publish_alvar_tfs()
            
            self.rate.sleep()
                
        self.ar_map_sb.unregister()
        
    # Get translation and rotation from source frame to target_frame
    def _get_transform(self, source_frame, target_frame):         
        trans = np.zeros(3)
        rot = np.zeros(4)
        
        success = -1
         
        if self.tf_lsnr.frameExists(source_frame) and self.tf_lsnr.frameExists(target_frame):
            try:
                now = rospy.Time(0)
                self.tf_lsnr.waitForTransform(source_frame, target_frame, now, rospy.Duration(1.0))
                trans, rot  = self.tf_lsnr.lookupTransform(source_frame, target_frame, now)
                
                success = 1
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
                
        return (success, trans, rot)
        
    def _publish_alvar_tfs(self):
        for prev in self.seen_markers:
            ar_frame = "usr_ar_marker_" + str(prev[0])
            
            backoff = 0.675
            
            quaternion = (prev[4], prev[5], prev[6], prev[7])
            
            euler = tf.transformations.euler_from_quaternion(quaternion)
            
            yaw = euler[2] + math.pi/2
            
            x_bck = backoff * math.cos(yaw)
            y_bck = backoff * math.sin(yaw)
            
            new_x = prev[1] - x_bck
            new_y = prev[2] - y_bck            
            
            self.start_bcst.sendTransform( (new_x, new_y, prev[3]),
                                           (prev[4], prev[5], prev[6], prev[7]),
                                           rospy.Time.now(), ar_frame, self.map_frame)
                                           
    def _return_markers(self):
        index = 0
        
        while(index < len(self.seen_markers)):
        
            self.queue.put(self.seen_markers[index])
            
            index = index + 1
                                      
    def _return_altered_markers(self):
        index = 0
        
        backoff = 0.675
        
        while(index < len(self.seen_markers)):
            quaternion = (self.seen_markers[index][4], 
                          self.seen_markers[index][5], 
                          self.seen_markers[index][6], 
                          self.seen_markers[index][7])
        
            euler = tf.transformations.euler_from_quaternion(quaternion)
            
            yaw = euler[2] + math.pi/2
            
            new_quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
            
            x_bck = backoff * math.cos(yaw)
            y_bck = backoff * math.sin(yaw)
            
            
            mkr_id = self.seen_markers[index][0]
            x = self.seen_markers[index][1] - x_bck
            y = self.seen_markers[index][2] - y_bck    
            z = self.seen_markers[index][3]
            rx = new_quaternion[0]
            ry = new_quaternion[1]
            rz = new_quaternion[2]
            rw = new_quaternion[3]
            dist = self.seen_markers[index][8]
            
            new_marker = (mkr_id, x, y, z, rx, ry, rz, rw, dist)
            
            self.seen_markers[index] = new_marker
        
            self.queue.put(self.seen_markers[index])
            
            index = index + 1                                      
          
    def _return_last_marker(self):
        self.queue.put(self.seen_markers[len(self.seen_markers)-1])                                 
                          
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
            
