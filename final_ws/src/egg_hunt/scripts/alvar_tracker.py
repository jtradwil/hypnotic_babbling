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
    marker_delta = 0.5
    
    map_frame = "map"

    def __init__(self, queue):
        self.queue = queue
        self.tf_lsnr = tf.TransformListener()
        self.start_bcst = tf.TransformBroadcaster()
        self.rate = rospy.Rate(self.run_rate)
        
    
    def _marker_cb(self, data):
        
        for marker in data.markers:
            ar_frame = "ar_marker_" + str(marker.id)
            
            (success, trans, rot) = self._get_transform("map", ar_frame)
            (r_success, r_trans, r_rot) = self._get_transform("base_link", ar_frame)
            
            if((success == 1) and (r_success == 1)):
                distance = np.linalg.norm(r_trans)
                
                new_marker = (marker.id, trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], distance)   
            
                prev_marker = 0
            
                for prev in self.seen_markers:                
                    if( (trans[0] < (prev[1] + self.marker_delta)) and (trans[0] > (prev[1] - self.marker_delta)) and
                        (trans[1] < (prev[2] + self.marker_delta)) and (trans[1] > (prev[2] - self.marker_delta)) or
                        (prev[0] == marker.id) ):
                        
                        rospy.loginfo('Copy Marker %d at X: %f, Y: %f', new_marker[0], new_marker[1], new_marker[2])
                        
                        # Previous marker, but we are closer / more accurate
                        if(distance < prev[8]):
                            rospy.loginfo('\t Using new location for marker %d', new_marker[0])
                            prev = new_marker
                            
                        prev_marker = 1
                        
                
                if(prev_marker == 0):
                    
                    self.seen_markers.append(new_marker)
                    rospy.loginfo('Found Marker %d at X: %f, Y: %f', new_marker[0], new_marker[1], new_marker[2])
            

    def _run(self):
        self.ar_map_sb = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self._marker_cb)
        
        run = 1
        
        while not(rospy.is_shutdown()) and (run == 1):
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
            
            self.start_bcst.sendTransform( (prev[1], prev[2], prev[3]),
                                           (prev[4], prev[5], prev[6], prev[7]),
                                           rospy.Time.now(), ar_frame, self.map_frame)
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
                                      
            
