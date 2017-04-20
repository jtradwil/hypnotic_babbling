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
    def __init__(self, queue):
        self.queue = queue
    
    def _marker_cb(self, data):
        if len(data.markers)>0:	#if there is a alvar marker in the image
            pass

    def _run(self):
        self.ar_map_sb = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._marker_cb)
