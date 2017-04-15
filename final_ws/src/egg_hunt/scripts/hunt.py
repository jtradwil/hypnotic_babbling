#!/usr/bin/env python

import rospy
import smach
import smach_ros
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from eggs import find
from eggs import avg_size
from eggs import crop_height
import time
import cv_bridge
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion

#global variables
rabbit_one=0
rabbit_two=0
rabbit_three=0

one_x=0
one_y=0
one_z=0

two_x=0
two_y=0
two_z=0

three_x=0
three_y=0
three_z=0

target=0

eggs_counted=0

# define state Map
class Map(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MAP')
        return 'outcome1'


# define state Explore
class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def marker_callback(self,data):
        global rabbit_one,rabbit_two, rabbit_three, one_x, one_y, one_z, two_x, two_y, two_z, three_x, three_y, three_z
	if len(data.markers)>0:	#if there is a alvar marker in the image
            if (data.markers[0].id == 1): #if the marker is one, set true and x, y, and z
		rabbit_one=1

            if (data.markers[0].id == 2): #if the marker is two, set true and x, y, and z
		rabbit_two=1

            if (data.markers[0].id == 3): #if the marker is three, set true and x, y, and z
		rabbit_three=1
           
    def execute(self, userdata):
        rospy.loginfo('Executing state EXPLORE')
        self.ar_exp_sb = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_callback)
        while((rabbit_one and rabbit_two and rabbit_three)!=1):	#loop until all three rabbits have been found
            #Explore functionality
	    1+1            
        self.ar_exp_sb.unregister()
        return 'outcome1'


# define state Return to start
class RtoS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RETURN TO START')
        return 'outcome1'


# define state Wait for Target
class WforT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        
    def marker_callback(self,data):
        global target
	if len(data.markers)>0:	#if there is a alvar marker in the image
	    target=data.markers[0].id

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT FOR TARGET')
        self.ar_id_sb = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_callback)
	while (target==0):
	    1+1
        print target
        self.ar_id_sb.unregister()
        return 'outcome1'


# define state Navigate to Target
class NtoT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state NAVIGATE TO TARGET')
        time.sleep(60)
        return 'outcome1'


# define state Count Eggs
class CntEggs(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.bridge = cv_bridge.CvBridge()
        
    def image_callback(self, msg):
        global eggs_counted
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        print find(crop_height(image),avg_size(crop_height(image)))
        eggs_counted=1
        self.image_sb.unregister()

    def execute(self, userdata):
        rospy.loginfo('Executing state COUNT EGGS')
        self.image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        while (eggs_counted==0):
	    1+1
        return 'outcome1'


# define state Display Count
class DispCnt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DISPLAY COUNT')
        #return 'outcome1'
        return 'outcome2'
        

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome3'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MAP', Map(), 
                               transitions={'outcome1':'EXPLORE'})
        smach.StateMachine.add('EXPLORE', Explore(), 
                               transitions={'outcome1':'RETURN_TO_START'})
        smach.StateMachine.add('RETURN_TO_START', RtoS(), 
                               transitions={'outcome1':'WAIT_FOR_TARGET'})
        smach.StateMachine.add('WAIT_FOR_TARGET', WforT(), 
                               transitions={'outcome1':'NAVIGATE_TO_TARGET'})
        smach.StateMachine.add('NAVIGATE_TO_TARGET', NtoT(), 
                               transitions={'outcome1':'COUNT_EGGS'})
        smach.StateMachine.add('COUNT_EGGS', CntEggs(), 
                               transitions={'outcome1':'DISPLAY_COUNT'})
        smach.StateMachine.add('DISPLAY_COUNT', DispCnt(), 
                               transitions={'outcome1':'RETURN_TO_START', 'outcome2':'outcome3'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
