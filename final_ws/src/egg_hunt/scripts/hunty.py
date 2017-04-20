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
from tf import TransformListener
from geometry_msgs.msg import PoseStamped
import tf
import math

#global variables
rabbit_one=0
rabbit_two=0
rabbit_three=0

rabbit_one_pose = PoseStamped()
rabbit_two_pose = PoseStamped()
rabbit_three_pose = PoseStamped()

target=0

eggs_counted=0

def cvt_pose(pose):
    quaternion = (
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw=euler[2]
    x=math.sin(yaw)+pose.pose.position.x
    y=math.cos(yaw)+pose.pose.position.y

    if (yaw<3.14):    
        new_yaw=yaw+3.14
    else:
        new_yaw=yaw-3.14

    ret_pose=pose
    ret_pose.pose.position.x=x
    ret_pose.pose.position.y=y
    ret_pose.pose.position.z=0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, new_yaw)
    ret_pose.pose.orientation.x=quaternion[0]
    ret_pose.pose.orientation.y=quaternion[1]
    ret_pose.pose.orientation.z=quaternion[2]
    ret_pose.pose.orientation.w=quaternion[3]
    return ret_pose


# define state Map
class Map(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.tfl=TransformListener()	#create a tf listenter

    def map_marker_callback(self,data):
        global rabbit_one,rabbit_two, rabbit_three, rabbit_one_pose, rabbit_two_pose, rabbit_three_pose
        if len(data.markers)>0:	#if there is a alvar marker in the image

        tag_pose = PoseStamped()	#manipulate the alvar message into a pose
        tag_pose.header=data.markers[0].header
        tag_pose.pose=data.markers[0].pose.pose
        success=False
        while not success:
            try:
                tag_pose=self.tfl.transformPose("/map", tag_pose)	#try to transform
                success=True
            except Exception:
                time.sleep(0.01)

            if (data.markers[0].id == 1)and(rabbit_one==0): #if the marker is one, set true and x, y, and z
                rabbit_one=1
                rabbit_one_pose=tag_pose	#save the pose of the rabbit in the map frame

            if (data.markers[0].id == 2)and(rabbit_two==0): #if the marker is two, set true and x, y, and z
                rabbit_two=1
                rabbit_two_pose=tag_pose

            if (data.markers[0].id == 3)and(rabbit_three==0): #if the marker is three, set true and x, y, and z
                rabbit_three=1
                rabbit_three_pose=tag_pose

    def execute(self, userdata):
        rospy.loginfo('Executing state MAP')
        self.ar_map_sb = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.map_marker_callback)
        ####
        #Launch nodes and wait until complete
        ####
        self.ar_map_sb.unregister()
        return 'outcome1'


# define state Explore
class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.tfl=TransformListener()

    def ex_marker_callback(self,data):
        global rabbit_one,rabbit_two, rabbit_three, rabbit_one_pose, rabbit_two_pose, rabbit_three_pose
        if len(data.markers)>0:	#if there is a alvar marker in the image

        tag_pose = PoseStamped()	#make a pose from the alvar message
        tag_pose.header=data.markers[0].header
        tag_pose.pose=data.markers[0].pose.pose
        success=False
        while not success:
            try:
                tag_pose=self.tfl.transformPose("/map", tag_pose)	#tranform into the map frame
                success=True	
            except Exception:
                time.sleep(0.01)

            if (data.markers[0].id == 1)and(rabbit_one==0): #if the marker is one, set true and x, y, and z
                rabbit_one=1
                rabbit_one_pose=tag_pose

            if (data.markers[0].id == 2)and(rabbit_two==0): #if the marker is two, set true and x, y, and z
                rabbit_two=1
                rabbit_two_pose=tag_pose

            if (data.markers[0].id == 3)and(rabbit_three==0): #if the marker is three, set true and x, y, and z
                rabbit_three=1
                rabbit_three_pose=tag_pose

    def execute(self, userdata):
        global rabbit_one,rabbit_two, rabbit_three, rabbit_one_pose, rabbit_two_pose, rabbit_three_pose
        rospy.loginfo('Executing state EXPLORE')
        self.ar_exp_sb = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ex_marker_callback)
        ###delay until all rabbits are identified###
        while((rabbit_one and rabbit_two and rabbit_three)!=1):	#loop until all three rabbits have been found
            ####
            #Explore until all of the rabbits are found
            ####
            pass
                     
        self.ar_exp_sb.unregister()
        rabbit_one_pose=cvt_pose(rabbit_one_pose)
        rabbit_two_pose=cvt_pose(rabbit_two_pose)
        rabbit_three_pose=cvt_pose(rabbit_three_pose)
        return 'outcome1'


# define state Return to start
class RtoS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RETURN TO START')
        time.sleep(15)
        return 'outcome1'


# define state Wait for Target
class WforT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def wt_marker_callback(self,data):
        global target
        if len(data.markers)>0:	#if there is a alvar marker in the image
        target=data.markers[0].id	#set the target to the id of the tag in the frame

    def execute(self, userdata):
        global target
        rospy.loginfo('Executing state WAIT FOR TARGET')
        target=0	#clear target
        self.ar_id_sb = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.wt_marker_callback)	#subscribe
        while (target==0):	#wait until the target is set
            pass

        self.ar_id_sb.unregister()	#unsubscribe
        return 'outcome1'


# define state Navigate to Target
class NtoT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state NAVIGATE TO TARGET')
        time.sleep(15)
        return 'outcome1'


# define state Count Eggs
class CntEggs(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.bridge = cv_bridge.CvBridge()

    def image_callback(self, msg):
        global eggs_counted
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")	#convert image
        print find(crop_height(image),avg_size(crop_height(image)))	#count the eggs
        eggs_counted=1	#set flag
        self.image_sb.unregister()	#unsubscribe

    def execute(self, userdata):
        rospy.loginfo('Executing state COUNT EGGS')
        self.image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback) #subscribe to usb_cam topic
        while (eggs_counted==0):	#loop until eggs have been counted
            pass
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





