#!/usr/bin/env python

import rospy
import smach
import smach_ros
import cv2
import numpy as np
import os

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

    def execute(self, userdata):
        rospy.loginfo('Executing state EXPLORE')
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

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT FOR TARGET')
        return 'outcome1'


# define state Navigate to Target
class NtoT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state NAVIGATE TO TARGET')
        return 'outcome1'


# define state Count Eggs
class CntEggs(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state COUNT EGGS')
        
	cap = cv2.VideoCapture(0)
	ret, image = cap.read()
    	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        pinkLower = np.array([165, 127, 178], np.uint8)
    	pinkUpper = np.array([175, 166, 255], np.uint8)
    	maskPink = cv2.inRange(hsv, pinkLower, pinkUpper)
    	maskPink = cv2.erode(maskPink, None, iterations=2)
    	maskPink = cv2.dilate(maskPink, None, iterations=8)
    	contoursPink, hierarchy = cv2.findContours(maskPink,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    	print(str(len(contoursPink))+" Pink Eggs")

    	yellowLower = np.array([22, 102, 224], np.uint8)
    	yellowUpper = np.array([30, 153, 255], np.uint8)
    	maskYellow = cv2.inRange(hsv, yellowLower, yellowUpper)
    	maskYellow = cv2.erode(maskYellow, None, iterations=2)
    	maskYellow = cv2.dilate(maskYellow, None, iterations=8)
    	contoursYellow, hierarchy = cv2.findContours(maskYellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    	print(str(len(contoursYellow))+" Yellow Eggs")

    	greenLower = np.array([82, 153, 114], np.uint8)
    	greenUpper = np.array([95, 217, 204], np.uint8)
    	maskGreen = cv2.inRange(hsv, greenLower, greenUpper)
    	maskGreen = cv2.erode(maskGreen, None, iterations=2)
    	maskGreen = cv2.dilate(maskGreen, None, iterations=8)
    	contoursGreen, hierarchy = cv2.findContours(maskGreen,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    	print(str(len(contoursGreen))+" Green Eggs")

    	blueLower = np.array([100, 204, 242], np.uint8)
    	blueUpper = np.array([110, 255, 255], np.uint8)
    	maskBlue = cv2.inRange(hsv, blueLower, blueUpper)
    	maskBlue = cv2.erode(maskBlue, None, iterations=2)
    	maskBlue = cv2.dilate(maskBlue, None, iterations=8)
    	contoursBlue, hierarchy = cv2.findContours(maskBlue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    	print(str(len(contoursBlue))+" Blue Eggs")

    	orangeLower = np.array([10, 153, 229], np.uint8)
    	orangeUpper = np.array([14, 242, 255], np.uint8)
    	maskOrange = cv2.inRange(hsv, orangeLower, orangeUpper)
    	maskOrange = cv2.erode(maskOrange, None, iterations=2)
    	maskOrange = cv2.dilate(maskOrange, None, iterations=8)
    	contoursOrange, hierarchy = cv2.findContours(maskOrange,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    	print(str(len(contoursOrange))+" Orange Eggs")

    	purpleLower = np.array([120, 127, 191], np.uint8)
    	purpleUpper = np.array([128, 153, 255], np.uint8)
    	maskPurple = cv2.inRange(hsv, purpleLower, purpleUpper)
    	maskPurple = cv2.erode(maskPurple, None, iterations=2)
    	maskPurple = cv2.dilate(maskPurple, None, iterations=8)
    	contoursPurple, hierarchy = cv2.findContours(maskPurple,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    	print(str(len(contoursPurple))+" Purple Eggs")

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
