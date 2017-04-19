#!/usr/bin/env python

# imports
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv_bridge

# Callback for image publisher
def image_cb(msg):
    try:
        srcA = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except cv_bridge.CvBridgeError as e:
        print(e)

    cv2.imshow("View", srcA)


# standard ros boilerplate
if __name__ == "__main__":
    try:            
        print(cv2.__version__)
                
        rospy.init_node('camera_view')
        rate = rospy.Rate(50)

        bridge = cv_bridge.CvBridge()
        image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, image_cb)

        while not rospy.is_shutdown():
            cv2.waitKey(3)
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass


