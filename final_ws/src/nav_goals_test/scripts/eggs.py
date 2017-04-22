#!/usr/bin/env python
import cv2
import numpy as np
import os

pinkLower = np.array([165, 89, 102], np.uint8)
pinkUpper = np.array([180, 204, 255], np.uint8)
yellowLower = np.array([22, 102, 153], np.uint8)
yellowUpper = np.array([33, 204, 255], np.uint8)
greenLower = np.array([75, 127, 38], np.uint8)
greenUpper = np.array([95, 255, 255], np.uint8)
blueLower = np.array([100, 127, 153], np.uint8)
blueUpper = np.array([113, 255, 255], np.uint8)
orangeLower = np.array([9, 127, 127], np.uint8)
orangeUpper = np.array([15, 255, 255], np.uint8)
purpleLower = np.array([120, 127, 127], np.uint8)
purpleUpper = np.array([128, 179, 255], np.uint8)

def crop_height(image):
    egg_x=0
    egg_y=0
    egg_w=0
    egg_h=0
    

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    height, width, channels = image.shape
    eggLower = np.array([0, 190, 190], np.uint8)
    eggUpper = np.array([180, 255, 255], np.uint8)
    maskEgg = cv2.inRange(hsv, eggLower, eggUpper)

    contours, hierarchy = cv2.findContours(maskEgg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:	
        x,y,w,h = cv2.boundingRect(cnt)	#fit a square around the target
        if (w*h>egg_w*egg_h):
	    egg_x=x
            egg_y=y
            egg_w=w
            egg_h=h
    if (egg_h>0):
        cropped=image[egg_y-float(0.2*egg_h):egg_y+egg_h+float(0.2*egg_h), 0:width]
    else:
	cropped=image
    return cropped

def avg_size(image):

    avg_size=0
    sizes=[]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    maskPink = cv2.inRange(hsv, pinkLower, pinkUpper)
    maskPink = cv2.erode(maskPink, None, iterations=2)
    maskPink = cv2.dilate(maskPink, None, iterations=12)
    contoursPink, hierarchy = cv2.findContours(maskPink,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    maskYellow = cv2.inRange(hsv, yellowLower, yellowUpper)
    maskYellow = cv2.erode(maskYellow, None, iterations=2)
    maskYellow = cv2.dilate(maskYellow, None, iterations=12)
    contoursYellow, hierarchy = cv2.findContours(maskYellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    maskGreen = cv2.inRange(hsv, greenLower, greenUpper)
    maskGreen = cv2.erode(maskGreen, None, iterations=2)
    maskGreen = cv2.dilate(maskGreen, None, iterations=12)
    contoursGreen, hierarchy = cv2.findContours(maskGreen,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    maskBlue = cv2.inRange(hsv, blueLower, blueUpper)
    maskBlue = cv2.erode(maskBlue, None, iterations=2)
    maskBlue = cv2.dilate(maskBlue, None, iterations=12)
    contoursBlue, hierarchy = cv2.findContours(maskBlue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    maskOrange = cv2.inRange(hsv, orangeLower, orangeUpper)
    maskOrange = cv2.erode(maskOrange, None, iterations=2)
    maskOrange = cv2.dilate(maskOrange, None, iterations=12)
    contoursOrange, hierarchy = cv2.findContours(maskOrange,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    maskPurple = cv2.inRange(hsv, purpleLower, purpleUpper)
    maskPurple = cv2.erode(maskPurple, None, iterations=2)
    maskPurple = cv2.dilate(maskPurple, None, iterations=12)
    contoursPurple, hierarchy = cv2.findContours(maskPurple,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    contours = contoursPurple+contoursOrange+contoursBlue+contoursGreen+contoursYellow+contoursPink
    for cnt in contours:
	x,y,w,h = cv2.boundingRect(cnt)	#fit a square around the target
        sizes.append(w*h)
    sizes.sort()
    new_sizes=[]
    if len(sizes)>0:
    	median=sizes[len(sizes)/2]
    	for number in sizes:
            if (number>median-(0.6*median))and (number<median+(0.6*median)):
	        new_sizes.append(number)
        avg_size=sum(new_sizes)/len(new_sizes)	
    else:
        avg_size=0
    
    return avg_size


def find(image, avg):

    pink=0
    yellow=0
    green=0
    blue=0
    orange=0
    purple=0

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    maskPink = cv2.inRange(hsv, pinkLower, pinkUpper)
    maskPink = cv2.erode(maskPink, None, iterations=2)
    maskPink = cv2.dilate(maskPink, None, iterations=12)
    contoursPink, hierarchy = cv2.findContours(maskPink,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contoursPink:
	x,y,w,h = cv2.boundingRect(cnt)	#fit a square around the target
	if (w*h<avg+(.40*avg))and (w*h>avg-(.40*avg)):
	    pink=pink+1

    maskYellow = cv2.inRange(hsv, yellowLower, yellowUpper)
    maskYellow = cv2.erode(maskYellow, None, iterations=2)
    maskYellow = cv2.dilate(maskYellow, None, iterations=12)
    contoursYellow, hierarchy = cv2.findContours(maskYellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contoursYellow:
	x,y,w,h = cv2.boundingRect(cnt)	#fit a square around the target
	if (w*h<avg+(.40*avg))and (w*h>avg-(.40*avg)):
	    yellow=yellow+1

    maskGreen = cv2.inRange(hsv, greenLower, greenUpper)
    maskGreen = cv2.erode(maskGreen, None, iterations=2)
    maskGreen = cv2.dilate(maskGreen, None, iterations=12)
    contoursGreen, hierarchy = cv2.findContours(maskGreen,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contoursGreen:
	green=green+1

    maskBlue = cv2.inRange(hsv, blueLower, blueUpper)
    maskBlue = cv2.erode(maskBlue, None, iterations=2)
    maskBlue = cv2.dilate(maskBlue, None, iterations=12)
    contoursBlue, hierarchy = cv2.findContours(maskBlue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contoursBlue:
	x,y,w,h = cv2.boundingRect(cnt)	#fit a square around the target
	if (w*h<avg+(.40*avg))and (w*h>avg-(.40*avg)):
		blue=blue+1

    maskOrange = cv2.inRange(hsv, orangeLower, orangeUpper)
    maskOrange = cv2.erode(maskOrange, None, iterations=2)
    maskOrange = cv2.dilate(maskOrange, None, iterations=12)
    contoursOrange, hierarchy = cv2.findContours(maskOrange,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contoursOrange:
	x,y,w,h = cv2.boundingRect(cnt)	#fit a square around the target
	if (w*h<avg+(.40*avg))and (w*h>avg-(.40*avg)):
            orange=orange+1

    maskPurple = cv2.inRange(hsv, purpleLower, purpleUpper)
    maskPurple = cv2.erode(maskPurple, None, iterations=2)
    maskPurple = cv2.dilate(maskPurple, None, iterations=12)
    contoursPurple, hierarchy = cv2.findContours(maskPurple,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contoursPurple:
	x,y,w,h = cv2.boundingRect(cnt)	#fit a square around the target
	if (w*h<avg+(.40*avg))and (w*h>avg-(.40*avg)):
	    purple=purple+1

    return [pink, yellow, green, blue, orange, purple]

