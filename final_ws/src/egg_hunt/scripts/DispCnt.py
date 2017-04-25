#!/usr/bin/env python

import numpy as np
import cv2
import rospkg
import rospy



id_dim=(1000,650)
pink_dim=(253,1241)
yellow_dim=(603,1254)
green_dim=(946,1264)
blue_dim=(1283,1251)
orange_dim=(1953,1254)
voilet_dim=(1633,1267)


def update():
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rabbit_id=rospy.get_param('rabbit_id')
        colors=rospy.get_param('colors')
        pink=colors[0]
        yellow=colors[1]
        green=colors[2]
        blue=colors[3]
        orange=colors[4]
        voilet=colors[5]

        img = cv2.imread(str(rospkg.RosPack().get_path('egg_hunt'))+"/scripts/egg_count.jpg",1)

        cv2.putText(img,str(rabbit_id), id_dim, cv2.FONT_HERSHEY_TRIPLEX, 5, 1,5)
        cv2.putText(img,str(pink), pink_dim, cv2.FONT_HERSHEY_TRIPLEX, 5, 1,5)
        cv2.putText(img,str(yellow), yellow_dim, cv2.FONT_HERSHEY_TRIPLEX, 5, 1,5)
        cv2.putText(img,str(green), green_dim, cv2.FONT_HERSHEY_TRIPLEX, 5, 1,5)
        cv2.putText(img,str(blue), blue_dim, cv2.FONT_HERSHEY_TRIPLEX, 5, 1,5)
        cv2.putText(img,str(voilet), voilet_dim, cv2.FONT_HERSHEY_TRIPLEX, 5, 1,5)
        cv2.putText(img,str(orange), orange_dim, cv2.FONT_HERSHEY_TRIPLEX, 5, 1,5)
    
    
        cv2.imshow('image',img)
        cv2.waitKey(1)
        cv2.resizeWindow('image', 600,600)
        rate.sleep()
    cv2.destroyAllWindows()

def init():
    rospy.init_node('count_display', anonymous=False)
    cv2.namedWindow('image',cv2.WINDOW_NORMAL)

# standard ros boilerplate
if __name__ == "__main__":
    try:
	    init()
	    update()
    except rospy.ROSInterruptException:
        pass

