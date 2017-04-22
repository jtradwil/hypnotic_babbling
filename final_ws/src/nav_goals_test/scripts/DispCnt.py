#!/usr/bin/env python

import numpy as np
import cv2
pink=0
yellow=1
green=1
blue=2
voilet=0
orange=1
pink_dim=(253,1241)
yellow_dim=(603,1254)
green_dim=(946,1264)
blue_dim=(1283,1251)
voilet_dim=(1633,1267)
orange_dim=(1953,1254)
img = cv2.imread('egg_count.jpg',1)

cv2.putText(img,str(pink), pink_dim, cv2.FONT_HERSHEY_TRIPLEX, 5, 255,5)
cv2.putText(img,str(yellow), yellow_dim, cv2.FONT_HERSHEY_TRIPLEX, 5, 255,5)
cv2.putText(img,str(green), green_dim, cv2.FONT_HERSHEY_TRIPLEX, 5, 255,5)
cv2.putText(img,str(blue), blue_dim, cv2.FONT_HERSHEY_TRIPLEX, 5, 255,5)
cv2.putText(img,str(voilet), voilet_dim, cv2.FONT_HERSHEY_TRIPLEX, 5, 255,5)
cv2.putText(img,str(orange), orange_dim, cv2.FONT_HERSHEY_TRIPLEX, 5, 255,5)

cv2.namedWindow('image',cv2.WINDOW_NORMAL)
cv2.imshow('image',img)
cv2.waitKey(0)
cv2.destroyAllWindows()

