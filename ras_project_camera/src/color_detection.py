#!/usr/bin/env python 

import rospy

import cv2
import numpy as np
from matplotlib import pyplot as plt
from sensor_msgs import Image


#frame = cv2.imread('/home/albina/Documents/RAS/cv/green.jpg')
#frame = cv2.resize(frame, (0,0), fx=0.3, fy=0.3)
class camera_image(object):
	def __init__(self):
	
		rospy.init_node('camera_image', anonymous=True)
		rospy.Subscriber('/image_converter/output_video', Image, self.image_callback)
		
		rate = rospy.Rate(10) #10Hz
		while not roapy.is_shutdown():
			resized = cv2.resize(image,None,fx=0.3, fy=0.3, interpolation = cv2.INTER_AREA)
			image = cv2.GaussianBlur(resized,(9,9),20)

			# define range of green color in HSV
			lower_green = np.array([20,0,0])
			upper_green = np.array([80,255,255])

			# Convert BGR to HSV
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

			# Threshold the HSV image to get only blue colors
			mask = cv2.inRange(hsv, lower_green, upper_green)

			# Bitwise-AND mask and original image
			#res = cv2.bitwise_and(frame,frame, mask= mask)

			#cv2.imshow('frame',frame)
			#cv2.imshow('mask',mask)
			#cv2.imshow('res',res)
			#cv2.waitKey(0)
			#cv2.destroyAllWindows()

			#edges = cv2.Canny(mask,50,70)

			ret,thresh = cv2.threshold(mask,127,255,0)

			contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			#print(contours)
			#cv2.drawContours(frame, contours, -1, (0,255,0), 3)
			cnt =contours [0]
			M = cv2.moments(cnt)
			#print M
			x,y,w,h = cv2.boundingRect(cnt)
			#print (x,y,w,h)
			c_x = x+w/2
			c_y = y+h/2
			#print(c_x, c_y)

			cv2.rectangle(resized,(x,y),(x+w,y+h),(0,255,0),2)

			#cv2.imshow('Image1',resized)
			#cv2.imwrite('object with bounding box.jpg',resized)
			#cv2.waitKey(0)
			#cv2.destroyAllWindows()

			#publish coordinates


			rate.sleep()
		rospy.spin()

	def image_callback(self, msg):
		self.
