#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import baxter_interface

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from cv_bridge import CvBridge, CvBridgeError


pub = rospy.Publisher('ball_position_cam', Point)


def initialize_threshold_trackbar():



	# Initialize thresholding values
	low_h  = 0
	high_h = 10
	low_s  = 110
	high_s = 255
	low_v  = 150
	high_v = 255
	#lower_red = np.array([0,110,150])
	#upper_red = np.array([10,255,255])

	#Creating slidable trackbars for HSV thresholding values
	# "Do nothing" callback
	def nothing(x):
		pass

	cv2.createTrackbar("Low H", "Control", low_h, 255, nothing)
	cv2.createTrackbar("High H", "Control", high_h, 255, nothing)
	cv2.createTrackbar("Low S", "Control", low_s, 255, nothing)
	cv2.createTrackbar("High S", "Control", high_s, 255, nothing)
	cv2.createTrackbar("Low V", "Control", low_v, 255, nothing)
	cv2.createTrackbar("High V", "Control", high_v, 255, nothing)

	print "Initialized."


def callback(message):


	#Capturing image of web camera
	bridge = CvBridge()

	cv_image = bridge.imgmsg_to_cv2(message, "bgr8")
	height, width, depth = cv_image.shape	


	#Converting image to HSV format
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


	#Thresholding image based on current trackbar values
	low_h  = cv2.getTrackbarPos("Low H", "Control")
	high_h = cv2.getTrackbarPos("High H", "Control")
	low_s  = cv2.getTrackbarPos("Low S", "Control")
	high_s = cv2.getTrackbarPos("High S", "Control")
	low_v  = cv2.getTrackbarPos("Low V", "Control")
	high_v = cv2.getTrackbarPos("High V", "Control")

	thresholded = cv2.inRange(hsv, np.array([low_h, low_s, low_v]), np.array([high_h, high_s, high_v]))
	#res = cv2.bitwise_and(cv_image, cv_image, mask= thresholded)


	#Morphological opening (remove small objects from the foreground)
	thresholded = cv2.erode(thresholded, np.ones((2,2), np.uint8), iterations=1)
	thresholded = cv2.dilate(thresholded, np.ones((2,2), np.uint8), iterations=1)

	#Morphological closing (fill small holes in the foreground)
	thresholded = cv2.dilate(thresholded, np.ones((2,2), np.uint8), iterations=1)
	thresholded = cv2.erode(thresholded, np.ones((2,2), np.uint8), iterations=1)


	#Get left hand range state from rangefinder
	#dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()

	#if dist>65000:
        #        print "==[VISION]== ERROR - calibrate_distance - no distance found"
	#else:
	#	print "Rangefinder distance to object:", distance

	
	#Finding center of red ball
	ret,thresh = cv2.threshold(thresholded,157,255,0)
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

	M = cv2.moments(thresh)
	
	A = M['m10']
	B = M['m00']

	if B>500:
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])

		P = Point()
		P.x = cx-(width)/2
		P.y = -(cy - (height/2))
		pub.publish(P)

		print "Center of ball: (", P.x, ", ", P.y, ")"

	else:
		print "Ball off screen."


	#Draw contour around object
	cv2.drawContours(thresholded,contours,-1,(255,0,0),3)


	#Printing to screen the images
	cv2.imshow("Original", cv_image)
	cv2.imshow("Thresholded", thresholded)
    	cv2.waitKey(3)


def listener():

	#Create names for OpenCV images and orient them appropriately
	cv2.namedWindow("Control", 1)
	cv2.namedWindow("Original", 2)
	cv2.namedWindow("Thresholded", 3)

	#Initialize threshol
	initialize_threshold_trackbar()


	#Initiate node for left hand camera
	rospy.init_node('left_hand_camera', anonymous=True)

	#Subscribe to left hand camera image 
	rospy.Subscriber("/cameras/right_hand_camera/image", Image, callback)
	#rospy.Subscriber("/camera_node/image_raw", Image, callback)






	#Keep from exiting until this node is stopped
	rospy.spin()
 
	
         
if __name__ == '__main__':
     listener()
