#!/usr/bin/env python
import cv2 
import numpy as np 
import imutils

# Webcamera no 0 is used to capture the frames 
cap = cv2.VideoCapture(0) 

#position_vector = Float32MultiArray()
#position_vector.data = []

flag = 0

# This drives the program into an infinite loop. 
while(1):	 
	# Captures the live stream frame-by-frame 
	_, frame = cap.read() 
	# Converts images from BGR to HSV 
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
	lower_color_thres = np.array([0,150,150]) 
	upper_color_thres = np.array([255,255, 255])
	# Here we are defining range of bluecolor in HSV 
	# This creates a mask of blue coloured 
	# objects found in the frame. 
	mask = cv2.inRange(hsv, lower_color_thres, upper_color_thres) 
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None
	
	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid

		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)

		if flag == 0:
			center0a = int(M["m10"] / M["m00"])
			center0b = int(M["m01"] / M["m00"])
			flag = 1

		#position_vector.data = [float(int(M["m10"] / M["m00"]) - center0a)/1000, float(int(M["m01"] / M["m00"]) - center0b)/1000]
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		# only proceed if the radius meets a minimum size
		if radius > 1:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
	
	# The bitwise and of the frame and mask is done so 
	# that only the blue coloured objects are highlighted 
	# and stored in res 
	res = cv2.bitwise_and(frame,frame, mask= mask) 
	cv2.imshow('frame',frame) 
	cv2.imshow('mask',mask) 
	cv2.imshow('res',res) 

	# This displays the frame, mask 
	# and res which we created in 3 separate windows. 
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

# Destroys all of the HighGUI windows. 
cv2.destroyAllWindows() 

# release the captured frame 
cap.release() 
