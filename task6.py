#!/usr/bin/env python

## RO-06-Germanidis-Witt

import roslib
import sys
import rospy
import cv2
import numpy as np
from sklearn import linear_model

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def find_white_pixels_gray(image):
	white_pixels = np.empty(shape=[1,2])
	for y in range(image.shape[0]):
		for x in range(image.shape[1]):	
			if image[y,x] == 255:				#255 means white	
				white_pixels=np.append(white_pixels,[[y,x]],axis=0)	#add coordinates
	return white_pixels

def find_white_pixels_hsv(image):
	white_pixels = np.empty(shape=[1,2])
	for y in range(image.shape[0]):
		for x in range(image.shape[1]):	
			if image[y,x][2] == 255:				#V=255 means white	
				white_pixels=np.append(white_pixels,[[y,x]],axis=0)	#add coordinates
	return white_pixels

def find_white_pixels_yuv(image):
	white_pixels = np.empty(shape=[1,2])
	for y in range(image.shape[0]):
		for x in range(image.shape[1]):	
			if image[y,x][0] != 0:				#Y=0 means black	
				white_pixels=np.append(white_pixels,[[y,x]],axis=0)	#add coordinates
	return white_pixels

def get_line_equation(points):	
	X_1 = np.empty([])
	y_1 = np.empty([])	
	X_2 = np.empty([])
	y_2 = np.empty([])
	#assumption: 2 lanes
	X = points[:,1]
	y = points[:,0]
	X_mean = np.mean(X) #calculate the mean of all x-values from the white points
	for i in range(len(X)): #divide the points in two groups
		if X[i]>=X_mean:
			X_1 = np.append(X_1,X[i])
			y_1 = np.append(y_1,y[i])
		else:
			X_2 = np.append(X_2,X[i])
			y_2 = np.append(y_2,y[i])
	X_1 = np.transpose(np.matrix(X_1))	
	y_1 = np.transpose(np.matrix(y_1))
	X_2 = np.transpose(np.matrix(X_2))
	y_2 = np.transpose(np.matrix(y_2))
	ransac = linear_model.RANSACRegressor()	#RANSAC
	ransac.fit(X_1, y_1)										#fit first line
	#y=m*x+b 
	b1 = ransac.predict(0)		
	m1 = (ransac.predict(200)-b1)/200
	ransac.fit(X_2, y_2)										#fit second line
	b2 = ransac.predict(0)
	m2 = (ransac.predict(200)-b2)/200
	return m1,b1,m2,b2




class lane_seg:
	wp_img_gray = np.empty([])
	m1_gray = []
	wp_img_hsv = np.empty([])
	m1_hsv = []
	wp_img_yuv = np.empty([])
	m1_yuv = []

	def __init__(self):
		self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)  #get the raw image			
		self.bridge = CvBridge()   	
		self.pub_img_gray = rospy.Publisher("/image_processing/image_gray",Image, queue_size=1)    #publisher for the gray image
		self.pub_img_hsv = rospy.Publisher("/image_processing/image_hsv",Image, queue_size=1)      #publisher for the hsv image
		self.pub_img_yuv = rospy.Publisher("/image_processing/image_yuv",Image, queue_size=1)      #publisher for the yuv image
		self.pub_img_gray_lines = rospy.Publisher("/image_processing/image_gray_lines",Image, queue_size=1) #publisher for the gray image with lines
		self.pub_img_hsv_lines = rospy.Publisher("/image_processing/image_hsv_lines",Image, queue_size=1)   #publisher for the hsv image with lines
		self.pub_img_yuv_lines = rospy.Publisher("/image_processing/image_yuv_lines",Image, queue_size=1)   #publisher for the yuv image with lines

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		#convert RGB to GRAY
		img_gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		#convert RGB to HSV
		img_hsv=cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		#convert RGB to YUV
		img_yuv=cv2.cvtColor(cv_image, cv2.COLOR_BGR2YUV)

		#processing img_gray
		gray_max = 255
		gray_min = 190	
		ret,img_gray=cv2.threshold(img_gray, gray_min, gray_max, cv2.THRESH_BINARY);

		#processing img_hsv 
		hsv_lowB = np.array([0,0,128])								#the relevant value is V
		hsv_upB = np.array([255,255,255])       			#V=0 means black
		mask_hsv = cv2.inRange(img_hsv, hsv_lowB, hsv_upB) 					#create a mask
		img_hsv = cv2.bitwise_and(img_hsv,img_hsv, mask= mask_hsv)  #use the mask
   
		#processing the img_yuv
		yuv_lowB = np.array([128,0,0])					#the relevantest value is Y
		yuv_upB = np.array([255,255,255])				#Y=255 means white
		mask_yuv = cv2.inRange(img_yuv, yuv_lowB, yuv_upB)					#create a mask
		img_yuv = cv2.bitwise_and(img_yuv,img_yuv, mask= mask_yuv)  #use the mask

		#find the line equations
		x_max = cv_image.shape[1]
		
		#in the gray image
		if lane_seg.wp_img_gray.all():
			lane_seg.wp_img_gray = find_white_pixels_gray(img_gray)
			lane_seg.m1_gray,lane_seg.b1_gray,lane_seg.m2_gray,lane_seg.b2_gray = get_line_equation(lane_seg.wp_img_gray)	
			print "Grayscale"
			print "y = ", lane_seg.m1_gray, "* x + ", lane_seg.b1_gray
			print "y = ", lane_seg.m2_gray, "* x + ", lane_seg.b2_gray
		img_gray=cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)		#convert GRAY to RGB for visualisation
		self.pub_img_gray.publish(self.bridge.cv2_to_imgmsg(img_gray,"bgr8"))	#publish the gray image
		if lane_seg.m1_gray: #add the lines to the image
			cv2.line(img_gray,(0,0*lane_seg.m1_gray+lane_seg.b1_gray),(x_max,x_max*lane_seg.m1_gray+lane_seg.b1_gray),(0,0,255),5)
			cv2.line(img_gray,(0,0*lane_seg.m2_gray+lane_seg.b2_gray),(x_max,x_max*lane_seg.m2_gray+lane_seg.b2_gray),(0,0,255),5)

		#in the hsv image
		if lane_seg.wp_img_hsv.all():
			lane_seg.wp_img_hsv = find_white_pixels_hsv(img_hsv)
			lane_seg.m1_hsv,lane_seg.b1_hsv,lane_seg.m2_hsv,lane_seg.b2_hsv = get_line_equation(lane_seg.wp_img_hsv)
			print "HSV"
			print "y = ", lane_seg.m1_hsv, "* x + ", lane_seg.b1_hsv
			print "y = ", lane_seg.m2_hsv, "* x + ", lane_seg.b2_hsv
		img_hsv=cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR) 		#convert HSV to RGB for visualisation
		self.pub_img_hsv.publish(self.bridge.cv2_to_imgmsg(img_hsv,"bgr8"))	#publish the hsv image
		if lane_seg.m1_hsv: #add the lines to the image
			cv2.line(img_hsv,(0,0*lane_seg.m1_hsv+lane_seg.b1_hsv),(x_max,x_max*lane_seg.m1_hsv+lane_seg.b1_hsv),(0,0,255),5)
			cv2.line(img_hsv,(0,0*lane_seg.m2_hsv+lane_seg.b2_hsv),(x_max,x_max*lane_seg.m2_hsv+lane_seg.b2_hsv),(0,0,255),5)

		#in the yuv image
		if lane_seg.wp_img_yuv.all():
			lane_seg.wp_img_yuv = find_white_pixels_yuv(img_yuv)
			lane_seg.m1_yuv,lane_seg.b1_yuv,lane_seg.m2_yuv,lane_seg.b2_yuv = get_line_equation(lane_seg.wp_img_yuv)
			print "YUV"
			print "y = ", lane_seg.m1_yuv, "* x + ", lane_seg.b1_yuv
			print "y = ", lane_seg.m2_yuv, "* x + ", lane_seg.b2_yuv
		img_yuv=cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)		#convert YUV to RGB for visualisation
		self.pub_img_yuv.publish(self.bridge.cv2_to_imgmsg(img_yuv,"bgr8"))	#publish the yuv image
		if lane_seg.m1_yuv: #add the lines to the image
			cv2.line(img_yuv,(0,0*lane_seg.m1_yuv+lane_seg.b1_yuv),(x_max,x_max*lane_seg.m1_yuv+lane_seg.b1_yuv),(0,0,255),5)
			cv2.line(img_yuv,(0,0*lane_seg.m2_yuv+lane_seg.b2_yuv),(x_max,x_max*lane_seg.m2_yuv+lane_seg.b2_yuv),(0,0,255),5)

		try:  
			self.pub_img_gray_lines.publish(self.bridge.cv2_to_imgmsg(img_gray,"bgr8"))	#publish images with lines
			self.pub_img_hsv_lines.publish(self.bridge.cv2_to_imgmsg(img_hsv,"bgr8"))	      
			self.pub_img_yuv_lines.publish(self.bridge.cv2_to_imgmsg(img_yuv,"bgr8"))	
		except CvBridgeError as e:
			print(e)
      

def main(args):
	rospy.init_node('lane_seg', anonymous=True)
	ic = lane_seg()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)




