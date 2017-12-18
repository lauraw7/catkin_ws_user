#!/usr/bin/env python

## RO-07-Germanidis-Witt

import roslib
import sys
import rospy
import cv2
import math
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


def yaw_to_quaternion(yaw):
#convert a yaw angle (in radians) into a Quaternion message
	return Quaternion(0, 0, math.sin(yaw / 2), math.cos(yaw / 2))

def find_colored_pixels(image):
	colored_pixels = np.empty(shape=[1,2])
	for y in range(image.shape[0]):
		for x in range(image.shape[1]):	
			if image[y,x] != 0:											#0 means black	
				colored_pixels=np.append(colored_pixels,[[x,y]],axis=0)	#add coordinates
	return colored_pixels[1:,:]

def group_pixels(colored_pixels):
	colored_pixels = colored_pixels[np.argsort(colored_pixels[:,0])]	#sort colored_pixels by x-coordinate
	dist_pixel = 20 		
	points = []
	counter = 1
	x = colored_pixels[0,0]
	y = colored_pixels[0,1]		
	sumx = x
	sumy = y
	for index in range(len(colored_pixels)):
		if math.sqrt((colored_pixels[index,0] - x)**2+(colored_pixels[index,1]-y)**2) <= dist_pixel:
			sumx = sumx + colored_pixels[index,0]
			sumy = sumy + colored_pixels[index,1]
			counter = counter+1
		else: 
			if counter > 30: 
				points.append([sumx/counter,sumy/counter])
			x = colored_pixels[index,0]
			y = colored_pixels[index,1]		
			sumx = x
			sumy = y
 			counter = 1
	if counter > 30:
		points.append([sumx/counter,sumy/counter])
	#filtering out points which have a small distance to each other
	ind_del = []	
	for i in range(len(points)):
		differences = []
		for b in range(1,len(points)-i):
			differences.append(math.sqrt((points[i+b][0]-points[i][0])**2+(points[i+b][1]-points[i][1])**2))
		if differences:
			if min(differences) <= 20:
				ind_del.append(differences.index(min(differences))+b)
	for a in range(len(ind_del)):
		points = np.delete(points,ind_del[a],0)
				
	return np.rint(points)	# auf int runden
		
def map_color_bulb(color):
	r_opt = 180			#define optimal H-values
	b_opt = 100
	g_opt = 70
	p_opt = 140
	hsv_color = cv2.cvtColor(np.uint8([[color]]), cv2.COLOR_BGR2HSV)	#convert RGB into HSV-colors
	H = hsv_color[0][0][0]
	
	differences = [np.absolute(r_opt-H), np.absolute(b_opt-H), np.absolute(g_opt-H), np.absolute(p_opt-H)] #calculate differences 

	if np.absolute(r_opt-H) == min(differences):			#chose the color with the littlest difference
		bulb = 'red'
	elif np.absolute(b_opt-H) == min(differences):
		bulb = 'blue'
	elif np.absolute(g_opt-H) == min(differences):
		bulb = 'green'
	elif np.absolute(p_opt-H) == min(differences):
		bulb = 'purple'
	else:
		bulb = 'kA'
	return bulb 


def find_colors(colored_pixels,img):
	colors = []
	for index in range(len(colored_pixels)):
		x = int(colored_pixels[index,0])
 		y = int(colored_pixels[index,1])
		#while img[y,x,0] >= 250 and img[y,x,1] >= 250 and img[y,x,2] >= 250 and x<=img.shape[1] and y<=img.shape[0]: #R,G und B >= 250
		#	x += 1
		#	y += 1
		farbe = map_color_bulb(img[y,x])
		colors.append([colored_pixels[index,0],colored_pixels[index,1],farbe])
	return colors	

def calc_circle(c,nr1,nr2,car_pos, img):
	carx = img.shape[1]/2 
	cary = img.shape[0]/2 

	x1 = c[nr1][0]
	y1 = c[nr1][1]
	x2 = c[nr2][0]
	y2 = c[nr2][1]

	dL1L2 = np.sqrt((x1-x2)**2+(y1-y2)**2)			#difference between Lamp1 and Lamp2
	#cv2.line(img,(int(x1),int(y1)),(int(x2),int(y2)),(255,0,0),2)
	
	dL1C = np.sqrt((x1-carx)**2+(y1-cary)**2)		#difference between Lamp1 and Car
	dL2C = np.sqrt((carx-x2)**2+(cary-y2)**2)		#difference between Lamp2 and Car
	alpha = np.arccos((dL1L2**2-dL1C**2-dL2C**2)/(-2*dL1C*dL2C)) #*180/3.1415	
	mittelpunktL1L2 = [(x1+x2)/2,(y1+y2)/2]			#calculation of the "Mittelsenkrechte"
	steigung = (y2-y1)/(x2-x1)
	ms_m = -1/steigung
	ms_b = mittelpunktL1L2[1]-ms_m*mittelpunktL1L2[0]
	#cv2.line(img,(0,int(0*ms_m+ms_b)),(400,int(400*ms_m+ms_b)),(0,0,255),2)

	if np.absolute(x2-x1)>=np.absolute(y2-y1):
		ind = [x1,x2].index(min([x1,x2]))
	else:	
		ind = [y1,y2].index(min([y1,y2]))
	xmin = [x1,x2][ind]
	ymin = [y1,y2][ind]
	Hx = xmin+np.cos(alpha)*500						#calculation of the "Lot"
	Hy = ymin+np.sin(alpha)*500
	#cv2.line(img,(0,int(0*(Hy-y1)/(Hx-x1)+(y1-(Hy-y1)/(Hx-x1)*x1))),(400,int(400*(Hy-y1)/(Hx-x1)+(y1-(Hy-y1)/(Hx-x1)*x1))),(0,nr2*50,0),2)
	l_m = -1/((Hy-ymin)/(Hx-xmin))+0.001
	l_b = ymin-l_m*xmin+0.001
	#cv2.line(img,(0,int(0*l_m+l_b)),(400,int(400*l_m+l_b)),(0,0,255),2)
	Mx = (ms_b-l_b)/(l_m-ms_m)						#calculation of the center of the circle
	My = ms_m*Mx+ms_b
	#cv2.circle(img,(int(Mx),int(My)),3,[100,100,0],3) #img,center,radius,color,thickness
	
	r = np.sqrt((dL1L2/2)**2+(mittelpunktL1L2[0]-Mx)**2+(mittelpunktL1L2[1]-My)**2) #calculation of the radius
	return int(Mx),int(My),int(r)

def find_intersections(Mx1,My1,r1,Mx2,My2,r2):	#find all intersections from two given circles
	c = np.sqrt((Mx2-Mx1)**2+(My2-My1)**2)
	x = (r1**2+c**2-r2**2)/(2*c)
	if np.absolute(x) > r1:
		intersections = []
	else:	
		y = np.sqrt(r1**2-x**2)
		S1x = Mx1+x*(Mx2-Mx1)/c-y*(My2-My1)/c
		S1y = My1+x*(My2-My1)/c+y*(Mx2-Mx1)/c
		S2x = Mx1+x*(Mx2-Mx1)/c+y*(My2-My1)/c
		S2y = My1+x*(My2-My1)/c-y*(Mx2-Mx1)/c
		intersections = [[S1x,S1y],[S2x,S2y]]	
	return intersections

def select_intersection(old_pos,intersections):
	differences = []
	for i in range(len(intersections)):
		differences.append(np.sqrt((old_pos[0]-intersections[i][0])**2+(old_pos[1]-intersections[i][1])**2))
		ind = differences.index(min(differences)) 
	return intersections[ind] #return the intersection with the littlest difference to the last car position

def calc_world_coor(c,car_pos):	#convert pixel in world coordinates
	intrinsic = np.matrix([[547.439725, 0.000000, 648.637375], [0.000000, 549.658679, 291.874735], [0.000000, 0.000000, 1.000000]])
	distortion = np.matrix([-0.262394, 0.047130, 0.002927, 0.004176, 0.000000])
	realworld_points = []
	image_points = np.empty(shape=[len(c),2])
	for i in range(len(c)):
		image_points[i][0] = c[i][0]		
		image_points[i][1] = c[i][1]
		if c[i][2] == 'blue':
			realworld_points.append((4.18,1.77,0))
		elif c[i][2] == 'green':
			realworld_points.append((2.29,1.14,0))
		elif c[i][2] == 'purple':
			realworld_points.append((2.29,2.40,0))
		elif c[i][2] == 'red':
			realworld_points.append((3.55,3.03,0))
	realworld_points = np.matrix(realworld_points)
	ret, rotation_vec, translation_vec = cv2.solvePnP(realworld_points, image_points, intrinsic, distortion)
	rotation_mat, jacobian = cv2.Rodrigues(rotation_vec);	#obtain the rotation matrix from the rotation vector
	extrinsic = np.concatenate((np.matrix(rotation_mat[:,0]).T, np.matrix(rotation_mat[:,1]).T, translation_vec),axis=1)
	homography = np.dot(intrinsic,extrinsic) 				#calculate homography-matrix
	pixel_point = np.matrix([car_pos[0],car_pos[1],1])		#add a z-component to the pixel-coordinates
	coor = np.dot(np.linalg.inv(homography),pixel_point.T)  #calculate 3D-world-coordinates
	coor = coor/coor[2]										#normalize 3D-world-coordinates by z-component
	inv_rotation_mat = np.linalg.inv(rotation_mat)
	yaw = -np.arctan2(inv_rotation_mat[1][0],inv_rotation_mat[0][0])
	return coor[0], coor[1], yaw	#return x- and y-coordinate and the yaw angle

class visual_gps:
	s_y = []
	s_x=[]
	saved_pos_x=[]	
	saved_pos_y=[]
	saved_pos_x_add = []
	saved_pos_y_add = []
	car_pos = [320, 240]
	car_x_old=320
	car_y_old=240
	counter = 0;	
	def __init__(self):
		self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback, queue_size=1)  #get the raw image			
		self.bridge = CvBridge()   	
		self.pub_img_res = rospy.Publisher("/image_processing/img_result",Image, queue_size=1)      #publisher for the result
		self.pub_odom = rospy.Publisher("/odom_gps",Odometry, queue_size=50)      						#publisher for the odometry

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		#preprocessing the image (step 1 for filtering out other lights)
		img_hsv=cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		hsv_lowB = np.array([0,160,0])								
		hsv_upB = np.array([255,255,255])       			
		mask_hsv = cv2.inRange(img_hsv, hsv_lowB, hsv_upB) 			#create a mask
		img_hsv = cv2.bitwise_and(img_hsv,img_hsv, mask= mask_hsv)  #use the mask
		cv_image = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)

		#find light bulb pixels
		img_gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		gray_max = 80	
		gray_min = 50	
		ret,img_gray=cv2.threshold(img_gray, gray_min, gray_max, cv2.THRESH_BINARY);
		
		colored_pixels = find_colored_pixels(img_gray)

		#group pixels to have only one per bulb
		cp = group_pixels(colored_pixels)

		#find colors of the light bulbs
		c = find_colors(cp,cv_image)
	
		#publish found light bulbs in 'gray' image
		img_res = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)

		for i in range(len(c)):
			if c[i][2] == 'red':
				cv2.circle(img_res,(int(c[i][0]),int(c[i][1])),5,[0,0,255],10) #img,center,radius,color,thickness
			elif c[i][2] == 'blue':
				cv2.circle(img_res,(int(c[i][0]),int(c[i][1])),5,[255,0,0],10) #img,center,radius,color,thickness
			elif c[i][2] == 'green':
				cv2.circle(img_res,(int(c[i][0]),int(c[i][1])),5,[0,255,0],10) #img,center,radius,color,thickness
			elif c[i][2] == 'purple':
				cv2.circle(img_res,(int(c[i][0]),int(c[i][1])),5,[150,0,150],10) #img,center,radius,color,thickness
		
		if len(c)>=3 and c[0][2] != c[1][2] and c[0][2] != c[2][2] and c[1][2] != c[2][2]:	#at least 3 different lamps are visible
			#circle 1	
			[Mx1,My1,r1] = calc_circle(c,0,1,visual_gps.car_pos,img_res) 
			cv2.circle(img_res,(int(Mx1),int(My1)),int(r1),[200,0,0],3) #img,center,radius,color,thickness
			#circle 2		
			[Mx2,My2,r2] = calc_circle(c,0,2,visual_gps.car_pos,img_res) 
			cv2.circle(img_res,(int(Mx2),int(My2)),int(r2),[0,200,0],3) #img,center,radius,color,thickness
		
			#find intersections
			intersections = find_intersections(Mx1,My1,r1,Mx2,My2,r2)
			if intersections == []:
				print 'Keine Schnittpunkte gefunden'
			else:
				cv2.circle(img_res,(int(intersections[0][0]),int(intersections[0][1])),2,[255, 255, 255],10) #img,center,radius,color,thickness
				cv2.circle(img_res,(int(intersections[1][0]),int(intersections[1][1])),2,[255, 255, 255],10) #img,center,radius,color,thickness

				#select right intersection and calculate world coordinates
				dif_sum = 10000;
				for i in range(len(intersections)):
					[xt,yt,yawt] = calc_world_coor(c,intersections[i])
					dif_x = np.absolute(visual_gps.car_x_old-xt)
					dif_y = np.absolute(visual_gps.car_y_old-yt)
					if (dif_x+dif_y)<dif_sum:
						dif_sum = dif_x+dif_y
						car_x = xt
						car_y = yt
						yaw = yawt
						ind_i = i
				right_intersection = intersections[ind_i]			
				cv2.circle(img_res,(int(right_intersection[0]),int(right_intersection[1])),2,[0, 0, 150],5) #img,center,radius,color,thickness

				visual_gps.car_x_old = car_x
				visual_gps.car_y_old = car_y

				rospy.loginfo('\n \n car position: \n x: %s   y: %s\n yaw_angle: %s',car_x,car_y,yaw)
				quaternion = yaw_to_quaternion(yaw)
				#create the odometry message
				odom_gps = Odometry()
				odom_gps.header.stamp = rospy.Time.now()
				odom_gps.header.frame_id = "odom"
				odom_gps.pose.pose = Pose(Point(car_x,car_y,0),quaternion)
				odom_gps.child_frame_id = "base_link"
				odom_gps.twist.twist = Twist(Vector3(0.1, -0.1, 0), Vector3(0, 0, 0.1))
				self.pub_odom.publish(odom_gps)

		print 'end callback'

		try:  
			self.pub_img_res.publish(self.bridge.cv2_to_imgmsg(img_res,"bgr8"))	      
		except CvBridgeError as e:
			print(e)
      

def main(args):
	rospy.init_node('visual_gps', anonymous=True)
	ic = visual_gps()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)




