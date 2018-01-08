#!/usr/bin/env python

## RO-07-Germanidis-Witt

import roslib
import sys
import rospy
import cv2
import math
import random
import numpy as np
import matplotlib.pyplot as plt

from functions_t9 import init_particles, move_particles, calc_angles_particles2balloons, calc_weights, resample_particles, new_marker, init_field, calc_position
from balloon_detector import BalloonDetector

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from visualization_msgs.msg import MarkerArray, Marker

class mcpf:
	odom_updates = 0
	x = []			#x position
	y = []			#y position
	z = []			#angle
	particles_first_moved = 0

	def __init__(self): 	
		#Initialize a set of 100 random particles
		mcpf.particles = init_particles()
		mcpf.field = init_field() #markers to show the field borders

		self.detector = BalloonDetector() 
		self.bridge = CvBridge()   
		self.pub_pcloud = rospy.Publisher("/mcmarkerarray",MarkerArray, queue_size=1)      #publisher for the markers		
		self.pub_field = rospy.Publisher("/field",MarkerArray, queue_size=1)      #publisher for the markers
		self.sub_image = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1)
		self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callback_odom, queue_size=1)
		self.pub_odom = rospy.Publisher("/mcpf_gps",Odometry, queue_size=50)  
		self.pub_pcloud.publish(mcpf.particles)	  
		self.pub_field.publish(mcpf.field)	
	
	def callback_odom(self,data):
		mcpf.odom_updates += 1
		#marker = new_marker(-data.pose.pose.position.x+4,data.pose.pose.position.y+3,'r',0.15)
		#marker.color.b = 1.0
		#marker.color.g = 1.0
		#marker.id = len(mcpf.field.markers)+1		
		#mcpf.field.markers.append(marker)
		
		x = -data.pose.pose.position.x+4
		y = data.pose.pose.position.y+3
		z = data.pose.pose.orientation.z
		
		mcpf.x.append(x)
		mcpf.y.append(y)
		mcpf.z.append(z)

		if len(mcpf.x)>=80: 				#delay odometry data to match with the pictures
			changes = [0,0]
			changes[0] = -(mcpf.x[-80]-mcpf.x[-79]) #change in x-direction
			changes[1] = -(mcpf.y[-80]-mcpf.y[-79]) #change in y-direction
			#marker = new_marker(mcpf.x[-80],mcpf.y[-80],'r',0.15)	#add marker to show the actual 
			#marker.color.b = 1.0																		#position of the car (in white)
			#marker.color.g = 1.0																		#from the odometry data
			#marker.id = len(mcpf.field.markers)+1		
			#mcpf.field.markers.append(marker)

			#print "move particles"
			mcpf.particles = move_particles(mcpf.particles, changes, mcpf.z[-79])
			mcpf.particles_first_moved = 1
		
		self.pub_pcloud.publish(mcpf.particles)
		self.pub_field.publish(mcpf.field)	

	def callback(self, data):
		print "picture after",mcpf.odom_updates,"odometry updates"
		mcpf.odom_updates=0
		img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		xy = self.detector.calculate_best_position(img)			

    # Don't publish a pose if location can't be reliably determined
		if xy is None:
			print("No location found")
			return
		#marker = new_marker(xy[0],xy[1],'r',0.15)			#add marker to show the actual 
		#marker.color.b = 1.0														#position of the car (in white) 
		#marker.color.g = 1.0														#calculated from the picture
		#marker.id = len(mcpf.field.markers)+1		
		#mcpf.field.markers.append(marker)


		#angles between car and balloons
		cb_angles,cb_colors = self.detector.calculate_angles2balloons()	

		#angles between particles and balloons 
		pb_angles = calc_angles_particles2balloons(self.detector,mcpf.particles,img) 

		#calculate weights
		weights = calc_weights(pb_angles,cb_angles,cb_colors)
		
		#generate a new particle set
		if mcpf.particles_first_moved == 1:
			mcpf.particles = resample_particles(mcpf.particles, weights)		

		x,y,z = calc_position(mcpf.particles)

		print "Position",x,y,z		
		print np.arcsin(z)
		quaternion = Quaternion(0, 0, z, np.cos(np.arcsin(z)))
		#create the odometry message
		odom_gps = Odometry()
		odom_gps.header.stamp = rospy.Time.now()
		odom_gps.header.frame_id = "odom"
		odom_gps.pose.pose = Pose(Point(x,y,0),quaternion)
		odom_gps.child_frame_id = "base_link"
		odom_gps.twist.twist = Twist(Vector3(0.1, -0.1, 0), Vector3(0, 0, 0.1))
		self.pub_odom.publish(odom_gps)

def main(args):
	rospy.init_node('mcpf', anonymous=True)
	m = mcpf()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)




