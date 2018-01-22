#!/usr/bin/env python

## RO-10-Germanidis-Witt

import roslib
import sys
import rospy
import numpy as np
import path_parser
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16	

from tf.transformations import euler_from_quaternion, quaternion_from_euler

map_size_x=600 #cm
map_size_y=400 #cm
resolution = 10 #cm
matrix = np.zeros( (map_size_x/resolution,map_size_y/resolution,2),dtype='f' )

def main():

	pub_velocity = rospy.Publisher("/manual_control/speed", Int16, queue_size=10)
	pub_steering = rospy.Publisher("/manual_control/steering", Int16, queue_size=10)

	global matrix
	matrix = np.load('matrixDynamic_lane1.npy')									#load the potential field maps

	def calc_steering(target):				#searches the nearest point on the lane and calculate the steering angle
		
		car_length=0.3

		global matrix
		yaw, x, y = target
		print '______________________________________________________________________________________'
		print 'yaw in deg:',np.rad2deg(yaw),'		position in m: (',x,',',y,')'
		x_index=min(max(0,np.int(x*100/resolution)),(map_size_x/resolution-1)) #convert m in cm, divide by resolution  
		y_index=min(max(0,np.int(y*100/resolution)),(map_size_y/resolution-1)) #and limit the range
		print ' '
		x3, y3 = matrix[x_index,y_index,:]
		print 'potential field values at [',x_index,',',y_index,']:',x3,y3
		f_x=np.cos(yaw)*x3 + np.sin(yaw)*y3
		f_y=-np.sin(yaw)*x3 + np.cos(yaw)*y3

		Kp=4
		steering=-Kp*np.arctan(f_y/(2.5*f_x))		#calculate the steering angle

		if (steering>np.deg2rad(45)):  #limit at 45 degrees
			steering = (np.pi)/4

		if (steering<-np.deg2rad(45)): #limit at -45 degrees
			steering = -(np.pi)/4

		if (f_x>0):			#nearest point is in front of the car
			speed = -150	
		else:						#nearest point is behind the car
			speed = 150		

			if (f_y>0): #lane is on the right side of the car
				steering = -steering #invert the steering angle

		print 'steering in deg:',np.rad2deg(steering)
		pub_velocity.publish(Int16(speed))
		steering_car=steering*(180/np.pi)*2+90				#between 0 and 180 instead of -45 and 45
		print 'steering_car:',steering_car
		pub_steering.publish(Int16(steering_car))


	def callback_odom(data):
		x = data.pose.pose.position.x								#get position
		y = data.pose.pose.position.y
		orientation_q = data.pose.pose.orientation	#get orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		calc_steering((yaw,x,y))	#calculate the steering angle

	sub_odom = rospy.Subscriber("/odom_gps", Odometry, callback_odom, queue_size=10)   	#odometry from the car

if __name__ == '__main__':
	rospy.init_node('nav')
	main()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")



