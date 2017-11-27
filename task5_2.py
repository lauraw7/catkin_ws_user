#!/usr/bin/env python

## RO-05-Germanidis-Witt

import roslib
import sys
import rospy
import numpy as np

from std_msgs.msg import Int16		# import datatype Int16
from nav_msgs.msg import Odometry

class trajectory_control:
	counter = 0
	last_error = 0
	init1 = 1
	error = 0

	def __init__(self):
		self.pub_velocity = rospy.Publisher('/manual_control/speed', Int16, queue_size=10)		
		self.pub_steering = rospy.Publisher('/manual_control/steering', Int16, queue_size=10)
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback, queue_size=1)  #get the odometry		

	def callback(self,d):
		if trajectory_control.init1 == 1: 	#Initialisation
			trajectory_control.desired_y = d.pose.pose.position.y + 0.2  	
			trajectory_control.init1 = 0		
		current_y = d.pose.pose.position.y   #get current y-position
		frequency = 0.01
		Kp = 3
		Kd = 0.9
		trajectory_control.last_error = trajectory_control.error  #save the error for the D-part of the controller
		trajectory_control.error = trajectory_control.desired_y-current_y
		steering_angle = Kp*(trajectory_control.error)+Kd*(trajectory_control.error-trajectory_control.last_error)*frequency + 110
		self.pub_steering.publish(steering_angle) #publish the calculated angle from the PD-controller
		rospy.loginfo("desired y: %s, current y: %s, steering angle: %s", trajectory_control.desired_y,current_y,steering_angle)
		trajectory_control.counter += 1
		print(trajectory_control.counter)
		if trajectory_control.counter < 1000:
			self.pub_velocity.publish(-100) #the used car drives backwards with a positiv velocity #move the car
		elif trajectory_control.counter == 1000:
			self.pub.velocity.publish(0)	#stop the car 


def main(args):
	rospy.init_node('trajectory_control', anonymous=True)
	tc = trajectory_control()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")


if __name__ == '__main__':
   main(sys.argv)

