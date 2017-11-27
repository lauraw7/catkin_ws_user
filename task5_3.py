#!/usr/bin/env python

## RO-05-Germanidis-Witt

import roslib
import sys
import rospy
import numpy as np
import math

from std_msgs.msg import Int16		# import datatype Int16
from sensor_msgs.msg import LaserScan

class wall_control:
	counter = 0
	last_error = 0
	init1 = 1
	error = 0

	def __init__(self):
		self.pub_velocity = rospy.Publisher('/manual_control/speed', Int16, queue_size=10)		
		self.pub_steering = rospy.Publisher('/manual_control/steering', Int16, queue_size=10)
		self.sub_scanner = rospy.Subscriber("scan", LaserScan, self.scanCallback, queue_size=100)		 	
       
	def scanCallback(self,scan_msg):
		if wall_control.init1 == 1:
			wall_control.error = 0
			wall_control.init1=0
			wall_control.desired_theta = 0		
		#240 entspricht -60 und 300 entspricht -120
  		dl2 = scan_msg.ranges[int(abs(scan_msg.angle_min+(120*3.1415/180)/scan_msg.angle_increment))]  #distance to the wall at 240 degrees
  		dr2 = scan_msg.ranges[int(abs(scan_msg.angle_min+(60*3.1415/180)/scan_msg.angle_increment))]   #distance to the wall at 300 degrees
		c = math.sqrt(dl2**2+dr2**2-2*dl2*dr2*math.cos(60))
		phi2 = math.asin(dr2*math.sin(60)/c)
		d02 = math.sin(phi2)*dl2
		thetal2 = math.acos(d02/dl2)
		theta02 = thetal2 - 30
		current_theta = theta02		#calculated current theta
		cy = d02 + math.sin(theta02*l)
		distance_to_wall = 0.4
		lookahead = 0.5
		wall_control.desired_theta = math.atan((distance_to_wall - cy)/lookahead)	#calculated desired theta

		frequency = 0.01
		Kp = 3		#parameter for the p-part of the controller
		Kd = 0.9	#parameter for the d-part of the controller
		wall_control.last_error = wall_control.error
		wall_control.error = wall_control.desired_theta-current_theta
		steering_angle = Kp*(wall_control.error)+Kd*(wall_control.error-wall_control.last_error)*frequency + 115
		self.pub_steering.publish(steering_angle) #publish the steering angle calculated with the PD-controller
		rospy.loginfo("desired theta: %s, current theta: %s, steering angle: %s", wall_control.desired_theta,current_theta,steering_angle)
		wall_control.counter += 1
		if wall_control.counter < 1000:
			self.pub_velocity.publish(300) #move the car
		elif wall_control.counter == 1000:
			self.pub.velocity.publish(0)  # stop the car


def main(args):
	rospy.init_node('wall_control', anonymous=True)
	wc = wall_control()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")


if __name__ == '__main__':
   main(sys.argv)

