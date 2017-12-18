#!/usr/bin/env python

## RO-08-Germanidis-Witt

import roslib
import sys
import rospy
import math
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def yaw_to_quaternion(yaw):
#convert a yaw angle (in radians) into a Quaternion message
	return Quaternion(0, 0, math.sin(yaw / 2), math.cos(yaw / 2))

class kalman:
	predicted = 0
	#Parameters for the Kalman filter for x
	Q_x = 0.001**2			
	R_x = 0.001**2
	x_t1 = 0				#x at t-1
	x_P_t1 = 3**2		#P at t-1
	x_P_t_exp = 0		#P at t, predicted
	x_K_t = 0				#Kalman constant K at t
	x_u_t1 = 0  #action

	#Parameters for the Kalman filter for y
	Q_y = 0.001**2
	R_y = 0.001**2
	y_t1 = 0
	y_P_t1 = 3**2
	y_P_t_exp = 0
	y_K_t = 0
	y_u_t1 = 0

	#Parameters for the Kalman filter for Theta
	Q_theta = 0.001**2
	R_theta = 0.001**2
	theta_t1 = 0
	theta_P_t1 = (np.pi/2)**2
	theta_P_t_exp = 0
	theta_K_t = 0
	theta_u_t1 = 0

	H = 1
	
	def __init__(self):
		self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callback_odom, queue_size=1)   	#odometry from the car
		self.sub_vgps= rospy.Subscriber("/odom_gps", Odometry, self.callback_vgps, queue_size=1)  #odometry from the visual gps
		self.pub_odom = rospy.Publisher("/odom_gps2",Odometry, queue_size=50)      						

	def callback_odom(self,data): 		#prediction
		kalman.predicted += 1
		#filter 1 - x
		kalman.x_t_exp=kalman.x_t1 + kalman.x_u_t1					#formulas described in the slides
		kalman.x_P_t_exp = kalman.x_P_t1 + kalman.Q_x
		kalman.x_S_t = kalman.R_x+kalman.H*kalman.x_P_t_exp*kalman.H
		kalman.x_K_t = kalman.x_P_t1*kalman.H*kalman.x_S_t**(-1)
		
		#filter 2 - y
		kalman.y_t_exp=kalman.y_t1 + kalman.y_u_t1
		kalman.y_P_t_exp = kalman.y_P_t1 + kalman.Q_y
		kalman.y_S_t = kalman.R_y+kalman.H*kalman.y_P_t_exp*kalman.H
		kalman.y_K_t = kalman.y_P_t1*kalman.H*kalman.y_S_t**(-1)
	
		#filter 3 - theta
		kalman.theta_t_exp = kalman.theta_t1 + kalman.theta_u_t1
		kalman.theta_P_t_exp = kalman.theta_P_t1 + kalman.Q_theta
		kalman.theta_S_t = kalman.R_theta+kalman.H*kalman.theta_P_t_exp*kalman.H
		kalman.theta_K_t = kalman.theta_P_t1*kalman.H*kalman.theta_S_t**(-1)
	
	def callback_vgps(self,data):		#update
		print 'UPDATE after ', kalman.predicted, 'Predictions'
		kalman.predicted = 0

		x_new =  kalman.x_t_exp + kalman.x_K_t*(data.pose.pose.position.x+0.6-kalman.x_t_exp)
		kalman.x_t1 = x_new			
		kalman.x_P_t1 = kalman.x_P_t_exp-kalman.x_K_t*kalman.x_S_t*kalman.x_K_t		

		y_new =  kalman.y_t_exp + kalman.y_K_t*(data.pose.pose.position.y+3.6-kalman.y_t_exp)
		kalman.y_t1 = y_new			
		kalman.y_P_t1 = kalman.y_P_t_exp-kalman.y_K_t*kalman.y_S_t*kalman.y_K_t		
		
		theta = np.arccos(data.pose.pose.orientation.w)*2*np.sign(data.pose.pose.orientation.z)
		theta_new =  kalman.theta_t_exp + kalman.theta_K_t*(theta-kalman.theta_t_exp)
		kalman.theta_t1 = theta_new			
		kalman.theta_P_t1 = kalman.theta_P_t_exp-kalman.theta_K_t*kalman.theta_S_t*kalman.theta_K_t		

		print 'Ks: x:',kalman.x_K_t,'y:',kalman.y_K_t,'theta:',kalman.theta_K_t
		print 'new position: ', x_new, y_new
		print 'new theta:',theta_new

		quaternion = yaw_to_quaternion(theta_new)
		#create the odometry message
		odom_gps = Odometry()
		odom_gps.header.stamp = rospy.Time.now()
		odom_gps.header.frame_id = "odom"
		odom_gps.pose.pose = Pose(Point(x_new,y_new,0),quaternion)
		odom_gps.child_frame_id = "base_link"
		odom_gps.twist.twist = Twist(Vector3(0.1, -0.1, 0), Vector3(0, 0, 0.1))
		self.pub_odom.publish(odom_gps)		

def main(args):
	rospy.init_node('kalman', anonymous=True)
	rospy.Rate(10)
	ic = kalman()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)




