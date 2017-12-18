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
	init = 1
	delta_t = 0.1		#time step size
	x_t1 = 0			#x at t-1
	x_t2 = 0			#x at t-2
	y_t1 = 0			#analog
	y_t2 = 0
	theta_t1 = 0
	theta_t2 = 0
	
	def __init__(self):
		self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callback_odom, queue_size=1)    #odometry from the car
		self.sub_vgps= rospy.Subscriber("/odom_gps", Odometry, self.callback_vgps, queue_size=1) #odometry from visual gps
		self.pub_odom = rospy.Publisher("/odom_gps2",Odometry, queue_size=50)      						

	def callback_odom(self,data): 		#prediction
		kalman.predicted += 1  			#counter for the predictionsteps 
		if kalman.init == 1:
			kalman.init = 0
			x_t = data.pose.pose.position.x + 0.6			#values have to be modified because car odometry and 
			y_t = data.pose.pose.position.y + 3.6			#visual gps odometry have diffrent origins
			theta_t = np.arccos(data.pose.pose.orientation.w)*2*np.sign(data.pose.pose.orientation.z)
		else:
			v = (kalman.x_t2-kalman.x_t1)*kalman.delta_t
			theta = np.arccos(data.pose.pose.orientation.w)*2*np.sign(data.pose.pose.orientation.z)
			delta_x = v*np.cos(theta)*kalman.delta_t
			delta_y = v*np.sin(theta)*kalman.delta_t
			delta_theta = kalman.theta_t2-kalman.theta_t1	
			kalman.x_t_exp = kalman.x_t1 + delta_x
			kalman.y_t_exp = kalman.y_t1 + delta_y
			kalman.theta_t_exp = kalman.theta_t1 + delta_theta

		kalman.x_t2 = kalman.x_t1
		kalman.x_t1 = data.pose.pose.position.x + 0.6
		kalman.y_t2 = kalman.y_t1
		kalman.y_t1 = data.pose.pose.position.y + 3.6
		kalman.theta_t2 = kalman.theta_t1
		kalman.theta_t1 = np.arccos(data.pose.pose.orientation.w)*2*np.sign(data.pose.pose.orientation.z)

	def callback_vgps(self,data):		#update
		print 'UPDATE after ', kalman.predicted, 'predictions'
		kalman.predicted = 0
		kx = 0.3
		ky = 0.3
		kt = 0.5
		x_new = kx * data.pose.pose.position.x + (1-kx)*kalman.x_t_exp
		y_new = ky * data.pose.pose.position.y + (1-ky)*kalman.y_t_exp
		theta_new = kt * (np.arccos(data.pose.pose.orientation.w)*2*np.sign(data.pose.pose.orientation.z))+ (1-kt)*kalman.theta_t_exp
		print 'k*gps+(1-k)*prediction'
		print 'k*',data.pose.pose.position.x,'+(1-k)*',kalman.x_t_exp,'=',x_new
		print 'k*',data.pose.pose.position.y,'+(1-k)*',kalman.y_t_exp,'=',y_new
		print 'new theta:',theta_new

		quaternion = yaw_to_quaternion(theta_new)		#create a quaternion from the angle
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




