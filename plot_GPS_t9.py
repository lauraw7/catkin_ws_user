#!/usr/bin/env python

## RO-03-Germanidis-Witt

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


class gps:
	saved_x=[]	
	saved_y=[]
	saved_yaw=[]
	saved_mcpfx= [None]*15000	
	saved_mcpfy= [None]*15000
	saved_mcpfyaw= [None]*15000
	cnt = 0
	
	def __init__(self):
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback, queue_size=1)  #get the odometry		
		self.mcpf_sub = rospy.Subscriber("/mcpf_gps", Odometry, self.callback2, queue_size=1)  #get the odometry		

	def callback(self,data):
		gps.saved_x.append(data.pose.pose.position.x + 4)
		gps.saved_y.append(data.pose.pose.position.y + 3)
		gps.saved_yaw.append(np.rad2deg(np.arcsin(data.pose.pose.orientation.z)*2))
		gps.cnt += 1
		print gps.cnt
		if gps.cnt == 1601:
			sx = []
			sy=[]
			syaw=[]
			for i in range(80,gps.cnt):
				sx.append(gps.saved_x[i-80])
				sy.append(gps.saved_y[i-80])
				syaw.append(gps.saved_yaw[i-80])
			plt.plot(sx[1:],sy[1:],'b')
			plt.plot(gps.saved_mcpfx[1:],gps.saved_mcpfy[1:],'or')
			plt.show()
			plt.plot(syaw[1:],'b')
			plt.plot(gps.saved_mcpfyaw[1:],'or')
			plt.show()

	def callback2(self,data):
		#gps.saved_ox.append(data.pose.pose.position.x)
		gps.saved_mcpfx[gps.cnt]=data.pose.pose.position.x
		gps.saved_mcpfy[gps.cnt]=data.pose.pose.position.y
		gps.saved_mcpfyaw[gps.cnt]=np.rad2deg(np.arcsin(data.pose.pose.orientation.z)*2)		



def main(args):
	rospy.init_node('gps', anonymous=True)
	ic = gps()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)




