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
	saved_ox= [None]*1000	
	saved_oy= [None]*1000
	saved_ot= [None]*1000
	saved_kx = [None]*1000
	saved_kt = [None]*1000
	saved_ky = [None]*1000
	saved_t=[]
	cnt = 0
	
	def __init__(self):
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback, queue_size=1)  #get the odometry		
		self.odom2_sub = rospy.Subscriber("/odom_gps", Odometry, self.callback2, queue_size=1)  #get the odometry		
		self.odom2_sub = rospy.Subscriber("/odom_gps2", Odometry, self.callback3, queue_size=1)  #get the odometry		
	def callback(self,data):
		gps.saved_x.append(data.pose.pose.position.x + 0.6)
		gps.saved_y.append(data.pose.pose.position.y + 3.6)
		gps.saved_t.append(np.arccos(data.pose.pose.orientation.w)*2*np.sign(data.pose.pose.orientation.z))
		gps.cnt += 1
		print gps.cnt
		if gps.cnt == 1001:
			plt.plot(gps.saved_x[1:],gps.saved_y[1:],'b')
			plt.show()
			#plt.plot(gps.saved_ox[1:],gps.saved_oy[1:],'r')
			#plt.plot(gps.saved_kx[1:],gps.saved_ky[1:],'g')
			plt.plot(gps.saved_x[1:],'b')
			plt.plot(gps.saved_ox[1:],'or')
			plt.plot(gps.saved_kx[1:],'og')	
			plt.show()
			plt.plot(gps.saved_y[1:],'b')
			plt.plot(gps.saved_oy[1:],'or')
			plt.plot(gps.saved_ky[1:],'og')	
			plt.show()
			plt.plot(gps.saved_t[1:],'b')
			plt.plot(gps.saved_ot[1:],'or')
			plt.plot(gps.saved_kt[1:],'og')	
			plt.show()
			for i in range(len(gps.saved_kx)):
				if gps.saved_kx[i]!=None:
					s_x.append(gps.saved_kx[i])
					s_y.append(gps.saved_ky[i])
					s_t.append(gps.saved_kt[i])
			plt.plot(s_x[1:],s_y[1:])
			plt.show()
			plt.plot(s_t[1:])
			plt.show()

	def callback2(self,data):
		#gps.saved_ox.append(data.pose.pose.position.x)
		gps.saved_ox[gps.cnt]=data.pose.pose.position.x
		gps.saved_oy[gps.cnt]=data.pose.pose.position.y
		gps.saved_ot[gps.cnt]=np.arccos(data.pose.pose.orientation.w)*2*np.sign(data.pose.pose.orientation.z)		

	def callback3(self,data):
		gps.saved_kx[gps.cnt]=data.pose.pose.position.x
		gps.saved_ky[gps.cnt]=data.pose.pose.position.y
		gps.saved_kt[gps.cnt]=np.arccos(data.pose.pose.orientation.w)*2*np.sign(data.pose.pose.orientation.z)	

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




