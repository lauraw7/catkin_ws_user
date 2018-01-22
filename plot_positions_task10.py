#!/usr/bin/env python

## RO-10-Germanidis-Witt

import roslib
import sys
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt

import path_parser

from nav_msgs.msg import Odometry

class gps:
	saved_x=[]	
	saved_y=[]
	cnt = 0
	
	def __init__(self):
		self.gps_sub = rospy.Subscriber("/odom_gps", Odometry, self.callback, queue_size=1)  #get the odometry	
		xy = np.array(list(path_parser.read_points('sample_map_origin_map.txt')))			#read in the points of the map
		gps.x, gps.y = xy.T	


	def callback(self,data):
		gps.saved_x.append(data.pose.pose.position.x)
		gps.saved_y.append(data.pose.pose.position.y)
		gps.cnt += 1
		print gps.cnt
		
		if gps.cnt == 1200:
			plt.plot(gps.saved_x[1:],gps.saved_y[1:],'r')
			plt.plot(gps.x,gps.y,'g')
			plt.show(block=True)

def main(args):
	rospy.init_node('plot_gps', anonymous=True)
	ic = gps()
	rospy.spin()


if __name__ == '__main__':
	main(sys.argv)




