#!/usr/bin/env python

#RO-10-Germanidis-Witt

import numpy as np

import path_parser

import matplotlib.pyplot as plt

map_size_x=600 #cm
map_size_y=400 #cm
resolution = 10 #cm
matrix = np.zeros( (map_size_x/resolution,map_size_y/resolution,2),dtype='f' )

def main(map_file):
	xy = np.array(list(path_parser.read_points(map_file)))			#read in the points of the map
	x, y = xy.T																									#in m

	fig = plt.figure(figsize=(12, 10), facecolor='w')						#plot the map
	plt.plot(x, y, ':o', markersize=2)
	
	global matrix
	matrix = np.load('matrixDynamic_lane1.npy')									#load the potential field maps

	plt.gca().set_aspect(1, 'datalim')  # keep circles as circles
	plt.tight_layout()

	def show_nearest(target):				#shows the nearest point on the lane

		car_length=0.3

		global matrix
		yaw, x, y = target
		print '_________________________________________________________'
		print 'yaw in deg:',np.rad2deg(yaw),'		position in m: (',x,',',y,')'
		x_index=np.int(x*100/resolution) #convert m in cm and divide by resolution  
		y_index=np.int(y*100/resolution)
		print ' '
		x3, y3 = matrix[x_index,y_index,:]
		print 'potential field values at [',x_index,',',y_index,']:',x3,y3
		f_x=np.cos(yaw)*x3 + np.sin(yaw)*y3
		f_y=-np.sin(yaw)*x3 + np.cos(yaw)*y3

		Kp=4
		steering=Kp*np.arctan(f_y/(2.5*f_x))

		if (steering>np.deg2rad(45)): #limitation 45 degrees
			steering = (np.pi)/4

		if (steering<-np.deg2rad(45)):
			steering = -(np.pi)/4

		if (f_x>0):	#nearest point is in front of the car
			speed = -100
		else:				#nearest point is behind the car
			speed = 100
			if (f_y>0): #lane is on the right side of the car
				steering = -steering #invert the steering angle

		print 'steering in deg:',np.rad2deg(steering)

		r = car_length * np.abs(np.tan((np.pi)/2-steering))

		if (r>10):
			r = 10
		print 'radius:',r
		if (steering<0.0):
			r=-r
		xc = x - np.sin(yaw) * r
		yc = y + np.cos(yaw) * r

		ax = plt.axes()
		ax.arrow(x, y, car_length*np.cos(yaw), car_length*np.sin(yaw), width=car_length, head_width=car_length, head_length=0.09, fc='b', ec='b') #car

		ax = plt.axes()
		ax.arrow(x, y, f_x*np.cos(yaw), f_x*np.sin(yaw), head_width=0.01, head_length=0.01, fc='r', ec='r') #x-axis of the car

		ax = plt.axes()
		ax.arrow(x, y, -f_y*np.sin(yaw), f_y*np.cos(yaw), head_width=0.01, head_length=0.01, fc='r', ec='r')#y-axis of the car

		ax = plt.axes()
		ax.arrow(x, y, x3, y3, head_width=0.01, head_length=0.01, fc='g', ec='g')
		

		plt.scatter(*(x,y) ,color='r')  #middlepoint on the rear axle of the car
		plt.scatter(*(x + x3, y + y3), color='g')			#nearest point 
		circ = plt.Circle((xc, yc), r, color='r', fill=False)
		plt.gcf().gca().add_artist(circ)
		plt.show(block=False)

	#plot the turning circles for 3 different positions
	show_nearest((0,2,1)) #yaw (rad),x,y(m)
	show_nearest((np.pi/2,5.4,2))
	show_nearest((np.pi/1.25,3,3))

	plt.show(block=True)
 

if __name__ == '__main__':
	main('sample_map_origin_map.txt')
