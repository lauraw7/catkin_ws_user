#!/usr/bin/env python

#RO-04-Germanidis-Witt

# --- imports ---
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from math import *

# --- definitions ---
def resetGrid():
    global occupancy_grid
    
    # set all values to "FREE"
    occupancy_grid.data = np.zeros(shape=(occupancy_grid.info.height*occupancy_grid.info.width,1))


# to a given cartesian x,y coordinate, mark the corresponding cell in the grid as "OCCUPIED"(100) or "UNKNOWN"(-1)
def setCell(x,y,state):
    global occupancy_grid

    res = occupancy_grid.info.resolution
    x_scaled = (x * 1.0 / res) + occupancy_grid.info.width/2
    y_scaled = (y * 1.0 / res) + occupancy_grid.info.height/2
    if x_scaled >= occupancy_grid.info.width or x_scaled < 0 or y_scaled >= occupancy_grid.info.height or y_scaled < 0:
        return

    offset = (int(round(x_scaled)) - 1) * occupancy_grid.info.height
    occupancy_grid.data[int(offset) + int(round(y_scaled) - 1)] = state

def scanCallback(scan_msg):
    global occupancy_grid
    resetGrid()
    #convert scan measurements into an occupancy grid  
    for i in range(0,len(scan_msg.ranges)):
       if scan_msg.ranges[i] < scan_msg.range_max:	#obstacle detected
	   distance = scan_msg.ranges[i] 
           angle = scan_msg.angle_min+i*scan_msg.angle_increment 
           #rospy.loginfo("d = %s   Winkel = %s", distance, angle)   
	   y=cos(angle)*distance
           x=sin(angle)*distance
           setCell(x,y,100)    #mark as "occupied"
           d = distance+occupancy_grid.info.resolution 
           while d <=scan_msg.range_max:
             y_ = cos(angle)*d
             x_ = sin(angle)*d
             setCell(x_,y_,-1) #mark as unknown
	     d += occupancy_grid.info.resolution 
    pub_grid.publish(occupancy_grid)

# --- main ---
rospy.init_node("scan_grid")

# init occupancy grid
occupancy_grid = OccupancyGrid()
occupancy_grid.header.frame_id = "laser"
occupancy_grid.info.resolution = 0.1 #None # in m/cell      #8m in jede Richtung?

# width x height cells
occupancy_grid.info.width = 40 #None
occupancy_grid.info.height = 40 #None

# origin is shifted at half of cell size * resolution
occupancy_grid.info.origin.position.x = int(-1.0 * occupancy_grid.info.width / 2.0) * occupancy_grid.info.resolution
occupancy_grid.info.origin.position.y = int(-1.0 * occupancy_grid.info.height / 2.0) * occupancy_grid.info.resolution
occupancy_grid.info.origin.position.z = 0
occupancy_grid.info.origin.orientation.x = 0
occupancy_grid.info.origin.orientation.y = 0
occupancy_grid.info.origin.orientation.z = 0
occupancy_grid.info.origin.orientation.w = 1

rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=100)

pub_grid = rospy.Publisher("scan_grid", OccupancyGrid, queue_size=100)

rospy.spin()
