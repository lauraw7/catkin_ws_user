#!/usr/bin/env python

## RO-05-Germanidis-Witt

import roslib
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Int16		        # import datatype Int16
from std_msgs.msg import Float32		# import datatype Float32

def calc_av_sq_error(heading_angle_saved,desired_heading_angle): 
  sum_sq = 0
  for i in range(0,len(heading_angle_saved)):
      sum_sq += (heading_angle_saved[i]-desired_heading_angle)**2 #sum up all squared errors
  return sum_sq/len(heading_angle_saved)			  #divide by number of saved differences to get the mean value


class straight_line_control:
  counter = 0
  heading_angle_saved = []
  desired_heading_angle = []

  def __init__(self):
    self.pub_velocity = rospy.Publisher("/manual_control/speed", Int16, queue_size=10)		
    self.pub_steering = rospy.Publisher("/manual_control/steering", Int16, queue_size=10)
    self.yaw_sub = rospy.Subscriber("/model_car/yaw",Float32,self.callback, queue_size=1)  				

  def callback(self,yaw):
    if not(straight_line_control.desired_heading_angle):
       straight_line_control.desired_heading_angle=yaw.data-10 #start at minus 10 degrees (requirement from the assginment)
    current_heading_angle = yaw.data
    straight_line_control.heading_angle_saved.append(current_heading_angle) #save the heading angles
    calibratedZeroSteeringAngle = 115
    Kp = 3	
    steering_angle = Kp*(straight_line_control.desired_heading_angle-current_heading_angle) + calibratedZeroSteeringAngle 
    self.pub_steering.publish(steering_angle)
    #rospy.loginfo("desired heading: %s, current heading: %s, steering angle: %s",   straight_line_control.desired_heading_angle,current_heading_angle,steering_angle)
    straight_line_control.counter += 1
    if straight_line_control.counter < 400:
       self.pub_velocity.publish(300) #move the car
    elif straight_line_control.counter == 400:
       self.pub_velocity.publish(0) #stop the car
       rospy.loginfo("av.sq. e: %s", calc_av_sq_error(straight_line_control.heading_angle_saved,straight_line_control.desired_heading_angle))

def main(args):
  rospy.init_node('straight_line_control', anonymous=True)
  slc = straight_line_control()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
   main(sys.argv)




