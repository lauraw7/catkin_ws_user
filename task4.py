#!/usr/bin/env python

#RO-04-Germanidis-Witt

# --- imports ---
import rospy
import sys
import roslib 
import math
from std_msgs.msg import Int16		
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

class data_scanner:
  scan_ = LaserScan
  def __init__(self):          #initialize subscriber 
    self.sub_scanner = rospy.Subscriber("scan", LaserScan, self.scanCallback, queue_size=100)			 	
       
  def scanCallback(self,scan_msg):
     self.scan_ = scan_msg     #save actual scan message


def main(args):
  rospy.init_node('data_scanner', anonymous=True)
  pub_velocity = rospy.Publisher('/manual_control/speed', Int16, queue_size=10)		#initialize publisher 
  pub_steering = rospy.Publisher('/manual_control/steering', Int16, queue_size=10)    
  scanner_ = data_scanner()
  #measurement1
  scan1 = scanner_.scan_	
  
  #move the car
  angle = 90		#0,30,60,90,120,150,179
  speed = -200
        
  pub_steering.publish(angle)
  counter = 0
  while counter < 10:
    counter += 1
    pub_velocity.publish(speed) #drive backwards
  pub_velocity.publish(0)       #stop
  
  #measurement2
  scan2 = scanner_.scan_

  #calculate the angle
  alpha = 10 #in degrees  alpha*3.1415/180 in rad
  #Sketch1
        
  dl1 = scan1.ranges[int(abs((scan1.angle_min+alpha*3.1415/180)/scan1.angle_increment))]
  dr1 = scan1.ranges[int(abs((scan1.angle_min-alpha*3.1415/180)/scan1.angle_increment))]
  rospy.loginfo("dl1: %s   dr1: %s", dl1, dr1)
  d01 = math.cos(alpha)*dl1
  #Sketch2
  dl2 = scan2.ranges[int(abs((scan2.angle_min+alpha*3.1415/180)/scan2.angle_increment))]
  dr2 = scan2.ranges[int(abs((scan2.angle_min-alpha*3.1415/180)/scan2.angle_increment))]
  c = math.sqrt(dl2**2+dr2**2-2*dl2*dr2*math.cos(2*alpha))
  phi2 = math.asin(dr2*math.sin(2*alpha)/c)
  d02 = math.sin(phi2)*dl2
  thetal2 = math.acos(d02/dl2)
  theta02 = thetal2 - alpha
  #Sketch3
  R = (d02-d01)/math.sin(theta02)
  #Sketch4
  l = 0.255 
  r = R 
  
  phi = math.asin(l/r)
  #rospy.loginfo('Phi: %s',phi)
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
   main(sys.argv)




