#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib

def calc_kreisbogen(pos1,pos2,radius,RL,start_end):
    start_x = pos1[0][0]
    start_y = pos1[0][1]
    end_x = pos2[0][0]
    end_y = pos2[0][1]
    d = np.sqrt((start_x-end_x)**2+(start_y-end_y)**2) #Laenge der Kreissehne
    alpha = 2*np.arcsin(d/(2*radius))
    if start_end == 'start':
        start_yaw = pos1[0][2]
        if RL == 'L' and alpha !=0 and start_yaw<=np.pi:
            alpha = 2*np.pi-alpha  
        elif RL == 'R' and alpha !=0 and start_yaw>=np.pi:
            alpha = 2*np.pi-alpha
    elif start_end == 'end': 
        start_yaw = pos1[0][2]
        if RL == 'R' and alpha !=0:
            alpha = alpha 
        elif RL == 'L' and alpha!=0:
            alpha = 2*np.pi-alpha
    elif start_end == 'mid':
        alpha = alpha
    bogen = radius*alpha  
    return bogen,alpha

def find_intersections(Mx1,My1,r1,Mx2,My2,r2):	#find all intersections from two given circles
    intersections = []
    c = np.sqrt((Mx2-Mx1)**2+(My2-My1)**2) 
    x = (r1**2+c**2-r2**2)/(2*c)
    y = np.sqrt(r1**2-x**2)
    S1x = Mx1+x*(Mx2-Mx1)/c-y*(My2-My1)/c
    S1y = My1+x*(My2-My1)/c+y*(Mx2-Mx1)/c
    S2x = Mx1+x*(Mx2-Mx1)/c+y*(My2-My1)/c
    S2y = My1+x*(My2-My1)/c-y*(Mx2-Mx1)/c
    if np.isnan(S1x) == 0:
        intersections.append([S1x,S1y])
    if np.isnan(S2x) == 0:
        intersections.append([S2x,S2y])
    return intersections

def calc_gerade(xcs,ycs,xcg,ycg,radius,CSC):
		#calculate the line between the start and the final circle
    if CSC[0]==CSC[2] and CSC[0]=='L':
        m = (ycg-ycs)/(xcg-xcs)
        b = ycs-m*xcs-radius
    elif CSC[0]==CSC[2] and CSC[0]=='R':
        m = (ycg-ycs)/(xcg-xcs)
        b = ycs-m*xcs+radius
    elif CSC[0]=='R':
        m = (ycg+radius-(ycs+radius))/(xcg-xcs)
        b = ycs-m*xcs+radius
    else:
        m = (ycg+radius-(ycs-radius))/(xcg-xcs)
        b = ycs-m*xcs-radius
    return m,b

def calc_intersections(xcs,ycs,xcg,ycg,radius,start_end,CSC):
    m,b=calc_gerade(xcs,ycs,xcg,ycg,radius,CSC)
    if start_end == 'start':
        d = b-(-m)*xcs-ycs
        y1=ycs+(d-(-m)*np.sqrt(radius**2*((-m)**2+1**2)-d**2))/((-m)**2+1**2)
        y2=ycs+(d+(-m)*np.sqrt(radius**2*((-m)**2+1**2)-d**2))/((-m)**2+1**2)
        x1=xcs+(-m*d+1*np.sqrt(radius**2*((-m)**2+1**2)-d**2))/((-m)**2+1**2)
        x2=xcs+(-m*d-1*np.sqrt(radius**2*((-m)**2+1**2)-d**2))/((-m)**2+1**2)
    elif start_end == 'end':
        d = b-(-m)*xcg-ycg
        y1=ycg+(d-(-m)*np.sqrt(radius**2*((-m)**2+1**2)-d**2))/((-m)**2+1**2)
        y2=ycg+(d+(-m)*np.sqrt(radius**2*((-m)**2+1**2)-d**2))/((-m)**2+1**2)
        x1=xcg+(-m*d+1*np.sqrt(radius**2*((-m)**2+1**2)-d**2))/((-m)**2+1**2)
        x2=xcg+(-m*d-1*np.sqrt(radius**2*((-m)**2+1**2)-d**2))/((-m)**2+1**2)
    return x1,y1,x2,y2

def calc_middle(x1,y1,x2,y2,Mx,My,r,RL):
		#calculate the middle of two intersection points
    x = max([x1,x2])-(max([x1,x2])-min([x1,x2]))/2
    x_= abs(Mx-x)
    if RL == 'L':
        y=My-np.sqrt(r**2-x_**2)   
    else:
        y=My+np.sqrt(r**2-x_**2)   
    return x,y











def main():
	step_length = 2
	start = [(0,0)]
	points = [(3,4),(2,1),(5,4),(2,2),(8,1)]  
	vertices,edges = create_RTT(5,points,[(0,0)],step_length)
	plot_RTT(vertices,edges) 

if __name__ == '__main__':
	main()
