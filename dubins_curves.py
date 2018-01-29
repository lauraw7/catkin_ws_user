#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib

from dubins_curves_help_functions import calc_intersections, calc_middle, find_intersections, calc_kreisbogen

def calc_CSC(start,goal,radius,CSC):
	plt.gca().set_aspect(1, 'datalim')  # keep circles as circles
	ax = plt.axes()
	ax.scatter(*(12,12),color='w') #fuer bessere Darstellung
	ax.scatter(*(-10,-5),color='w') #fuer bessere Darstellung    
	car_length=0.3
	xs = start[0][0]
	ys = start[0][1]
	yaws = start[0][2]
	xg = goal[0][0]
	yg = goal[0][1]
	yawg = goal[0][2]
		  
	if CSC[0]=='L':
		xcs = xs - np.sin(yaws) * radius
		ycs = ys + np.cos(yaws) * radius
	elif CSC[0]=='R':
		xcs = xs + np.sin(yaws) * radius
		ycs = ys - np.cos(yaws) * radius  
	if CSC[2]=='L':
		xcg = xg - np.sin(yawg) * radius
		ycg = yg + np.cos(yawg) * radius
	elif CSC[2] == 'R':
		xcg = xg + np.sin(yawg) * radius
		ycg = yg - np.cos(yawg) * radius
  
	#Plot the car in start and end position               
	ax.arrow(xs, ys, car_length*np.cos(yaws), car_length*np.sin(yaws), width=car_length, head_width=car_length, head_length=0.09, fc='b', ec='b') 
	ax.arrow(xg, yg, car_length*np.cos(yawg), car_length*np.sin(yawg), width=car_length, head_width=car_length, head_length=0.09, fc='b', ec='b') 

	#Plot turning circle at the start	and at the goal
	circs = plt.Circle((xcs, ycs), radius, color='r', fill=False)
	ax.add_artist(circs) 
	circg = plt.Circle((xcg, ycg), radius, color='r', fill=False)
	ax.add_artist(circg)
    
	#Calculate the line between the two circles
	x1,y1,x3,y3=calc_intersections(xcs,ycs,xcg,ycg,radius,'start',CSC) #both intersections from the line with the first circle
	x1,y1=calc_middle(x1,y1,x3,y3,xcs,ycs,radius,CSC[0])							 #calculate the middle of both
	x2,y2,x4,y4=calc_intersections(xcs,ycs,xcg,ycg,radius,'end',CSC)	 #both intersections from the line with the second circle
	x2,y2=calc_middle(x2,y2,x4,y4,xcg,ycg,radius,CSC[2])							 #calculate the middle of both
  
	ax.plot([x1,x2],[y1,y2], color='g')				#plot the calculated line 
	L_S=np.sqrt((x2-x1)**2+(y2-y1)**2)			
	L_bogen1,alpha1 = calc_kreisbogen(start,[(x1,y1)],radius,CSC[0],'start')
	L_bogen2,alpha2 = calc_kreisbogen(goal,[(x2,y2)],radius,CSC[2],'end')
	 
	#Plot the driven arcs		
	#start cicle
	if CSC[0]=='L': 
		offset1 = np.rad2deg(yaws)-90
	elif CSC[0]=='R':
		offset1=180-np.rad2deg(alpha1)			    
	arc1 = matplotlib.patches.Arc((xcs, ycs), 2*radius, 2*radius,angle=offset1,theta1=0,theta2=np.rad2deg(alpha1), color='g')
	ax.add_artist(arc1)
   #final circle
	if CSC[2]=='L':  
		offset2=np.rad2deg(yawg)-90+360-np.rad2deg(alpha2)
	elif CSC[2]=='R':
		offset2=np.rad2deg(yawg)+90         	    
	arc2 = matplotlib.patches.Arc((xcg, ycg), 2*radius, 2*radius,angle=offset2,theta1=0,theta2=np.rad2deg(alpha2), color='g')
	ax.add_artist(arc2)
    
	plt.show()    
	return L_bogen1,L_S,L_bogen2
 
def calc_CCC(start,goal,radius,CCC):
	plt.gca().set_aspect(1, 'datalim')  # keep circles as circles
	ax = plt.axes()
	ax.scatter(*(12,12),color='w') #fuer bessere Darstellung
	ax.scatter(*(-10,-5),color='w') #fuer bessere Darstellung    
	car_length=0.3
	xs = start[0][0]
	ys = start[0][1]
	yaws = start[0][2]
	xg = goal[0][0]
	yg = goal[0][1]
	yawg = goal[0][2]
		  
	if CCC[0]=='L':
		xcs = xs - np.sin(yaws) * radius
		ycs = ys + np.cos(yaws) * radius
	elif CCC[0]=='R':
		xcs = xs + np.sin(yaws) * radius
		ycs = ys - np.cos(yaws) * radius  
	if CCC[2]=='L':
		xcg = xg - np.sin(yawg) * radius
		ycg = yg + np.cos(yawg) * radius
	elif CCC[2] == 'R':
		xcg = xg + np.sin(yawg) * radius
		ycg = yg - np.cos(yawg) * radius
  
	#Plot the car in start and end position               
	ax.arrow(xs, ys, car_length*np.cos(yaws), car_length*np.sin(yaws), width=car_length, head_width=car_length, head_length=0.09, fc='b', ec='b') 
	ax.arrow(xg, yg, car_length*np.cos(yawg), car_length*np.sin(yawg), width=car_length, head_width=car_length, head_length=0.09, fc='b', ec='b') 

	#Plot turning circle at the start	and at the goal
	circs = plt.Circle((xcs, ycs), radius, color='r', fill=False)
	ax.add_artist(circs) 
	circg = plt.Circle((xcg, ycg), radius, color='r', fill=False)
	ax.add_artist(circg)
    
	#Calculate the line between the two circles
	x1,y1,x3,y3=calc_intersections(xcs,ycs,xcg,ycg,radius,'start',CCC) #both intersections from the line with the first circle
	x1,y1=calc_middle(x1,y1,x3,y3,xcs,ycs,radius,CCC[0])							 #calculate the middle of both
	x2,y2,x4,y4=calc_intersections(xcs,ycs,xcg,ycg,radius,'end',CCC)	 #both intersections from the line with the second circle
	x2,y2=calc_middle(x2,y2,x4,y4,xcg,ycg,radius,CCC[2])							 #calculate the middle of both
  #ax.plot([x1,x2],[y1,y2], color='g')				#plot the calculated line 
    
  #Calculate m and b from this connection line
	m1 = (y2-y1)/(x2-x1)
	b1=y1-m1*x1
   
	#Calculate the circle center
		#center of the line = center of the circle in die middle
	h = np.sqrt((x1-x2)**2+(y1-y2)**2)
	a = np.arcsin((y2-y1)/h)
	ycm=np.sin(a)*(h/2)					
	xcm=np.sqrt((h/2)**2-ycm**2)
	#ax.scatter(*(xcm,ycm),color="c")

    
	#Orthogonale of the connection line between the circles
	m2 = -1/m1
	b2=ycm-(m2*xcm)
	hy1=m2*(xcm+2)+b2	
	hy2=m2*(xcm-2)+b2
	#ax.plot([(xcm+2),(xcm-2)],[hy1,hy2], color='g')
       
	if CCC[1]=='R':
		ycm=ycs
		xcm=(ycm-b2)/m2
		#Mittelkreis verschieben,sodass y kleiner ycs    
		intersections_1 = find_intersections(xcs,ycs,radius,xcm,ycm,radius)
		intersections_2 = find_intersections(xcg,ycg,radius,xcm,ycm,radius)
		vx=0
		ret = 0
		while ret != 1:
			ycm=ycm-0.1
			xcm=(ycm-b2)/m2+vx
			intersections_1 = find_intersections(xcs,ycs,radius,xcm,ycm,radius)
			intersections_2 = find_intersections(xcg,ycg,radius,xcm,ycm,radius)
			if len(intersections_1)==0 and len(intersections_2)!=0:
				vx=vx-0.1
				ycm=ycm+0.1
				xcm=xcm+vx
			intersections_1 = find_intersections(xcs,ycs,radius,xcm,ycm,radius)
			intersections_2 = find_intersections(xcg,ycg,radius,xcm,ycm,radius)
			if len(intersections_1)!=0 and len(intersections_2)==0:
				vx=vx+0.1
				ycm=ycm+0.1
				xcm=xcm+vx
			intersections_1 = find_intersections(xcs,ycs,radius,xcm,ycm,radius)
			intersections_2 = find_intersections(xcg,ycg,radius,xcm,ycm,radius)
			if len(intersections_1)==0 and len(intersections_2)==0:
				ycm=ycm+0.1
				xcm=(ycm-b2)/m2+vx
				intersections_1 = find_intersections(xcs,ycs,radius,xcm,ycm,radius)
				intersections_2 = find_intersections(xcg,ycg,radius,xcm,ycm,radius)
				ret = 1
                
		circm = plt.Circle((xcm, ycm), radius, color='r', fill=False)
		ax.add_artist(circm)
		if len(intersections_1)==0 or len(intersections_2)==0:
			plt.show()
			print "No solution found"
			return
                
    #plt.scatter(*(intersections_1[0][0],intersections_1[0][1]),color='c')
    #plt.scatter(*(intersections_1[1][0],intersections_1[1][1]),color='c')
    #plt.scatter(*(intersections_2[0][0],intersections_2[0][1]),color='c')
    #plt.scatter(*(intersections_2[1][0],intersections_2[1][1]),color='c')
   
		x1 = intersections_1[0][0]	#intersection start and middle circle
		y1 = intersections_1[0][1]
		x2 = intersections_2[0][0]	#intersection middle and final circle
		y2 = intersections_2[0][1]
				
		L_bogen1,alpha1 = calc_kreisbogen(start,[(x1,y1)],radius,CCC[0],'start')
		L_bogen3,alpha3 = calc_kreisbogen([(x1,y1)],[(x2,y2)],radius,CCC[1],'mid')
		L_bogen2,alpha2 = calc_kreisbogen(goal,[(x2,y2)],radius,CCC[2],'end')
    
		#Plot the driven arcs		
		#start cicle
		if CCC[0]=='L': 
			offset1 = np.rad2deg(yaws)-90
		elif CCC[0]=='R':
			offset1=180-np.rad2deg(alpha1)			    
		arc1 = matplotlib.patches.Arc((xcs, ycs), 2*radius, 2*radius,angle=offset1,theta1=0,theta2=np.rad2deg(alpha1), color='g')
		ax.add_artist(arc1)
    #final circle
		if CCC[2]=='L':  
			offset2=np.rad2deg(yawg)-90+360-np.rad2deg(alpha2)
		elif CCC[2]=='R':
			offset2=np.rad2deg(yawg)+90         	    
		arc2 = matplotlib.patches.Arc((xcg, ycg), 2*radius, 2*radius,angle=offset2,theta1=0,theta2=np.rad2deg(alpha2), color='g')
		ax.add_artist(arc2)
		#middle circle
		offset3=70 
		arc3 = matplotlib.patches.Arc((xcm, ycm), 2*radius, 2*radius,angle=offset3,theta1=0,theta2=np.rad2deg(alpha3), color='g')
		ax.add_artist(arc3)
    
		plt.show()    
		return L_bogen1,L_bogen3,L_bogen2  

def main():
	start = [(2,3,np.pi/2)]  
	goal = [(8,4,0)]
	turning_radius = 4

	CSC = 'LSL'
	L1,L2,L3=calc_CSC(start,goal,turning_radius, CSC)
	print "%s = %5.2f with %s = %5.2f, %s = %5.2f, %s = %5.2f" % (CSC,L1+L2+L3,CSC[0],L1,CSC[1],L2,CSC[2],L3)
	CSC = 'RSL'
	L1,L2,L3=calc_CSC(start,goal,turning_radius, CSC)
	print "%s = %5.2f with %s = %5.2f, %s = %5.2f, %s = %5.2f" % (CSC,L1+L2+L3,CSC[0],L1,CSC[1],L2,CSC[2],L3)
	CCC = 'LRL'
	L1,L2,L3=calc_CCC(start,goal,turning_radius, CCC)
	print "%s = %5.2f with %s = %5.2f, %s = %5.2f, %s = %5.2f" % (CCC,L1+L2+L3,CCC[0],L1,CCC[1],L2,CCC[2],L3)

if __name__ == '__main__':
	main()
