import rospy
import random
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from balloon_detector import vector_to_angle

xmin = 0			#field borders
ymin = 0
xmax = 7.0
ymax = 5.0	
numParticles = 100

cells_x = 70  #cells in each direction for the grid
cells_y = 50
 
def new_marker(x,y,color,size=0.1):   #generate a new marker
	marker = Marker()
	marker.header.frame_id = "/world"
	marker.header.stamp = rospy.Time.now();
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	marker.pose.position.x = x
	marker.pose.position.y = y
	marker.pose.position.z = 0
	marker.pose.orientation.z = 1 #z[i]
	marker.scale.x = size
	marker.scale.y = size
	marker.scale.z = 0.00001
	marker.color.a = 1.0
	if color == 'r':
		marker.color.r = 1.0
	elif color == 'g':
		marker.color.g = 1.0
	elif color == 'b':
		marker.color.b = 1.0
	return marker

def init_field():
	field = MarkerArray()
	bounds_x = [xmin,xmin,xmax,xmax]
	bounds_y = [ymin,ymax,ymin,ymax]
	lamps_x = [2.29,3.55,4.18,2.29] 
	lamps_y = [2.4,3.03,1.77,1.14]
	#show field borders in green
	for i in range(4):                       
		marker = new_marker(bounds_x[i],bounds_y[i],'g',size=0.15)
		field.markers.append(marker)
	#show lamps in blue
	for i in range(4):										
		marker = new_marker(lamps_x[i],lamps_y[i],'b',size=0.3)
		field.markers.append(marker)
	#show the grid in green
	#cells_x = 50
	#cells_y = 30
	#x = [None]*cells_x
	#for xi in range(cells_x):
	#	for yi in range(cells_y):
	#		marker = new_marker((xi*xmax/cells_x),(yi*ymax/cells_y),'g',size=0.025)
	#		field.markers.append(marker)
	id_ = 0
	for f in field.markers:
		f.id = id_
		id_ +=1
	return field

def init_particles():    
	random.seed(0)   #to get for every start the same init-distribution
	particles = MarkerArray()
	for i in range(numParticles):
		marker = new_marker(random.uniform(xmin,xmax),random.uniform(ymin,ymax),'r')
		particles.markers.append(marker)
	id_ = 0
	for m in particles.markers:
		m.id = id_
		id_ +=1
	return particles
	

def move_particles(particles,changes,z):
	#changes -> [x,y]
	for i in range(numParticles):
		particles.markers[i].pose.position.x += changes[0] + random.uniform(-0.01,0.01) 
		particles.markers[i].pose.position.y += changes[1] + random.uniform(-0.01,0.01)
		particles.markers[i].pose.orientation.z = z + random.uniform(-0.001,0.001)
	return particles


def calc_angles_particles2balloons(detector,particles,img):
	#lamps -> red, purple, blue, green
	lamp_positions = np.array([[3.55,3.03],[2.29,2.4],[4.18,1.77],[2.29,1.14]])
	pb_angles = [None]*numParticles
	for i in range(numParticles):
		pos = [particles.markers[i].pose.position.x,particles.markers[i].pose.position.y]v #position from the particle
		angles = []
		for lamp in range(4):
			x = lamp_positions[lamp][0]-pos[0]	#difference between particle position and lamp
			y = lamp_positions[lamp][1]-pos[1]			
			angles.append(np.arctan2(y,x))	 
		pb_angles[i]=angles
	return pb_angles

def calc_weights(pb_angles,cb_angles,cb_colors):
	#pb_angles->red, purple, blue, green   #cb_angles->colors listed in cb_colors
	weights = [None]*numParticles
	standardDeviation = np.deg2rad(20)
	for w in range(numParticles):
		weight = 1
		for i in range(len(cb_angles)):	#select the angle to the matching lamp
			if cb_colors[i] == 'red':
				e = 0
			elif cb_colors[i] == 'purple':
				e = 1
			elif cb_colors[i] == 'blue':
				e = 2
			else: #'green'
				e = 3
			expectedAngle = pb_angles[w][e]
			perceivedAngle = cb_angles[i]

			weight = weight * np.exp(-((expectedAngle-perceivedAngle)**2/standardDeviation**2))
		weights[w]= weight
	#normalize the weights
	weights = 1/np.sum(weights)*np.array(weights)
	return weights

def resample_particles(particles, weights):  #low variance resampling
	help = [None]*len(weights)
	help[0]= [0.,weights[0]]
	for i in range(1,len(weights)):
		help[i]	= [help[i-1][1],help[i-1][1]+weights[i]] #calculate the bounds for each weight

	look_at = []	
	for M in range(numParticles):
		look_at.append((1.0*M+0.5)/numParticles)		#calculate the "look-at"-points
	auswahl = []
	p = 0  #look-at point
	i = 0	 #weight index
	while len(auswahl)<numParticles:
		if look_at[p]>=help[i][0] and look_at[p]<help[i][1]:
				auswahl.append(i)   #add the index of the selected weight
				p += 1
		else:
				i += 1

	new_particles = MarkerArray()
	new_particles.markers = [None]*numParticles
	for ind in range(len(auswahl)):			#generate the new particle set
		x = particles.markers[auswahl[ind]].pose.position.x
		y = particles.markers[auswahl[ind]].pose.position.y
		new_particles.markers[ind] = new_marker(x,y,'r')
		new_particles.markers[ind].pose.orientation.z = particles.markers[auswahl[ind]].pose.orientation.z
	id_ = 0
	for c in new_particles.markers:
		c.id = id_
		id_ +=1
	return new_particles

def calc_position(particles):
	dif_x = (xmax-xmin)/cells_x 
	dif_y = (ymax-ymin)/cells_y
	counter = [0]*cells_x*cells_y 
	part = [[]]*cells_x*cells_y

	for p in range(numParticles): #go through all particles
		x_cell = round(particles.markers[p].pose.position.x/dif_x) #calculate the matching grid cell
		y_cell = round(particles.markers[p].pose.position.y/dif_y)
		if x_cell > cells_x or y_cell > cells_y:
			a = 0 #outside the field
		else: #inside the field
			counter[int(x_cell*cells_y+y_cell)]+= 1 #increase counter in the corresponding grid cell
			particle_tuple = part[int(x_cell*cells_y+y_cell)]
			particle_tuple.append([particles.markers[p].pose.position.x,particles.markers[p].pose.position.y,particles.markers[p].pose.orientation.z])
			part[int(x_cell*cells_y+y_cell)] = particle_tuple  #add the x, y and z(orientation)component 
	
	ind = counter.index(max(counter))  #find grid cell with the most particles
	if len(part[ind]) != 0:
		x = 0.
		y = 0.
		vs = 0.
		vc = 0.
		for i in range(len(part[ind])):
			x += part[ind][i][0]
			y += part[ind][i][1]
			vs += np.sin(part[ind][i][2])
			vc += np.cos(part[ind][i][2])
		return 1./len(part[ind])*x,1./len(part[ind])*y,np.arctan2(vs,vc) #calculate the means
	else:
		return 0.,0.,0.
		



