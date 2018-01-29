#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

def create_RTT(num_samples,samples,start,step_length):
	vertices = start
	edges = []
	for i in range(num_samples):
		sample = samples[i]
		diff = [0]*len(vertices)
		if len(vertices)>= 0:
			for v in range(len(vertices)):
				diff[v]=np.sqrt((vertices[v][0]-sample[0])**2+(vertices[v][1]-sample[1])**2) #calculate difference to each vertices in the graph
		ind = diff.index(min(diff)) #choose the vertice with the smallest distance
		if diff[ind] <= step_length: #distance is smaller than the step length -> add the sample directly as a new vertices
			vertices.append(sample)
			edges.append((sample,vertices[ind]))
		else:
			alpha = np.arcsin((sample[1]-vertices[ind][1])/diff[ind]) 
			y_triangle = np.sin(alpha)*step_length										
			x_triangle = np.sqrt(step_length**2-y_triangle**2)
			x_new = vertices[ind][0]+x_triangle				#calculate new x-coordinate											
			y_new = vertices[ind][1]+y_triangle				#calculate new y-coordinate
			new_sample = (x_new,y_new)
			vertices.append(new_sample)	#add the vertice with the new coordinates
			edges.append((new_sample,vertices[ind]))
	return vertices,edges

def plot_RTT(vertices,edges):
    for e in range(len(edges)): #plot all edges as red lines
        plt.plot([edges[e][0][0],edges[e][1][0]],[edges[e][0][1],edges[e][1][1]],color='r')#([x1,x2],[y1,y2])

    for v in range(len(vertices)):#plot all vertices as filled circles
        if v == 0:
            plt.scatter(*(vertices[v][0],vertices[v][1]), s=500, color='g') #start in green
            plt.text(vertices[v][0]-0.15,vertices[v][1]-0.05,'start')
        else: 
            plt.scatter(*(vertices[v][0],vertices[v][1]), s=500, color='r') #other vertices in red
            plt.text(vertices[v][0],vertices[v][1],v)    
    plt.show()

def main():
	step_length = 2
	start = [(0,0)]
	points = [(3,4),(2,1),(5,4),(2,2),(8,1)]  
	vertices,edges = create_RTT(5,points,[(0,0)],step_length)
	plot_RTT(vertices,edges) 

if __name__ == '__main__':
	main()
