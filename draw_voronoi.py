#!/usr/bin/env python

#RO-11-Germanidis-Witt, Task1b

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi, voronoi_plot_2d
from create_RRT import create_RTT

def main():
	step_length = 2
	start = [(0,0)]
	points = [(3,4),(2,1),(5,4),(2,2),(8,1)]  
	vertices,edges = create_RTT(5,points,[(0,0)],step_length) #calculate the RRT
	vor = Voronoi(vertices) #calculate Voronoi edges and vertices
	voronoi_plot_2d(vor)		#plot voronoi diagramm
	plt.show()

if __name__ == '__main__':
	main()
