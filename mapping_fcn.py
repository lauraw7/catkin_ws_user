#!/usr/bin/env python

#RO-04-Germanidis-Witt

import numpy

def mapping_fcn(steering_angle):
    #function which gets angles of the front wheel in deg as an input and 
    #returns values from 0 to 179
    
    #interpolate data with a higher degree polynomial
    #p(x) = a0 + a1*x + a2*x² + a3*x³ + a4*x⁴ + a5*x⁵ + a6*x⁶

    #measured values from task 2b
    x1 = 0  
    y1 = 1  
    x2 = 30
    y2 = 2  
    x3 = 60
    y3 = 3  
    x4 = 90
    y4 = 4  
    x5 = 120
    y5 = 5  
    x6 = 150
    y6 = 6  
    x7 = 179
    y7 = 7  

    #concenate in matrices
    x = [[1,x1,x1**2,x1**3,x1**4,x1**5,x1**6],[1,x2,x2**2,x2**3,x2**4,x2**5,x2**6],[1,x3,x3**2,x3**3,x3**4,x3**5,x3**6], \
        [1,x4,x4**2,x4**3,x4**4,x4**5,x4**6],[1,x5,x5**2,x5**3,x5**4,x5**5,x5**6],[1,x6,x6**2,x6**3,x6**4,x6**5,x6**6], \
        [1,x7,x7**2,x7**3,x7**4,x7**5,x7**6]]
    y = [y1, y2, y3, y4, y5, y6, y7]

    #solve the system of equations
    a = np.linalg.solve(x, y)

    #calculate the corresponding angle
    return value = a[0]+a[1]*degree+a[2]*degree**2+a[3]*degree**3+a[4]*degree**4+a[5]*degree**5+a[6]*degree**6
