import math
import numpy as np




def create_helix_trajectory(first,waypoints=[]):
    
 
    
    #start end height of helix (y-axis)
    h0,h1 = first[2], first[2] + 75
    #radius
    radius = 60.5
    #  distance between two screws (the y movement after full circle)
    d = 2.0
    # parameter <0,1> determining the position on helix
    t = 0
    # angular start position [rad]
    a0 = 0
    
    # angular speed coefficient
    aa= math.fabs(h1-h0) * 2.0 * math.pi/d 

    waypoints.append(first)
    
    i = 0

    while t <= 1.0:
        x = first[0] + (math.sin(a0) * radius)
        y = first[1] + (math.cos(a0) * radius) 
        z = first[2] - (h1-h0)*(t)
        h  = np.arctan2((y-waypoints[i][1]), (x-waypoints[i][0]))
        waypoints.append([int(x),int(y),int(z),h])
        t += 1.0/30
        if a0 < 2 * math.pi:
           a0+= (2 * math.pi)/10
        else:
           a0 = 0.0
        i+=1
    
    second = waypoints[-1]

    while 1.0 < t <= 2.0:
        x = first[0] + (math.sin(a0) * radius)
        y = first[1] + (math.cos(a0) * radius)
        z = second[2] + (h1-h0)*(t-1)
        h  = np.arctan2((y-waypoints[i][1]), (x-waypoints[i][0]))
        waypoints.append([int(x),int(y),int(z),h])
        t += 1.0/30
        if a0 < 2 * math.pi:
           a0+= (2 * math.pi)/10
        else:
           a0 = 0.0
        i+=1
    
    waypoints.append(first)
    

    return waypoints        