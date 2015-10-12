#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
from functions import angle_wrap

#===============================================================================
def splitandmerge(points, split_thres=0.1, inter_thres=0.3, min_points=6, dist_thres=0.12, ang_thres=np.deg2rad(10)):
    '''
    Takes an array of N points in shape (2, N) being the first row the x
    position and the second row the y position.

    Returns an array of lines of shape (L, 4) being L the number of lines, and
    the columns the initial and final point of each line in the form
    [x1 y1 x2 y2].

    split_thres: distance threshold to provoke a split
    inter_thres: maximum distance between consecutive points in a line
    min_point  : minimum number of points in a line
    dist_thres : maximum distance to merge lines
    ang_thres  : maximum angle to merge lines
    '''

    lines = split(points, split_thres, inter_thres, min_points, 0, points.shape[1]-1)
    return merge(lines, dist_thres, ang_thres)
    
#===============================================================================
def split(points, split_thres, inter_thres, min_points, first_pt, last_pt):
    '''
    Find lines in the points provided.
    first_pt: column position of the first point in the array
    last_pt : column position of the last point in the array
    '''
    assert first_pt >= 0
    assert last_pt <= points.shape[1]
    
    # TODO: CODE HERE!!!
    # Check minimum number of points
    if points.shape[1] < min_points:
        print('The number of points is less than the minimum')
        return None

    # Line defined as "a*x + b*y + c = 0"
    # Calc (a, b, c) of the line (prelab question)
    x1 = points[0, first_pt]
    y1 = points[1, first_pt]
    x2 = points[0, last_pt]
    y2 = points[1, last_pt]
    a = (y2-y1)/(x2-x1)
    b = (-1)
    c = y1-(y2-y1)/(x2-x1)*x1
    # Distances of points to line (prelab question)
    print('First is : %d  and last is : %d' %(first_pt, last_pt))
    dist = np.zeros(last_pt - first_pt+1)
    for i in range(first_pt, last_pt):
        dist[i- first_pt]= abs(a*points[0][i]+b*points[1][i]+c)/np.sqrt(a**2+b**2);
    indexMax = np.argmax(dist)                 # Index of the point with max. dist from line
    maximum = dist[indexMax]
    print ('The index of the point is %f , and the distance is %f ' %(indexMax, maximum))    
    #

    # Check split threshold
    if dist[indexMax] > split_thres:
        # Check sublines
        #
        indexMax = indexMax + first_pt
        prev = split(points, split_thres, inter_thres, min_points, first_pt, indexMax)
        post = split(points, split_thres, inter_thres, min_points, indexMax+1, last_pt)  
        # Return results of sublines
        if prev is not None and post is not None:
            return np.vstack((prev, post))
        elif prev is not None:
            return prev
        elif post is not None:
            return post
        else:
            return None

    # Do not need to split furthermore
    else:
        # Optional check interpoint distance
        for i in range(first_pt, last_pt):
            x11 = points[0][i]
            y11 = points[1][i]
            x22 = points[0][i+1]
            y22 = points[1][i+1]
            # Check interpoint distance threshold
            if np.sqrt(((x22-x11)**2+(y22-y11)**2)) > inter_thres:
                return None
        
        # It is a good line
        print ('The points for the line are  x1: %f , y1: %f   x2: %f , y2: %f ' %(x1, y1, x2, y2))   
        return np.array([[x1, y1, x2, y2]])
        
    # Dummy answer --> delete it
    #return np.array([[x1, y1, x2, y2]])

#===============================================================================
def merge(lines, dist_thres, ang_thres):
    '''
    Merge similar lines according to the given thresholds.
    '''
    #print ('Number of lines before merging is %f ' %(range(lines)))
    # No data received
    if lines is None:
        return None
        
    # Check and merge similar consecutive lines
    i = 0
    print(lines)
    while i in range(lines.shape[0]-1):
        print(lines[i])

        # Line angles
        print ('This is i: %d ' %(i))
        print ('This is i: %f ' %(lines[i,1]))
        print ('This is i: %f ' %(lines[i,2]))
        print ('This is i: %f ' %(lines[i,3]))
        print ('This is i: %f ' %(lines[i,0]))
        ang1 = np.arctan2(lines[i,3]-lines[i,1], lines[i,2]-lines[i,0])
        ang2 = np.arctan2(lines[i+1][3]-lines[i+1][1], lines[i+1][2]-lines[i+1][0])
        
        # Below thresholds?
        angdiff = abs(ang2 - ang1) 
        dist1 = np.sqrt((lines[i+1][3]-lines[i][3])**2+ (lines[i+1][2]-lines[i][2])**2)
        dist2 = np.sqrt((lines[i+1][3]-lines[i][1])**2+ (lines[i+1][2]-lines[i][0])**2)
        dist3 = np.sqrt((lines[i+1][1]-lines[i][3])**2+ (lines[i+1][0]-lines[i][2])**2)
        dist4 = np.sqrt((lines[i+1][1]-lines[i][1])**2+ (lines[i+1][0]-lines[i][0])**2)
        distdiff = min(dist1,dist2,dist3,dist4)
        if angdiff < ang_thres and distdiff < dist_thres:
            
            # Joined line
            print ('Merging the line %f , and the line %f ' %(i, (i+1)))
            p = [lines[i,0], lines[i,1], lines[i+1,2], lines[i+1, 3]]
            print(p)
            lines[i] = np.array(p)
            print (lines[i])

            # Delete unnecessary line
            lines = np.delete(lines, i+1, 0)
            
        # Nothing to merge
        else:
            i += 1
    #print ('Number of lines after merging is %f ' %(range(lines)))        
    return lines
