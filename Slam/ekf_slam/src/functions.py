#!/usr/bin/python
# -*- coding: utf-8 -*-

# ROS init
import roslib
roslib.load_manifest('ekf_localization')
import rospy

# Math
import numpy as np
from math import *

# ROS messages
from sensor_msgs.msg import LaserScan, PointCloud
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, PoseArray, PoseStamped, Pose

# Transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler

############################################################################################################
def publish_lines(lines, publisher, frame='/world', ns='none', time=None, color=(1,0,0)) :
    '''
    Publishes lines from an array of shape (N, 4) being N the number of lines.
    Lines are represented by the start and end points as [x1 y1 x2 y2].
    '''
    # Create message
    msg = Marker()
    msg.header.stamp = time if time is not None else rospy.Time.now()
    msg.header.frame_id = frame
    msg.ns = ns
    msg.id = 0
    msg.type = msg.LINE_LIST
    msg.action = msg.ADD
    msg.pose.position.x = 0.0
    msg.pose.position.y = 0.0
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    msg.scale.x = 0.01
    msg.scale.y = 0.0
    msg.scale.z = 0.0
    msg.color.r = color[0]
    msg.color.g = color[1]
    msg.color.b = color[2]
    msg.color.a = 1.0
    for i in range(lines.shape[0]):
        msg.points.append(Point(lines[i,0], lines[i,1], 0))
        msg.points.append(Point(lines[i,2], lines[i,3], 0))
    
    # Publish
    publisher.publish(msg)

############################################################################################################
#def publish_scan_points(pub_scan_points,pointsWorldFrame) :
    #'''
    #publish_scan_points() publishes the laser scan in world frame
    #''' 
    #scan_points=PointCloud()
    #scan_points.header.stamp = rospy.Time.now()
    #scan_points.header.frame_id="/world";
    #i = 0
    #while i <len(pointsWorldFrame) and len(pointsWorldFrame)!=1:
         #point=Point()
         #point.x=pointsWorldFrame[i,0]
         #point.y=pointsWorldFrame[i,1]
         #point.z=0
         #scan_points.points.append(point)
         #i+=1
    #pub_scan_points.publish(scan_points)

############################################################################################################
def get_map(x=0, y=0, a=0):
    '''
    Retrieves the map with offsets [x y a] if necessary.
    Lines defined as [x1 y1 x2 y2].
    For the EKF x = 0.7841748 y = 0.313926 a = -0.03
    '''
    lines = np.array([[0.00, 0.00, 0.00, 0.77],
                      [0.00, 0.77, 0.77, 0.77],
                      [0.77, 0.77, 0.77, 2.80],
                      [0.77, 2.80, 4.59, 2.80],
                      [4.59, 2.80, 4.59, 2.64],
                      [4.59, 2.64, 4.89, 2.64],
                      [4.89, 2.64, 4.89, 0.00],
                      [4.89, 0.00, 4.12, 0.00],
                      [4.12, 0.00, 4.12, 0.40],
                      [4.12, 0.40, 3.07, 0.40],
                      [3.07, 0.40, 3.07, 0.00],
                      [3.07, 0.00, 0.00, 0.00]]).T
    lines -= np.array([[x, y, x, y]]).T
    rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    rotate = np.vstack(( np.hstack(( rot, np.zeros((2,2)) )),
                         np.hstack(( np.zeros((2,2)), rot )) ))
    return np.dot(rotate, lines).T

############################################################################################################
#def mat2str(s,m) :
    #'''
    #mat2str() is a function that prints a matrix in the way that it can be seen propperly
    #s -> string to print as name of the matrix. It will appear at the beginig of the 1st row
    #m -> matrix/array to print
    #mat2str('hello: ',eye(3))
    #hello: [1 0 0]
           #[0 1 0]
           #[0 0 1]
    #'''
    #if m.ndim < 2:
        #m = array([m])
        
    #l = len(s)
    #for i in range(len(m)) :
        #if i != 0 :
            #s = s+'\n'
            #for j in range(l) : 
                #s = s+' '
        #s = s+str(m[i,:])
    #return s

############################################################################################################
def angle_wrap(a):
    '''
    Returns the angle a normalized between -pi and pi.
    Works with numbers and numpy arrays.
    '''
    a = a % (2 * np.pi)
    if (isinstance(a, int) or isinstance(a, float)) and (a > np.pi):
        a -= 2 * np.pi
    elif isinstance(a, np.ndarray): # arrays
        a[a > np.pi] -= 2 * np.pi
    return a

############################################################################################################
def comp(a, b) :
    '''
    comp(a,b) composes matrices a and b, being b the one that has to be transformed into a space.
    '''

    c1 = cos(a[2]) * b[0]  - sin(a[2]) * b[1] + a[0]
    c2 = sin(a[2]) * b[0]  + cos(a[2]) * b[1] + a[1]
    c3 = a[2] + b[2]
    c3 = angle_wrap(c3)
    C = np.array([c1, c2, c3])
    return C
    
#===============================================================================
def compInv(x):
    c3 = angle_wrap(-x[2])
    cc3 = cos(c3)
    sc3 = sin(c3)
    c1 = cc3*(-x[0]) - sc3 * (-x[1])
    c2 = sc3*(-x[0]) + cc3 * (-x[1])
    
    J = -np.eye(3)
    J[0,0] = -cc3
    J[0,1] = sc3
    J[0,2] = -x[0]*sc3 - x[1]*cc3
    J[1,0] = -sc3
    J[1,1] = -cc3
    J[1,2] = x[0]*cc3 - x[1]*sc3
    return np.array([c1,c2,c3]), J
    
#===============================================================================
def yaw_from_quaternion(quat):
    '''
    Returns yaw extracted from a geometry_msgs.msg.Quaternion.
    '''
    return euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]

#===============================================================================
def get_ekf_msgs(ekf):
    '''
    Create following messages:
    1-Robot odometry
    2-Robot uncertainity
    '''
    # Odometry
    msg_odom = Odometry()
    msg_odom.header.stamp = rospy.Time.now()
    msg_odom.header.frame_id = '/world'
    msg_odom.pose.pose.position.x= ekf.x_B_1[0]
    msg_odom.pose.pose.position.y= ekf.x_B_1[1]
    msg_odom.pose.pose.position.z= 0
    quat = quaternion_from_euler(0, 0, ekf.x_B_1[2])
    msg_odom.pose.pose.orientation.x = quat[0]
    msg_odom.pose.pose.orientation.y = quat[1]
    msg_odom.pose.pose.orientation.z = quat[2]
    msg_odom.pose.pose.orientation.w = quat[3]
        
    #Uncertainity
    msg_ellipse = Marker()
    msg_ellipse.header.frame_id = "/world"
    msg_ellipse.header.stamp = rospy.Time.now()
    msg_ellipse.type = Marker.CYLINDER
    msg_ellipse.pose.position.x = ekf.x_B_1[0]
    msg_ellipse.pose.position.y = ekf.x_B_1[1]
    msg_ellipse.pose.position.z = 0
    msg_ellipse.pose.orientation.x = 0
    msg_ellipse.pose.orientation.y = 0
    msg_ellipse.pose.orientation.z = 0
    msg_ellipse.pose.orientation.w = 1
    msg_ellipse.scale.x = ekf.P_B_1[0,0]
    msg_ellipse.scale.y = ekf.P_B_1[1,1]
    msg_ellipse.scale.z = 0.01
    msg_ellipse.color.a = 0.3
    msg_ellipse.color.r = 0.0
    msg_ellipse.color.g = 1.0
    msg_ellipse.color.b = 1.0
    
    # TF
    trans = (msg_odom.pose.pose.position.x, 
             msg_odom.pose.pose.position.y, 
             msg_odom.pose.pose.position.z)
    rotat = (msg_odom.pose.pose.orientation.x,
             msg_odom.pose.pose.orientation.y,
             msg_odom.pose.pose.orientation.z,
             msg_odom.pose.pose.orientation.w)
         
    # Reconstruct the map to visualize
    room_map_polar = np.zeros((0,2))
    room_map_points = np.zeros((0,4))
    for i in range(0,ekf.get_number_of_features_in_map()):
        if ekf.featureObservedN.shape[0] == 0 or ekf.featureObservedN[i] >= ekf.min_observations:
            rho = ekf.x_B_1[2*i+3]
            phi = ekf.x_B_1[2*i+4]
            plline = np.array([rho,phi])
            room_map_polar = np.vstack([room_map_polar,plline])
            aux = np.zeros((1,4))
            if np.abs(np.abs(phi)-np.pi/2)<np.deg2rad(45):
                # Horizontal line
                aux[0,0] = -1
                aux[0,2] = 5
            else:
                # Vertical line
                aux[0,0] = rho-1.5
                aux[0,2] = rho+1.5
        
            aux[0,1] = polar2y(plline,aux[0,0])
            aux[0,3] = polar2y(plline,aux[0,2])
            room_map_points = np.vstack([room_map_points,aux])
        
    return msg_odom, msg_ellipse, trans, rotat, room_map_points
    
#===============================================================================
def polar2y(line,x):
    '''
    given a line in polar coordinates and the x value of a point computes 
    its y projection
    '''  
    cos = np.cos(line[1])
    sin = np.sin(line[1])
    x0 = line[0]*cos
    y0 = line[0]*sin
    m = -cos/sin
    return m*(x-x0) + y0
