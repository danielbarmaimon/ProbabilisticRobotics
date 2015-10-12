#!/usr/bin/python
# -*- coding: utf-8 -*-

# Basic imports
import roslib
roslib.load_manifest('splitandmerge')
import rospy
import tf

# ROS messages
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Maths
import numpy as np

# Custom libraries
from splitandmerge import splitandmerge
from functions import publish_lines

#===============================================================================
class SplitAndMergeNode(object):
    '''
    Class to hold all ROS related transactions to use split and merge algorithm.
    '''
    
    #===========================================================================
    def __init__(self):
        '''
        Initializes publishers and subscribers.
        '''
        # Publishers
        self.pub_line = rospy.Publisher("lines", Marker)
        
        # Subscribers
        self.sub_scan = rospy.Subscriber("scan", LaserScan, self.laser_callback)
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.odom_callback)
        
        # TF
        self.tfBroad = tf.TransformBroadcaster()
    
    #===========================================================================
    def odom_callback(self, msg):
        '''
        Publishes a tf based on the odometry of the robot.
        '''
        # Translation
        trans = (msg.pose.pose.position.x, 
                 msg.pose.pose.position.y, 
                 msg.pose.pose.position.z)
        
        # Rotation
        rot = (msg.pose.pose.orientation.x,
               msg.pose.pose.orientation.y,
               msg.pose.pose.orientation.z,
               msg.pose.pose.orientation.w)
        
        # Publish transform
        self.tfBroad.sendTransform(translation = trans,
                                   rotation = rot, 
                                   time = msg.header.stamp,
                                   child = '/openni_depth_frame',
                                   parent = '/world')
    
    #===========================================================================
    def laser_callback(self, msg):
        '''
        Function called each time a LaserScan message with topic "scan" arrives. 
        '''
        
        # Project LaserScan to points in space
        rng = np.array(msg.ranges)
        ang = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        points = np.vstack((rng * np.cos(ang),
                            rng * np.sin(ang)))
                            
        # Filter long ranges
        cond = rng < msg.range_max
        points = points[:, rng < msg.range_max]
        
        # Use split and merge to obtain lines and publish
        lines = splitandmerge(points, split_thres=0.1,
                                      inter_thres=0.3,
                                      min_points=6,
                                      dist_thres=0.12,
                                      ang_thres=np.deg2rad(10))
        # Publish results
        publish_lines(lines, self.pub_line, frame=msg.header.frame_id,
                      time=msg.header.stamp, ns='scan_line', color=(1,0,0))
 
#===============================================================================       
if __name__ == '__main__':
    
    # ROS initializzation
    rospy.init_node('splitandmerge')
    node = SplitAndMergeNode()
    
    # Continue forever
    rospy.spin()
