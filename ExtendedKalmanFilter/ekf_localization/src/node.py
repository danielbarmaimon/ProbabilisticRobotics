#!/usr/bin/python
# -*- coding: utf-8 -*-

# Basic imports
import roslib
roslib.load_manifest('ekf_localization')
import rospy
import tf

# ROS messages
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Maths
import numpy as np

# Transforms
from tf.transformations import euler_from_quaternion

# Custom libraries
from splitandmerge import splitandmerge
from functions import publish_lines, get_map, get_ekf_msgs, yaw_from_quaternion, angle_wrap
from ekf_localization import EKF

#===============================================================================
class LocalizationNode(object):
    '''
    Class to hold all ROS related transactions to use split and merge algorithm.
    '''
    
    #===========================================================================
    def __init__(self, x0=0.8,y0=0.3,theta0=-0.03, odom_lin_sigma=0.025, odom_ang_sigma=np.deg2rad(2),
                        meas_rng_noise=0.2,  meas_ang_noise=np.deg2rad(10)):
        '''
        Initializes publishers, subscribers and the particle filter.
        '''
        # Publishers
        self.pub_lines = rospy.Publisher("lines", Marker)
        self.pub_odom = rospy.Publisher("predicted_odom", Odometry)
        self.pub_uncertainity = rospy.Publisher("uncertainity",  Marker)
        
        # Subscribers
        self.sub_scan = rospy.Subscriber("scan", LaserScan, self.laser_callback)
        self.sub_odom = rospy.Subscriber("odom", Odometry, self.odom_callback)
        
        # TF
        self.tfBroad = tf.TransformBroadcaster()
        
        # Incremental odometry
        self.last_odom = None
        self.odom = None
        
        # Flags
        self.new_odom = False
        self.new_laser = False
        self.pub = False
        
        # Particle filter
        self.ekf = EKF(get_map(), x0, y0, theta0, odom_lin_sigma, odom_ang_sigma, meas_rng_noise, meas_ang_noise)
    
    #===========================================================================
    def odom_callback(self, msg):
        '''
        Publishes a tf based on the odometry of the robot and calculates the incremental odometry as seen from the vehicle frame.
        '''
        # Save time
        self.time = msg.header.stamp
        
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
                                   
        # Incremental odometry
        if self.last_odom is not None:
            
            # Increment computation
            delta_x = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
            delta_y = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y
            yaw = yaw_from_quaternion(msg.pose.pose.orientation)
            lyaw = yaw_from_quaternion(self.last_odom.pose.pose.orientation)
            
            # Odometry seen from vehicle frame
            self.uk = np.array([delta_x * np.cos(lyaw) + delta_y * np.sin(lyaw),
                              - delta_x * np.sin(lyaw) + delta_y * np.cos(lyaw),
                                angle_wrap(yaw - lyaw)])
            
            # Flag
            self.new_odom = True
        
        # For next loop
        self.last_odom = msg
        
    
    #===========================================================================
    def laser_callback(self, msg):
        '''
        Function called each time a LaserScan message with topic "scan" arrives. 
        '''
        # Save time
        self.time = msg.header.stamp
        
        # Project LaserScan to points in space
        rng = np.array(msg.ranges)
        ang = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        points = np.vstack((rng * np.cos(ang),
                            rng * np.sin(ang)))
                            
        # Filter long ranges
        cond = rng < msg.range_max
        points = points[:, rng < msg.range_max]
        
        # Use split and merge to obtain lines and publish
        self.lines = splitandmerge(points, split_thres=0.1,
                                      inter_thres=0.3,
                                      min_points=6,
                                      dist_thres=0.12,
                                      ang_thres=np.deg2rad(10))

        # Have valid points
        if self.lines is not None:
            
            # Publish results
            publish_lines(self.lines, self.pub_lines, frame='/robot',
                      time=msg.header.stamp, ns='scan_lines_robot', color=(0,0,1))
            
            # Flag
            self.new_laser = True
    
    #===========================================================================
    def iterate(self):
        '''
        Main loop of the filter.
        '''
        # Prediction
        if self.new_odom:
            
            self.ekf.predict(self.uk.copy())
            self.new_odom = False
            self.pub = True
            
        # Weightimg and resampling
        if self.new_laser:
            
            Innovk_List, H_k_List, S_f_List, Rk_List=self.ekf.data_association(self.lines.copy())
            self.ekf.update_position(Innovk_List, H_k_List, S_f_List, Rk_List)
            self.new_laser = False
            self.pub = True
            
        # Publish results
        if self.pub:
            self.publish_results()
            self.pub = False
    
    #===========================================================================
    def publish_results(self):
        '''
        Publishes all results from the filter.
        '''
        # Map of the room
        map_lines = get_map()
        publish_lines(map_lines, self.pub_lines, frame='/world', ns='map', color=(0,1,0))
        
        msg_odom, msg_ellipse, trans, rot = get_ekf_msgs(self.ekf)

        self.pub_odom.publish(msg_odom)
        self.pub_uncertainity.publish(msg_ellipse)
        self.tfBroad.sendTransform(translation = trans,
                                   rotation = rot, 
                                   time = self.time,
                                   child = '/robot',
                                   parent = '/world')
 
#===============================================================================       
if __name__ == '__main__':
    
    # ROS initializzation
    rospy.init_node('localization')
    node = LocalizationNode(x0=0.8,y0=0.3,theta0=-0.03,
                            odom_lin_sigma = 0.025,
                            odom_ang_sigma = np.deg2rad(2),
                            meas_rng_noise = 0.2,
                            meas_ang_noise = np.deg2rad(10))
    # Filter at 10 Hz
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
    
        # Iterate filter
        node.iterate()
        r.sleep()
