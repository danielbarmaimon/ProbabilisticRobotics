#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import *
import numpy as np
from functions import angle_wrap, comp, compInv
import scipy.linalg
import rospy

#============================================================================
class EKF_SLAM(object):
    '''
    Class to hold the whole EKF-SLAM.
    '''
    
    #========================================================================
    def __init__(self, x0, y0, theta0, odom_lin_sigma, 
                 odom_ang_sigma, meas_rng_noise, meas_ang_noise):
        '''
        Initializes the ekf filter
        room_map : an array of lines in the form [x1 y1 x2 y2]
        num      : number of particles to use
        odom_lin_sigma: odometry linear noise
        odom_ang_sigma: odometry angular noise
        meas_rng_noise: measurement linear noise
        meas_ang_noise: measurement angular noise
        '''
        
        # Copy parameters
        self.odom_lin_sigma = odom_lin_sigma
        self.odom_ang_sigma = odom_ang_sigma
        self.meas_rng_noise = meas_rng_noise
        self.meas_ang_noise = meas_ang_noise
        self.chi_thres = 0 # TODO chose your own value
       
        # Odometry uncertainty 
        self.Qk = np.array([[ self.odom_lin_sigma**2, 0, 0],\
                            [ 0, self.odom_lin_sigma**2, 0 ],\
                            [ 0, 0, self.odom_ang_sigma**2]])
        
        # Measurements uncertainty
        self.Rk=np.eye(2)
        self.Rk[0,0] = self.meas_rng_noise
        self.Rk[1,1] = self.meas_ang_noise
        
        # State vector initialization
        self.x_B_1 = np.array([x0,y0,theta0]) # Position
        self.P_B_1 = np.zeros((3,3)) # Uncertainty
        
        # Initialize buffer for forcing observing n times a feature before 
        # adding it to the map
        self.featureObservedN = np.array([])
        self.min_observations = 0
    
    #========================================================================
    def get_number_of_features_in_map(self):
        '''
        returns the number of features in the map
        '''
        return (self.x_B_1.size-3)/2
    
    #========================================================================
    def get_polar_line(self, line, odom):
        '''
        Transforms a line from [x1 y1 x2 y2] from the world frame to the
        vehicle frame using odometry [x y ang].
        Returns [range theta]
        '''
        # Line points
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        
        # Compute line (a, b, c) and range
        line = np.array([y1-y2, x2-x1, x1*y2-x2*y1])
        pt = np.array([odom[0], odom[1], 1])
        dist = np.dot(pt, line) / np.linalg.norm(line[:2])
        
        # Compute angle
        if dist < 0:
            ang = np.arctan2(line[1], line[0])
        else:
            ang = np.arctan2(-line[1], -line[0])
        
        # Return in the vehicle frame
        return np.array([np.abs(dist), angle_wrap(ang - odom[2])])
        
    #========================================================================
    def predict(self, uk):
        '''
        Predicts the position of the robot according to the previous position
        and the odometry measurements. It also updates the uncertainty of the
        position
        '''
        #TODO: Program this function
        # - Update self.x_B_1 and self.P_B_1 using uk and self.Qk
             
        # Compound robot with odometry
        
        # Compute jacobians of the composition with respect to robot (A_k) 
        # and odometry (W_k)
        
        # Prepare the F_k and G_k matrix for the new uncertainty computation

        # Compute uncertainty
        
        # Update the class variables

    #========================================================================
        
    def data_association(self, lines):
        '''
        Implements ICCN for each feature of the scan.
        Innovk_List -> matrix containing the innovation for each associated 
                       feature
        H_k_List -> matrix of the jacobians.
        S_f_List -> matrix of the different S_f for each feature associated
        Rk_List -> matrix of the noise of each measurement
        '''
    
        #TODO: Program this function
        # fore each sensed line do:
        #   1- Transform the sened line to polar
        #   2- for each feature of the map (in the state vector) compute the 
        #      mahalanobis distance
        #   3- Data association
        
        # Init variable
        Innovk_List   = np.zeros((0,0))
        H_k_List      = np.zeros((0,0))
        S_f_List      = np.zeros((0,0))
        Rk_List       = np.zeros((0,0))
        idx_not_associated = np.zeros((0,1))
                
        return Innovk_List, H_k_List, S_f_List, Rk_List, idx_not_associated
        
    #========================================================================
    def update_position(self, Innovk_List, H_k_List, S_f_List , Rk_List) :
        '''
        Updates the position of the robot according to the given the position
        and the data association parameters.
        Returns state vector and uncertainty.
        
        '''
        #TODO: Program this function
        if Innovk_List.shape[0]<1:
            return
            
        # Kalman Gain
        
        # Update Position
        
        # Update Uncertainty
            
    #========================================================================
    def state_augmentation(self, lines, idx):
        '''
        given the whole set of lines read by the kineckt sensor and the
        indexes that have not been associated augment the state vector to 
        introduce the new features
        '''
        # If no features to add to the map exit function
        if idx.size<1:
            return
        
        # TODO Program this function

    #========================================================================
    def tfPolarLine(self,tf,line):
        '''
        Transforms a polar line in the robot frame to a polar line in the
        world frame
        '''
        # Decompose transformation
        x_x = tf[0]
        x_y = tf[1]
        x_ang = tf[2]  
        
        # Compute the new phi
        phi = angle_wrap(line[1] + x_ang)
        
        # Auxiliar computations
        sqrt2 = x_x**2+x_y**2
        sqrt = np.sqrt(sqrt2)
        atan2 = np.arctan2(x_y,x_x)
        sin = np.sin(atan2 - phi)
        cos = np.cos(atan2 - phi)
        
        # Compute the new rho
        rho = line[0] + sqrt* cos           
        
        # Allocate jacobians
        H_tf = np.zeros((2,3))
        H_line = np.eye(2)

        # TODO: Evaluate jacobian respect to transformation
        
        # TODO: Evaluate jacobian respect to line
                
        return np.array([rho,phi]), H_tf, H_line
                
    #========================================================================
    def lineDist(self,z,idx):
        '''
        Given a line and an index of the state vector it computes the
        distance between both lines
        '''        
        # TODO program this function
                
        # Transform the map line into robot frame and compute jacobians
        h = []
        H_position = []
        H_line =[]
        
        # Allocate overall jacobian
        H = []
        
        # Concatenate position jacobians and place them into the position
        
        # Place the position of the jacobina with respec to the line in its
        # position
        
        # Calculate innovation
        v = []
        
        # Calculate innovation uncertainty
        S = []
  
        # Calculate mahalanobis distance
        D = []
        
        return D,v,h,H,S