#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import *
import numpy as np
from functions import angle_wrap, comp
import ipdb

#===============================================================================
class EKF(object):
    '''
    Class to hold the whole EKF.
    '''
    
    #===========================================================================
    def __init__(self, room_map, x0, y0, theta0, odom_lin_sigma, odom_ang_sigma, 
                 meas_rng_noise, meas_ang_noise):
        '''
        Initializes the particle filter
        room_map : an array of lines in the form [x1 y1 x2 y2]
        num      : number of particles to use
        odom_lin_sigma: odometry linear noise
        odom_ang_sigma: odometry angular noise
        meas_rng_noise: measurement linear noise
        meas_ang_noise: measurement angular noise
        '''
        
        # Copy parameters
        self.map = room_map
        self.odom_lin_sigma = odom_lin_sigma
        self.odom_ang_sigma = odom_ang_sigma
        self.meas_rng_noise = meas_rng_noise
        self.meas_ang_noise = meas_ang_noise
        
        self.Qk = np.array([[ self.odom_lin_sigma**2, 0, 0],\
        [ 0, self.odom_lin_sigma**2, 0 ],\
        [ 0, 0, self.odom_ang_sigma**2]])
        
        self.Rk=np.eye(2)
        self.Rk[0,0] = self.meas_rng_noise
        self.Rk[1,1] = self.meas_ang_noise
        
        # Map
        map_xmin = np.min(self.map[:, 0])
        map_xmax = np.max(self.map[:, 0])
        map_ymin = np.min(self.map[:, 1])
        map_ymax = np.max(self.map[:, 1])
        
        # Pose initialization
        self.x_B_1 = np.array([x0,y0,theta0])        # (x,y,theta)
        self.P_B_1 = 1e-3*np.eye(3)             # Uncertainty of the postion of the robot
    
    #===========================================================================
    def predict(self, uk):
        '''
        Predicts the position of the robot according to the previous position and the odometry measurements. It also updates the uncertainty of the position
        '''
        #TODO: Program this function
        # 1- Update self.x_B_1 and self.P_B_1 using uk and self.Qk
     
    #===========================================================================
        
    def data_association(self, lines):
        '''
        Implements ICCN for each feature of the scan.
        Innovk_List -> nx2 matrix where each row is the innovation for each feature.
        H_k_List -> nx3 matrix, where every two rows define H_k for each feature.
        S_f_List -> nx3 matrix, where every 3 rows define S_f for each feature.
        Rk_List -> nx2 matrix, where each row is the measurement noise for each feature.
        '''
    
        #TODO: Program this function
        #   1- Transform map lines (self.map) to polar: use get_polar_line and current odom (self.x_B_1)
        #   2- Transform sensed lines (lines) to polar: use get_polar_line
        #   3- Data association
       
        # Init variable
        Innovk_List   = np.zeros((0,2))
        H_k_List      = np.zeros((0,3))
        S_f_List      = np.zeros((0,2))
        Rk_List       = np.zeros((0,2))
           
        return Innovk_List, H_k_List, S_f_List, Rk_List 
        
    #===========================================================================
    def update_position(self, Innovk_List, H_k_List, S_f_List , Rk_List) :
        '''
        Updates the position of the robot according to the given the position and the data association parameters.
        Returns state vector and uncertainty.
        
        '''
        #TODO: Program this function
        # Update self.x_B_1 and self.P_B_1

    #===========================================================================
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
        if dist > 0:
            ang = np.arctan2(line[1], line[0])
        else:
            ang = np.arctan2(-line[1], -line[0])
        
        # Return in the vehicle frame
        return np.array([np.abs(dist), angle_wrap(ang - odom[2])])
