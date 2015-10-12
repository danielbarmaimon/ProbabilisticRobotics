#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import numpy as np
from functions import angle_wrap

#===============================================================================
class ParticleFilter(object):
    '''
    Class to hold the whole particle filter.
    
    p_wei: weights of particles in array of shape (N,)
    p_ang: angle in radians of each particle with respect of world axis, shape (N,)
    p_xy : position in the world frame of the particles, shape (2,N)
    '''
    
    #===========================================================================
    def __init__(self, room_map, num, odom_lin_sigma, odom_ang_sigma, 
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
        self.num = num
        self.odom_lin_sigma = odom_lin_sigma
        self.odom_ang_sigma = odom_ang_sigma
        self.meas_rng_noise = meas_rng_noise
        self.meas_ang_noise = meas_ang_noise
        
        # Map
        map_xmin = np.min(self.map[:, 0])
        map_xmax = np.max(self.map[:, 0])
        map_ymin = np.min(self.map[:, 1])
        map_ymax = np.max(self.map[:, 1])
        
        # Particle initialization
        self.p_wei = 1.0 / num * np.ones(num)
        self.p_ang = 2 * np.pi * np.random.rand(num)
        self.p_xy  = np.vstack(( map_xmin + (map_xmax - map_xmin) * np.random.rand(num),
                                 map_ymin + (map_ymax - map_ymin) * np.random.rand(num) ))
    
    #===========================================================================
    def predict(self, odom):
        '''
        Moves particles with the given odometry.
        odom: incremental odometry [delta_x delta_y delta_yaw] in the vehicle frame
        '''
        # TODO: code here!!
        # Add Gaussian noise to odometry measures
        lineNoise =self.odom_lin_sigma*np.random.randn(2,self.num)/15;
        angNoise =self.odom_ang_sigma*np.random.randn(self.num)/15;
        
        # Increment particle positions in correct frame
        for i in range (self.p_xy.shape[1]):
        	odom[0] = odom[0] + lineNoise[0,i]
        	odom[1] = odom[1] + lineNoise[1,i]
        	odom[2] = odom[2] + angNoise[i]
        	#print ('The odom is %f and %f and %f ' %(odom[0],odom[1],odom[2]))
        	self.p_xy[0, i] += np.cos(self.p_ang[i])*odom[0]-np.sin(self.p_ang[i])*odom[1]
        	self.p_xy[1, i] += np.sin(self.p_ang[i])*odom[0]+np.cos(self.p_ang[i])*odom[1]

        # Increment angle
        	self.p_ang[i] += odom[2]
        	self.p_ang[i] = angle_wrap(self.p_ang[i])        
    	#print(self.p_xy)
    #===========================================================================
    def weight(self, lines):
        '''
        Look for the lines seen from the robot and compare them to the given map.
        Lines expressed as [x1 y1 x2 y2].
        '''
        # TODO: code here!!
        # Constant values for all weightings
        val_rng = 1.0 / (self.meas_rng_noise * np.sqrt(2 * np.pi))
        val_ang = 1.0 / (self.meas_ang_noise * np.sqrt(2 * np.pi))
        wmax=0
        # Loop over particles
        for i in range(self.num):
            # Transform map lines to local frame and to [range theta]
            wmax=0
            odom = [self.p_xy[0, i], self.p_xy[1, i], self.p_ang[i]] # Parameters of odom for each particle
            room_r = np.zeros(self.map.shape[0])
            room_theta = np.zeros(self.map.shape[0])
            for j in range(self.map.shape[0]):
            	(room_r[j], room_theta[j]) = self.get_polar_line(self.map[j], odom)	# Transfomration of map lines for each particle
            # Transform measured lines to [range theta] and weight them
            measured_r = np.zeros(lines.shape[0]); 
            measured_theta = np.zeros(lines.shape[0]);
            #print ('Number of lines given by robot is  l = %d ' %(lines.shape[0])) 
            for j in range(lines.shape[0]):
                (measured_r[j], measured_theta[j]) = self.get_polar_line(lines[j], [0,0,0])
                #print ('Now is  r = %f and theta %f of the measurements for particle %f ' %(measured_r[j], measured_theta[j], i))
                # ... self.get_polar_line(...)
                #
                # Weight them
                #print ('The size of self.p_wei is %d ' %(self.p_wei.shape[0]))
                weights = np.zeros((lines.shape[0], self.map.shape[0]))
                line_weights = np.zeros(lines.shape[0])
                for z in range(self.map.shape[0]):
                	a = val_rng*np.exp(-((measured_r[j]-room_r[z])**2)/float((2*self.meas_rng_noise**2)))
                	b = val_ang*np.exp(-((measured_theta[j]-room_theta[z])**2)/float((2*self.meas_ang_noise**2)))
                	w = a*b
                	weights[j][z] = w
                	if (wmax<w):
                		wmax=w
                	# if (self.p_wei[i]<a):
                	# 	self.p_wei[i]=a
                	# #print ('The size of self.p_wei first %f ' %(self.p_wei[j]))
                	# if (self.p_wei[i]<b):
                	# 	self.p_wei[i]*=b
                	# #print ('The size of self.p_wei second %f ' %(self.p_wei[j]))
                
                #
                #
            self.p_wei[i] *= wmax
            print(self.p_wei[i])
            # OPTIONAL question


            # make sure segments correspond, if not put weight to zero
            #
            #
            
            # Take best weighting (best associated lines)
            #
        
        
		# Normalize weights
        self.p_wei /= np.sum(self.p_wei)
        #for i in range(self.num):
        	#print ('The weight for particule %f is %f ' %(i, self.p_wei[i]))    
        
        
    #===========================================================================
    def resample(self):
        '''
        Systematic resampling of the particles.
        '''
        # TODO: code here!!
        # Look for particles to replicate
        p_xy_temp = np.zeros([2,self.p_xy.shape[1]])	# Temporary array of weights
        p_ang_temp = np.zeros(self.p_xy.shape[1])
        r = np.random.uniform(0,1.0/500)		        # Representation of a particle in the space
        cumulativeProb = self.p_wei[0]					# Variable to check total probability of chosen particles
        i = 0 											# Counter of number of particles for each interval
        j = 0
        for j in range (500):
        	u = r + (j)*(1.0/500)
        	while u > cumulativeProb:
        		i += 1
        		cumulativeProb += self.p_wei[i]
        		#print ('Particle %d in interval %d ' %(j, i))
        	#j+=i
        	#print ('this is i %d' %(i))
        	p_xy_temp[:,j]=self.p_xy[:,i]
        	p_ang_temp[j]=self.p_ang[i]
   
        self.p_xy = p_xy_temp
        self.p_ang = p_ang_temp
        self.p_wei[:] = 1.0/self.num	
        
        #self.p_wei = 


        #numberOfTimes = [] 
        #position = 0
        #
        #for i in range(self.num):
        #	numberOfTimes.append(int(self.p_wei[i]*self.num))
        #	print('Number of times is %d ' %(numberOfTimes[i]))
        #
        #
        #
        #
        
        # Pick chosen particles
        #
        #
        # mean = self.get_mean_particle
        # self.p_xy = mean[1]
        # self.p_ang = mean[2]
        # self.p_wei = mean[0]
        #self.p_xy *= numberOfTimes
        #self.p_ang *= numberOfTimes
        #self.p_wei *= numberOfTimes
    
    #===========================================================================
    def get_mean_particle(self):
        '''
        Gets mean particle.
        '''
        # Weighted mean
        weig = np.vstack((self.p_wei, self.p_wei))
        mean = np.sum(self.p_xy * weig, axis=1) / np.sum(self.p_wei)
        
        ang = np.arctan2( np.sum(self.p_wei * np.sin(self.p_ang)) / np.sum(self.p_wei),
                          np.sum(self.p_wei * np.cos(self.p_ang)) / np.sum(self.p_wei) )
                          
        return np.array([mean[0], mean[1], ang])
        
    #===========================================================================
    def get_polar_line(self, line, odom):
        '''
        Transforms a line from [x1 y1 x2 y2] from the world frame to the
        vehicle frame using odomotrey [x y ang].
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
