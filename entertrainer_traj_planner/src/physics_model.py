# -*- coding: utf-8 -*-
"""
Created on Tue Mar 14 21:10:40 2020

@author: psubacz
"""
import numpy as np

class Flight_Model_1:
    '''
    Flight model no air resistance
    '''
    def __init__(self):
        pass
    
    def deg_2_rad(self,deg):
        return (deg*np.pi)/180

    def time_of_flight(self,v_0,theta,g):
        '''
        Returns the time of flight for the whole journey (neglecting air resistance)
        '''
        return ((2*v_0*np.sin(theta))/(-1*g))
        
    def get_init_v_0ocity(self,x,y,theta,g):
        '''
        Calculates the initial v_0ocity needs to hit a target at X-Y location
        '''
        return np.sqrt(((x**2)*g*-1)/(((2*x*np.sin(theta))-(2*y*np.cos(theta)**2))))
    
    def max_projectile_hieght(self,v_0,theta, g):
        '''
        Returns the max projectile hieght it will reach
        '''
        return ((v_0**2*np.sin(theta)**2)/(-2*g))
    
    def max_projectile_dist(self,v_0,g,theta):
        '''
        Returns the max projectile dist it will reach
        '''
        return ((v_0**2)/(-1*g))*np.sin(2*theta)

    def calc_launch_impulse(self,dist,theta,t = 0.1, g = 9.81,m = 0.498):
        '''
        calculates impulse force of the launcher
        '''
        return (m*np.sqrt((g*dist)/(np.sin(2*theta))))/(t) 
         

    def cal_angle_reach(self,g,d,v_0):
        '''
        Returns the angle of reach required to trav_0 to trav_0 a distance in the X
        
        there are two solutions to this problem:
          theta = 0.5*srcsin(g*d/v^2)
              and
          theta = 90 - 0.5*srcsin(g*d/v^2)
            
        '''
        
        angle_1 = (0.5*np.arcsin((-1*g*d)/v_0**2))
        angle_2 = np.pi/2-(0.5*np.arcsin((-1*g*d)/v_0**2))            
            
        return [angle_1,angle_2]
    
    def trajectory_seq(self,v_0,theta,p_x=0,p_z=0,a_x=0,a_y=-9.81):
        #initial parameter
        time = 0
        while True:
            yield time, p_x + v_0*time*np.cos(theta) + a_x*time**2,p_z + v_0*time*np.sin(theta) + 0.5*a_y*time**2
                # np.array([time,
                #            v_0*np.cos(theta)+a_x*time,#horzontal speed
                #            v_0*np.sin(theta)+a_y*time,#vertical speed
                #            p_x + v_0*time*np.cos(theta) + a_x*time**2,#horzontal disp
                #            p_z + v_0*time*np.sin(theta) + 0.5*a_y*time**2])#vertical disp
            time+=0.01

    def calc_lnch_vel(self,x_f,theta, g = -9.81,x_i = 0):
        return np.sqrt((-1*g*(x_f-x_i))/(np.sin(2*theta)))

    def cal_impact_energy(self,vel,ball_mass,theta):
        return 0.5*ball_mass*vel**2*np.cos(theta)
            
            

