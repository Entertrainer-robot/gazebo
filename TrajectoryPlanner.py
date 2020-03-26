# -*- coding: utf-8 -*-
"""
Created on Tue Mar 16 20:11:35 2020

@author: psubacz
"""

#load env
#find direction 
#calucalate trajectory
#
import numpy as np
from collections import deque
from physics_model import Flight_Model_1


class Ball:
    '''
    ball class used to simulate a tennis ball
    '''
    def __init__(self):
        
        #tennis ball diameter (cm)
        self.tDiameter = 3.81
        #tennis ball mass (g)
        self.tMass = 49.8
        self.reset_ball()
        
    def reset_ball(self):
        self.x = 0
        self.y = 0
        self.z = 0        

class TrajectoryPlanner:
    '''
    
    '''
    def __init__(self, balls_loaded = None, ori = 0, lnch_ori = 45):
        
        if balls_loaded ==None:
            self.balls_loaded = deque([Ball(),Ball(),Ball()],3)
        else:
            self.balls_loaded = balls_loaded
            
        #X-Y orienetation of the robot from map
        self.ori = ori
        
        #Load the flight model 
        self.FM1 =Flight_Model_1()
        
        #set a max range
        self.MAX_RANGE = 4.572 #m
        
        #
        self.energy_threshold = 2500 #joules
        
        self.lnchr_angle = self.FM1.deg_2_rad(lnch_ori)

        self.max_vel = 15

        #tennis ball diameter (cm)
        self.tDiameter = 3.81
        #tennis ball mass (g)
        self.tMass = 49.8
        
    def get_ball_loadout(self):
        return len(self.balls_loaded)
    
    def get_lnchr_angle(self):
        return self.lnchr_angle
    
    def calc_lnch_vel(self,d):
        '''
        placeholder function for spring calculation from distance to velocity
        
        waiting on dan/troy to finish mechanical calculations
        '''
        
        spring_low = 1 #m/s
        spring_high = 22 #m/s
        return np.random.uniform(spring_low,spring_high)
    
    def get_dead_rkn_dist(self):
        '''
        placeholder function to get dead reckoning linear distance infront of 
        the robot
        '''
        launch_low = 0.01 #m 
        return np.random.uniform(launch_low,self.MAX_RANGE**2)
    
    def generarte_trajectory(self,g = -9.81):
        '''
        Generates a trajectory path from the currect location and orientation 
        to a target location and orientation
        
        '''
        

        #Query a random range from the map or range finder
        d = self.get_dead_rkn_dist()
        vel = 0
        eng = 0
        count = 0
        # if eng >= self.energy_threshold:
        #     return False        
        while count<100:
            count+=1
            print('\ndistance',d,'\nvelocity',vel,'\nenergy',eng,'\nlaunch_angle',self.lnchr_angle)

            #If the distance is greator than the max range, set the distance equal to max range
            if d>self.MAX_RANGE:
                d = self.MAX_RANGE

            #calculate the minimum velocity needed to reach the distance.  
            vel = self.FM1.calc_lnch_vel(d,self.lnchr_angle)

            if vel > self.max_vel:
                d -=0.01
                continue
            #Create trajectory generator
            max_TOF = self.FM1.time_of_flight(vel,self.lnchr_angle,-9.81)
            t_gen = self.FM1.trajectory_seq(v_0 = vel,theta = self.lnchr_angle)
            traj = next(t_gen)
            # print(traj,max_TOF)

            while traj[0]<=max_TOF:
                traj = next(t_gen)
                print(traj[0],max_TOF)
                
            break
            eng =self.FM1.cal_impact_energy(vel,self.tMass)

            if eng >= self.energy_threshold:
                d -=0.01
                continue

            #calcalute the angle to possibly shoot from, Pick the angle that has 
            #   the least ammount of movement. Will return a failure if an angle 
            launch_angle = self.FM1.cal_angle_reach(g,d,vel)
            # launch_angle = [np.nan,np.nan]
            if not (np.isnan(launch_angle[0])and np.isnan(launch_angle[1])):
                #calculate impact velocity
                #pick the angle that has the shortest travel from the current angle
                self.lnchr_angle = launch_angle[0] if (np.abs(self.lnchr_angle-launch_angle[0])<np.abs(self.lnchr_angle-launch_angle[1])) else launch_angle[1]
        
                print('\ndistance',d,'\nvelocity',vel,'\nenergy',eng,'\nlaunch_angle',self.lnchr_angle)
                break
            else:
                d -=0.01
        

        # print('distance',d,'\nvelocity',vel,'\nenergy',eng,'\nlaunch_angle',self.lnchr_angle)
        
        #Create a trajectory generator
        self.FM1.trajectory_seq(v_0 = vel,theta = self.lnchr_angle)
        trajectoryPath = None     
        
        
        
        trajectoryPath = None
        return trajectoryPath


# import warnings
# warnings.filterwarnings("ignore")

TP = TrajectoryPlanner()
TP.generarte_trajectory()
