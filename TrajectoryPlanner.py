# -*- coding: utf-8 -*-
"""
Created on Tue Mar 16 20:11:35 2020

@author: psubacz
"""
import numpy as np
from collections import deque
from physics_model import Flight_Model_1
import rospy
#from point_in_polygon import *
from gazebo_msgs.srv import GetWorldProperties, GetModelProperties, GetModelState
# from tf.transformations import euler_from_quaternion, quaternion_from_eulesr
from gazebo_ros.gazebo_interface import spawn_sdf_model_client
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

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
    Plans mulitple trajectories within a XZ plane that ignores air resistance


    '''
    def __init__(self, first_run = True, balls_loaded = None, ori = 0, lnch_ori = 45):

        if(first_run):
            rospy.init_node('EntertrainerTrajectoryPlanner')
        
        if balls_loaded ==None:
            self.balls_loaded = 3
        else:
            self.balls_loaded = balls_loaded
            
        #X-Y orienetation of the robot from map
        self.ori = ori
        
        #Load the flight model 
        self.FM1 =Flight_Model_1()

        #set a max range
        self.MAX_RANGE_X = 4.572 #m ->15ft
        self.MAX_RANGE_Y = 2.4384 #m -> 8ft

        #MAX size we are working with
        self.MAX_X = 20
        self.MAX_Y = 20
        self.MIN_X = 0
        self.MIN_Y = 0
        
        #
        self.energy_threshold = 2500 #joules
        
        self.lnchr_angle = self.FM1.deg_2_rad(lnch_ori)

        self.max_vel = 15

        #tennis ball diameter (cm)
        self.tDiameter = 3.81
        #tennis ball mass (g)
        self.tMass = 49.8

    def get_proxy_handles(self):
        pass

    def create_trej_gens(self,vel):
        '''
        ruturns array of generators and array of time of flights

        creates 16 trajectors generators ranging from 0-90 at 5 degree intervals.
        creates a list of 18 time of flight
        '''
        trajs_list = []
        tof_list = []

        for i in range(1,17):
            trajs_list.append(self.FM1.trajectory_seq(v_0 = vel,theta = 5*i))
            tof_list.append(self.FM1.time_of_flight(v_0 = vel, theta = 5*i,g =-9.81))
        return  trajs_list,tof_list
    
    def get_lnchr_angle(self):
        '''
        Needs to get the angle from teh ball launcher
        '''
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
        return np.random.uniform(launch_low,self.MAX_RANGE_X**2)
    
    def generate_trajs(self,g = -9.81):
        '''
        Generates a trajectory path from the currect location and orientation 
        to a target location and orientation.
        '''
        vel = 0
        eng = 0
        count = 0
      
        while count<100:
            count+=1
            print('\ndistance',d,'\nvelocity',vel,'\nenergy',eng,'\nlaunch_angle',self.lnchr_angle)

            #calculate the minimum velocity needed to reach the distance.  
            vel = self.FM1.calc_lnch_vel(d,self.lnchr_angle)

            traj_gens,traj_tofs = self.create_trej_gens(vel)
            
            for traj in traj_gens:
                print(next(traj))
            print(traj_tofs)

            # if vel > self.max_vel:
            #     d -=0.01
            #     continue
            # #Create trajectory generator
            # max_TOF = self.FM1.time_of_flight(vel,self.lnchr_angle,-9.81)
            # t_gen = self.FM1.trajectory_seq(v_0 = vel,theta = self.lnchr_angle)
            # traj = next(t_gen)
            # print(traj,max_TOF)

            # while traj[0]<=max_TOF:
            #     traj = next(t_gen)
            #     print(traj[0],max_TOF)
                
            # break
            # eng =self.FM1.cal_impact_energy(vel,self.tMass)

            # if eng >= self.energy_threshold:
            #     d -=0.01
            #     continue

            # #calcalute the angle to possibly shoot from, Pick the angle that has 
            # #   the least ammount of movement. Will return a failure if an angle 
            # launch_angle = self.FM1.cal_angle_reach(g,d,vel)
            # # launch_angle = [np.nan,np.nan]
            # if not (np.isnan(launch_angle[0])and np.isnan(launch_angle[1])):
            #     #calculate impact velocity
            #     #pick the angle that has the shortest travel from the current angle
            #     self.lnchr_angle = launch_angle[0] if (np.abs(self.lnchr_angle-launch_angle[0])<np.abs(self.lnchr_angle-launch_angle[1])) else launch_angle[1]
        
            #     print('\ndistance',d,'\nvelocity',vel,'\nenergy',eng,'\nlaunch_angle',self.lnchr_angle)
            #     break
            # else:
            #     d -=0.01
        

        # print('distance',d,'\nvelocity',vel,'\nenergy',eng,'\nlaunch_angle',self.lnchr_angle)
        
        #Create a trajectory generator
        self.FM1.trajectory_seq(v_0 = vel,theta = self.lnchr_angle)        
        trajectoryPath = None
        return trajectoryPath

    def get_XY_map(self):
        '''
        get the XY map from the gmapping slam topic
        '''
        pass

    def get_YZ_map(self):
        '''
        gets the YZ map from the forward facing camera
        '''
        pass

    def rectify_map(self,yz_map,xy_map):
        '''
        rectifys the XY mao and YZ map to create a pseudo point cloud in front of the robot
        '''
        return None

    def check_map_traj_collis(self, coords1, coords2):
        '''
        Compares the coordinates of the trajectory with a point in the rectify_map...
        '''

        if coords1 == coords2:
            return False
        else: 
            return True
        pass

    def publish_trajectory(self):
        '''
        '''
        pass

    def Traj_Controller(self,rate):
        '''
        Main controller for the trajectory planner
        '''
        r = rospy.Rate(rate)
        point_cloud_map = None

        while True:
            if not rospy.is_shutdown():
                print('rosok')
                break  
            
            #1. check to make sure we have a ball to launch
            if self.balls_loaded == 0:
                print('Needs to load more balls!')
                continue
            
            #2. Generate random points on map to launch from
            x = np.random.uniform(self.MIN_X,self.MAX_X)
            y = np.random.uniform(self.MIN_Y,self.MAX_Y)

            #3. Navigate to the points
            

            #4. Create a point cloud in front of the robot
            point_cloud_map = self.rectify_map(self.get_YZ_map,self.get_XY_map())

            #Query a random range from the map or range finder
            d = self.get_dead_rkn_dist()

            #If the distance is greator than the max range, set the distance equal to max range
            if d>self.MAX_RANGE_X:
                d = self.MAX_RANGE_X

            #5. Create trajectories 
            self.generate_trajs()

            #6. Trace trajectories along the path if 

                # if trajectory hit a object, check to see if it has hit the ground. 
                #     if it has not hit the ground, check to see how much energy/ force it hits and object with
                #         if energy is > energy threshold:
                #             invalidate the trajectory

            # 7. Look at the trajectories and pick the one with the medium distance to travel in the X direction

            # 8. publish command to ball launcher to set angle and launch velocity

            # 9. decrement balls counter

# import warnings
# warnings.filterwarnings("ignore")

TP = TrajectoryPlanner()
TP.Traj_controller(rate = 10)
