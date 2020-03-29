# -*- coding: utf-8 -*-
"""
Created on Tue Mar 16 20:11:35 2020

@author: psubacz
"""
import rospy
import numpy as np
from physics_model import Flight_Model_1
from std_msgs.msg import String

class TrajectoryPlanner:
    '''
    Calculates mulitple trajectories using the physical properties of the launcher
    '''
    def __init__(self, first_run = True, balls_loaded = None, ori = 0, lnch_ori = 45):

        #------ ROS nodes,pubs, and subs
        #create a ros publisher
        if(first_run):
            rospy.init_node('EntertrainerTrajectoryPlanner')
            self.traj_pub = rospy.Publisher('planned_trajectories', String, queue_size=10)

        #------ Launcher Properties
        #Load the flight model 
        self.FM1 =Flight_Model_1()
        #Max velocity produces by the launcher
        self.max_vel = 15 #m/s
        #Max force produced by the launcher
        self.max_force =  3.33 #(newtons)
        #energy threshold not to exceed
        self.energy_threshold = 2500 #joules
        #launcher angle
        self.lnchr_angle = self.FM1.deg_2_rad(lnch_ori)
        #set a max range
        self.MAX_RANGE_X = 4.572 #m ->15ft
        self.MAX_RANGE_Y = 2.4384 #m -> 8ft
        #max timer to wait for trajectory response
        self.max_time = 1.00

        #------ Robot Properties
        self.ori_global = 0
        self.ori_local = self.ori_global

        #------ Tennis ball properties
        #tennis ball diameter (cm)
        self.tDiameter = 3.81
        #tennis ball mass (g)
        self.tMass = 49.8
        #Get balls loaded in the launcher
        if balls_loaded ==None:
            self.balls_loaded = 3
        else:
            self.balls_loaded = balls_loaded

    def get_proxy_handles(self):
        pass

    def create_trej_gens(self,vel):
        '''
        ruturns array of labels,generators,and array of time of flights

        creates 16 trajectors generators ranging from 0-90 at 5 degree intervals.
        creates a list of 18 time of flight
        '''

        trajs_list = [[],[],[]]

        for i in range(1,17):
            tof = self.FM1.time_of_flight(v_0 = vel, theta = 5*i,g =-9.81)
            if tof > 0:
                trajs_list[0].append(5*i)
                trajs_list[1].append(self.FM1.trajectory_seq(v_0 = vel,theta = 5*i))
                trajs_list[2].append(tof)     
        return  trajs_list
    
    def get_lnchr_angle(self):
        '''
        Needs to get the angle from teh ball launcher
        '''
        return self.lnchr_angle
    
    def get_dead_rkn_dist(self):
        '''
        placeholder function to get dead reckoning linear distance infront of 
        the robot
        '''
        launch_low = 0.01 #m 
        return np.random.uniform(launch_low,self.MAX_RANGE_X**2)
    
    def generate_trajs(self,g = -9.81,dist = 0):
        '''
        Generates a trajectory path from the currect location and orientation 
        to a target location and orientation.

        Returns bool, trajectory label
        '''
        #velocity to launch 
        vel = 0 
        # impact energy
        eng = 0
        #counter
        count = 0
        #calculate the minimum velocity needed to reach the distance.  
        vel = self.FM1.calc_lnch_vel(dist,self.lnchr_angle)
        #generate trajectories
        traj_gens = self.create_trej_gens(vel)
        #timer values
        tic = 0
        toc = 0

        #While attempting to get a trajectory
        while True:
            #pick the longest trajectory
            traj_index = np.argmax(traj_gens[2])
            traj_gen = traj_gens[1][traj_index]
            #generate the trajectory 
            data = np.zeros((1+len(np.arange(0.01,np.max(traj_gens[2]),0.01)),3))
            for i in range(0,int(np.max(traj_gens[2])*100)+1):
                data[i] = next(traj_gen)
            #convert to str
            data_str = str(data)
            #Send data to pcl module, wait 5 seconds for a response
            tic = rospy.get_time()
            #while elapsed time is less than the max time
            while toc-tic<self.max_time:
                #time elapsed
                toc = rospy.get_time()
                #publish the trajectory
                self.traj_pub.publish(data_str)
                #
                #placeholder function and logic
                pcl_check = None            
                
                #4. Create a point cloud in front of the robot
                    # point_cloud_map = self.rectify_map(self.get_YZ_map,self.get_XY_map())        
                #6. Trace trajectories along the path if 

                # if trajectory hit a object, check to see if it has hit the ground. 
                #     if it has not hit the ground, check to see how much energy/ force it hits and object with
                #         if energy is > energy threshold:
                #             invalidate the trajectory

            # 7. Look at the trajectories and pick the one with the medium distance to travel in the X direction

            #if the trajectory checker is good, break the loop 
            if (pcl_check == True):
                return True, traj_gens[0]
            elif len(traj_gens[2])<=1:
                return False, None
            else:
                traj_gens[0].pop(traj_index)
                traj_gens[1].pop(traj_index)
                traj_gens[2].pop(traj_index)

    def get_balls_loaded(self):
        '''

        '''
        balls_loaded = 3
        return balls_loaded

    def Traj_Controller(self,rate):
        '''
        Main controller for the trajectory planner
        '''
        r = rospy.Rate(rate)

        while True:
            if not rospy.is_shutdown():
                print('rosok')  
            
            #0. create map of room

            #1. check to make sure we have a ball to launch
            if self.balls_loaded == 0:
                print('Needs to load more balls!')
                continue
            
            #2. Generate random points on map to launch from
            x = np.random.uniform(0,30)
            y = np.random.uniform(0,30)
            print(x,y)

            #3. Navigate to the points

            #reset the local orientation
            self.ori_local = 0
            while self.ori_local<= 360:
                #Query a random range from the map or range finder
                d = self.get_dead_rkn_dist()
                #If the distance is greator than the max range, set the distance equal to max range
                if d>self.MAX_RANGE_X:
                    d = self.MAX_RANGE_X

                #
                good,angle = self.generate_trajs(dist = d)

                #if a trajectory is not found turn the bot
                if good is True:
                    # 8. publish command to ball launcher to set angle, impulse, and launch velocity
                    #set launcher angle and fire

                    # 9. decrement balls counter
                    self.balls_loaded -= 1
                    pass
                else:
                    #turn 30 degrees in the local frame
                    self.ori_local += 30

# import warnings
# warnings.filterwarnings("ignore")

TP = TrajectoryPlanner()
TP.Traj_Controller(rate = 10)