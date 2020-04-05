# -*- coding: utf-8 -*-
"""
Created on Tue Mar 16 20:11:35 2020

@author: psubacz
"""
import rospy, time, cv2, imutils
import numpy as np
from physics_model import Flight_Model_1
from std_msgs.msg import String, Header, Float64MultiArray, Float64, Bool, Int32
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseArray
import object_recognition as obj_rec

class TrajectoryPlanner:
    '''
    Calculates mulitple trajectories using the physical properties of the launcher
    '''
    def __init__(self, first_run = True, balls_loaded = None, ori = 0, lnch_ori = 45,wait_timer = 2):

        #------ ROS nodes,pubs, and subs
        #create a ros publisher
        if(first_run):
            rospy.init_node('EntertrainerTrajectoryPlanner')
            #publishers
            self.traj_pos_traj = rospy.Publisher('traj_pos_traj', String, queue_size=10)
            self.traj_arc_path = rospy.Publisher('traj_arc_path', PoseArray, queue_size=10)
            self.traj_lnchr_cmd = rospy.Publisher('traj_lnchr_cmd', Float64MultiArray, queue_size=10)
            self.traj_twist_cmd = rospy.Publisher('traj_twist_cmd', Float64, queue_size=10)
            self.traj_nav_pts = rospy.Publisher('traj_nav_pts', Point, queue_size=10)
            #subscribers
            self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.mapper_callback)
            self.lnchr_angle_sub = rospy.Subscriber("lnchr_angle_sub", Float64, self.lnchr_angle_callback)
            self.collision_check_sub = rospy.Subscriber("collision_check_sub", Bool, self.collision_check_callback)
            self.collision_time_sub = rospy.Subscriber("collision_time_sub", Int32, self.collision_time_callback)

        self.collision_time = None
        self.is_collision = False

        #------ Mapper Properties
        self.ogrid = None
        self.ogrid_origin = None
        
        #Need to wait here for the map to populate,
        print('Waiting {} seconds to complete start up...'.format(wait_timer))
        time.sleep(wait_timer)
        # self.high_x,self.high_y = self.ogrid.shape
        # self.high_x = 300
        # self.high_y = 300
        
        self.low_x,self.low_y = 0,0
        #------ Launcher Properties
        #Load the flight model 
        self.FM1 =Flight_Model_1()
        #Max velocity produces by the launcher
        self.max_vel = 15 #m/s
        #Max force produced by the launcher
        self.max_force =  3.33 #(newtons)
        #energy threshold not to exceed
        self.energy_threshold = 2500    #joules
        self.energy_calculated = 0      #joules

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

        objrec = obj_rec.Object_Recognition()


    def mapper_callback(self,msg):
            self.ogrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            self.ogrid_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])

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
    
    def lnchr_angle_callback(self,msg):
        '''
        
        '''
        self.lnchr_angle = float(msg)

    def collision_check_callback(self,msg):
        self.is_collision = msg
    
    def collision_time_callback(self,msg):
        self.collision_time = msg

    def dist_trace_from_map(self,x,y):
        '''
        placeholder function to get dead reckoning linear distance infront of 
        the robot
        '''
        dx = x
        dy = y
        ori = np.deg2rad(self.ori_local)
        counter = 0
        while counter < int(self.high_x+self.high_y) :
            dx+=int(np.round(np.cos(ori)*1))
            dy+=int(np.round(np.sin(ori)*1))
            #Check to stay within bounds
            if dx > self.high_x-1 or dy > self.high_y-1 or dx < 0 or dy < 0 or self.ogrid[dx][dy] >0:
                return np.sqrt((dx-x)**2+(dy-y)**2)*0.05
            else:
                counter+=1
        return False

    def pack_pose(self,state):
        '''
        copied from https://github.com/jnez71/lqRRT/blob/master/demos/lqrrt_ros/nodes/lqrrt_node.py
        '''
    
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = state
        return msg    

    def generate_trajs(self,g = -9.81,dist = 0):
        '''
        Generates a trajectory path from the currect location and orientation 
        to a target location and orientation.

        Returns bool, trajectory label
        '''
        #velocity to launch 
        vel = 0
        #timer values
        tic = 0
        toc = 0

        # impact energy
        self.energy_calculated = 0

        #calculate the minimum velocity needed to reach the distance.  
        vel = self.FM1.calc_lnch_vel(dist,self.lnchr_angle)

        #generate trajectories
        traj_gens = self.create_trej_gens(vel)

        #While attempting to get a trajectory/
        while True:
            #pick the longest trajectory
            traj_index = np.argmax(traj_gens[2])
            traj_gen = traj_gens[1][traj_index]
            # angle = traj_gens
            print(traj_gens[0][traj_index])
            break
            #generate the trajectory as a list of poses.
            pose_list = []
            for _ in range(0,int(np.max(traj_gens[2])*100)+1):
                pose_list.append(self.pack_pose(next(traj_gen)))

            tic = rospy.get_time() #Send data to pcl module, wait 5 seconds for a response
            pcl_check = False
            while toc-tic<self.max_time: #while elapsed time is less than the max time  
                #time elapsed
                toc = rospy.get_time()
                #publish the trajectory
                self.traj_arc_path.publish(PoseArray(header = Header(stamp=rospy.Time.now(),frame_id = ''),poses =pose_list))
                if self.is_collision: #there is a collision
                    pcl_check = True
                    
            #if the trajectory checker is good, break the loop 
            if (pcl_check != True):
                launch_impulse = self.FM1.calc_launch_impulse(dist = d,theta = angle)*0.1
                return [True, traj_gens[0][traj_index],vel,launch_impulse]

            elif pcl_check == True:
                if self.collision_time is not None:
                    launch_impulse = self.FM1.calc_launch_impulse(dist = d,theta = angle)*0.1
                    
                    #impact energy for the x direction only
                    impact_energy = self.FM1.cal_impact_energy(vel = vel,ball_mass = self.tMass)
                    '''
                    Neural network processing here

                    Get camera image

                    process into CNN
                    
                    Get output stream
                    
                    Establish min impact energy for each item
                    
                    '''
                    if launch_impulse < self.energy_threshold:
                        return [True, traj_gens[0][traj_index],vel,launch_impulse]
                        self.collision_time = None
                return [False, traj_gens[0][traj_index],vel,launch_impulse]

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
        rospy.Rate(rate)
        while True:
            if not rospy.is_shutdown():
                pass
                
            #1. check to make sure we have a ball to launch
            if self.balls_loaded == 0:
                print('Needs to load more balls!')
                break
            
            #2. Generate random points on map to launch from
            x = np.random.randint(self.low_x,self.high_x)
            y = np.random.randint(self.low_y,self.high_y)

            #3. Navigate to the points
            self.traj_nav_pts.publish(x = x, y= y)

            #reset the local orientation
            self.ori_local = self.ori_global
            
            turn_rbt = 45
            counter = 0
            #while we have not turned in a full circle
            while counter<= 360//turn_rbt:
                #Query a random range from the map or range finder
                d = self.dist_trace_from_map(x= x, y = y)
                
                #If the distance is greator than the max range, set the distance equal to max range
                if d>self.MAX_RANGE_X:
                    d = self.MAX_RANGE_X

                # [True, traj_gens[0][traj_index],vel,launch_impulse]
                good,angle,vel,launch_impulse = self.generate_trajs(dist = d)

                self.energy_calculated = self.FM1.calc_launch_impulse(dist = d,theta = angle)
                
                #if a trajectory is not found turn the bot
                if good is True:
                    # 8. publish command to ball launcher with the following:
                    # data = [launcher_angle, launch velocity, impulse force, launch_force]
                    self.traj_lnchr_cmd.publish(data = [float(angle),float(vel),float(self.energy_calculated),float(launch_impulse)])
                    # 9. decrement balls counter
                    self.balls_loaded -= 1
                    print('Balls Away!')
                    time.sleep(10)
                    break
                else:
                    #turn 45 degrees in the local frame
                    self.ori_local += turn_rbt 
                    #publish twist msg to nav
                    counter+=1
                self.traj_twist_cmd.publish(self.ori_local)

    def pack_float(self,state):
        msg = Float64MultiArray()
        return msg.data.append(state)

if __name__ == '__main__':
    TP = TrajectoryPlanner()
    TP.Traj_Controller(rate = 10)