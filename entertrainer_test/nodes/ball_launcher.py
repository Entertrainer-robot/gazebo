#!/usr/bin/env python

import rospy
from math import *
#from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelState
from std_msgs.msg import String, Header, Float64MultiArray, Float64, Bool, Int32, Int32MultiArray
from gazebo_msgs.srv import GetWorldProperties, GetModelProperties, GetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist

from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench
from rotation import *


ball_name = "tennis_ball"
debug_ball = False
debug_launcher = False
class ball():
    def __init__(self, name):
        self.name = name
        self.pose = None
        self.launched = False
        self.name_for_force = name  + "::" + "ball_body"
        #self.name_for_force = "ball_body"

        self.ball_last_position = None

class BallLauncher():

    def __init__(self, rate=0):
        # Gazebo Modifiers
        self.apply_force = None
        self.get_model_state  = None
        self.set_model_state = None
        # Launcher Data
        self.launcher_angle = 45
        self.impulse_force = 0.7493
        self.cnt = 0
        self.balls = []
        self.balls.append(ball(ball_name+"1"))
        self.balls.append(ball(ball_name+"2"))
        self.balls.append(ball(ball_name+"3"))
        self.current_num_balls = len(self.balls)
        self.fake_tof = 0
        self.launch_ball = False
        self.in_launcher = True
        self.ball_last_position = None
        self.ball_current_position = None
        self.average_z = None
        self.average_z_num = 0
#        self.launcher_angle_pub = rospy.Publisher('lnchr_angle_sub', Float64, queue_size=10)
        self.launcher_status_pub = rospy.Publisher('lnchr_status_pub', Int32MultiArray, queue_size=10)

        self.launch_cmd = rospy.Subscriber("traj_lnchr_cmd", Float64, self.process_ball_launcher_command)

    def get_proxy_handles(self):

        if self.apply_force is None:
            try:
                rospy.wait_for_service('/gazebo/apply_body_wrench',timeout=0.1)
                self.apply_force = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
            except rospy.ROSException:
                print('/gazebo/apply_body_wrench service is unavailable')
        if self.get_model_state is None:
            try:
               # Handle for retrieving model properties
                rospy.wait_for_service('/gazebo/get_joint_properties',timeout=0.1)
                self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            except rospy.ROSException:
                print('/gazebo/get_model_properties service is unavailable')
        if self.set_model_state is None:
            try:
                rospy.wait_for_service('/gazebo/set_model_state',timeout=0.1)
                self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            except rospy.ROSException:
                print('/gazebo/set_model_state service is unavailable')


    def transform_robot_to_world_frame(self, robot_euler_angles, vector_value):
        # vector_value is x, y, z in robot frame
        #if(debug_launcher): print('Setting up the Launch Force', robot_pose)
        #if(debug_launcher): print('\tQuanternion =',robot_pose.x, robot_pose.y, robot_pose.z, robot_pose.w)
        txyz = robot_euler_angles #euler_from_quaternion(robot_pose)

        tx = txyz[0]
        ty = txyz[1]
        tz = txyz[2]
        if(debug_launcher): rospy.loginfo(('\tEuler =', tx, ty, tz))
        x = vector_value[0]
        y = vector_value[1]
        z = vector_value[2]
        if(debug_launcher): rospy.loginfo(('\tForce (robot) =', x, y, z))
        [x, y, z] = rxyz(tx, ty, tz, x, y, z)
        if(debug_launcher): rospy.loginfo(('\tForce (world) =', x, y, z))
        return [x, y, z]

    def activate_launcher(self, _ball, robot_euler_angles, force_n):
        #print(robot_twist)
        # Ball launcher angle
        angle_deg = self.launcher_angle
        angle_rad = radians(angle_deg)
        # Ball Launch relative to robot
        robot_x = force_n * cos(angle_rad)
        robot_z = force_n * sin(angle_rad)
        robot_y = force_n * 0
        # Ball Launch relative to Gazebo world
        ball_launch_vector = [robot_x, robot_y, robot_z]
        [x,y,z] = self.transform_robot_to_world_frame(robot_euler_angles, ball_launch_vector)

        wrench          = Wrench()
        wrench.force.x  = x
        wrench.force.y  = y
        wrench.force.z  = z
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0
        # You can also define the start time if necessary...
        #print("wrench = " + wrench)
        self.apply_force(body_name = _ball.name_for_force ,wrench = wrench, duration = rospy.Duration(0.1)) # try to apply that as an instance...? Looks like it defaults to frame step
        if(debug_launcher): rospy.loginfo('Ball force Applied')

    def process_ball_launcher_command(self, msg):
        a, v, e = msg.data
        self.launcher_angle = a
        #self.impulse_force = f # todo need the impulse force to be included.
        #self.launcher_angle_pub.publish(data = float(self.launcher_angle))
        self.launch_ball = True

    def update_cycle(self, robot_euler_angles, _pose):
        self.in_launcher = (True if self.current_num_balls > 0 else False)

        # smooth out z axis
        if(self.average_z == None):
            self.average_z = _pose.position.z
            self.average_z_num = 1
        else:
            self.average_z_num +=1
            self.average_z = ((self.average_z * self.average_z_num) + _pose.position.z) / self.average_z_num
            self.average_z_num = min(self.average_z_num, 10)


        if(debug_launcher): rospy.loginfo('Ball Launcher Values launch=' + str(self.launch_ball) + "\t Loaded=" + str(self.in_launcher) + "\t remaining=" + str(self.current_num_balls) + " average_z=" + str(self.average_z))
        self.cnt += 1

        if(self.launch_ball):
            if(self.in_launcher):
                if(debug_launcher): rospy.loginfo('\tLaunching Ball number=' + str(self.current_num_balls))
                ball_in_launcher = max((len(self.balls) - self.current_num_balls), 0)
                state_msg = ModelState()
                try:
                    #resp = self.set_model_state( state_msg )
                    #rospy.loginfo('Setting Ball Position ='+ str(state_msg.pose.position.x) +' '+ str(state_msg.pose.position.y) +' '+ str( state_msg.pose.position.z) + ' Repsonse = ' + str(resp))
                    #force_lbs = 0.7493 #7.2
                    force_lbs = self.impulse_force #
                    force_n = force_lbs * 4.44822 # neuton
                    self.activate_launcher(self.balls[ball_in_launcher], robot_euler_angles, force_n)
                    model_state = self.get_model_state(self.balls[ball_in_launcher].name, "")
                    self.launch_ball = False
                    if(model_state.success):
                        self.balls[ball_in_launcher].ball_last_position = _pose.position
                    self.balls[ball_in_launcher].launched = True
                        # Save off the ball location prior to Launch
                    self.fake_tof = 10
                finally:
                    pass

                self.in_launcher = False
                if(debug_launcher): rospy.loginfo(('Launching the Ball! Ball in Launcher = ' + str(self.in_launcher)))
                self.current_num_balls-=1
                self.cnt = 0

        offset = 1 # Just so the remaining balls don't drop and hit the launching ball
        if(self.fake_tof > 0):
            self.fake_tof -= 1
            offset = 2
        for b in range(len(self.balls)):
            if(not self.balls[b].launched):
                state_msg = ModelState()
                state_msg.model_name = self.balls[b].name
                state_msg.pose.position = _pose.position
                if(offset == 1):
                    #state_msg.pose.position.z = self.average_z + 0.15
                    state_msg.pose.position.z = 0.5 + 0.15

                else:
                    # Smoothing out data to reduce bounce
                    #state_msg.pose.position.z = self.average_z + (0.25  + (0.00125 * (offset - 2)))
                    state_msg.pose.position.z = 0.5 + (0.25  + (0.15 * (offset - 2)))
                offset += 1
                try:
                    resp = self.set_model_state( state_msg )
                    if(debug_ball): rospy.loginfo('Setting Ball Position ='+ str(state_msg.pose.position.x) +' '+ str(state_msg.pose.position.y) +' '+ str( state_msg.pose.position.z) + ' Repsonse = ' + str(resp))
                finally:
                    pass



#        if(debug_ball):
#            try:
#                model_state = self.get_model_state(ball_name, "")
#                if(model_state.success):
#                    _position = model_state.pose.position
#                    rospy.loginfo('-> Checked the Ball Position ='+ str(_position.x) +' '+ str(_position.y) +' '+ str( _position.z))
#            finally:
#                pass

        # Save off the ball location currently
#        model_state = self.get_model_state(ball_name, "")
#        try:
#            if(model_state.success):
#                self.ball_current_position = model_state.pose.position
#        finally:
#            pass

#        dx = self.ball_last_position.x - self.ball_current_position.x
#        dy = self.ball_last_position.y - self.ball_current_position.y
#        dz = self.ball_last_position.z - self.ball_current_position.z
#        distance = sqrt(dx**2 + dy**2 + dz**2)
#        rospy.loginfo('The ball Went ' + str(distance) + ' meters; dx=' + str(dx) + ' dy=' + str(dy) + ' dz=' + str(dz))


        if(self.cnt == 10):
            if(self.current_num_balls > 0):
                self.in_launcher = True
        elif(self.cnt == 300 and self.current_num_balls == 0):
            # Just reset
            self.current_num_balls = len(self.balls)
            self.cnt = 0
            self.in_launcher = True
            for b in range(len(self.balls)):
                self.balls[b].launched = False
        #if (self.cnt % 20 == 19 and self.current_num_balls > 0):
        #    self.launch_ball = True


        # Publish the state_msg
        #self.launcher_angle_pub.publish(data = float(self.launcher_angle))
        launcher_status_data = [int32(self.current_num_balls), int32(self.in_launcher)]
        self.launcher_status_pub.publish(data = launcher_status_data
