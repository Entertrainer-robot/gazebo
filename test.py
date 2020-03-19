#!/usr/bin/env python
import rospy
#from point_in_polygon import *
from gazebo_msgs.srv import GetWorldProperties, GetModelProperties, GetModelState
import matplotlib as mp
import matplotlib.pyplot as plt
from math import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_ros.gazebo_interface import spawn_sdf_model_client

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

entertrainer_name = "entertrainer"

class BasicEntertrainerController(object):

    def __init__(self, rate=0):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        print("Creating object Basic Controller")
        if(rate != 0):
            rospy.init_node('BasicEntertrainerController')

        self.get_world_properties = None
        self.get_model_properties = None
        self.get_model_state = None
        self.objects = {}
        if(rate != 0):
            update_timer = rospy.Timer(rospy.Duration(rate), self.basic_movement)

    def get_proxy_handles(self):
        if self.get_world_properties is None:
            try:
                # Handle for world properties update function
                rospy.wait_for_service('/gazebo/get_world_properties', timeout=2)
                self.get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            except rospy.ROSException:
                print('/gazebo/get_world_properties service is unavailable')
        if self.get_model_properties is None:
            try:
                # Handle for retrieving model properties
                rospy.wait_for_service('/gazebo/get_model_properties',timeout=0.1)
                self.get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
            except rospy.ROSException:
                print('/gazebo/get_model_properties service is unavailable')
        if self.get_model_state is None:
            try:
               # Handle for retrieving model properties
                rospy.wait_for_service('/gazebo/get_joint_properties',timeout=0.1)
                self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            except rospy.ROSException:
                print('/gazebo/get_model_properties service is unavailable')

    def basic_movement(self, event=None):
        self.get_proxy_handles()

        _position = None
        _twist = None
        try:
            msg_world = self.get_world_properties()
            if(msg_world.success):
                for model in msg_world.model_names:
                    if(model == entertrainer_name):
                        model_properties = self.get_model_properties(model)
                        if(model_properties.success):
                            model_state = self.get_model_state(model, "")
                            if(model_state.success):
                                _position = model_state.pose.position
                                _twist = model_state.pose.orientation
                                rospy.loginfo('Position ='+ str(_position.x) + str(_position.y) + str( _position.z))
        finally:
            pass

        twist = Twist()

        twist.linear.x = 0.22
        twist.angular.z = 0.22
        self._cmd_pub.publish(twist)

        rospy.loginfo('Twisting the robot!')




if __name__ == '__main__':
    try:
        print("Startin Program")
        b = BasicEntertrainerController(0.5)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
