#!/usr/bin/env python
from tf.transformations import quaternion_from_euler
import rospy
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from heapq import *
from nav_msgs.msg import GridCells, Path
from std_msgs.msg import *
import math
import tf
import copy

import numpy as np

class FrontierExplorer:
	def __init__(self):#constructor
		self.robot_x = 0
		self.robot_x = 0
		self.robot_theta = 0
		self.robotRadius = .135
		rospy.init_node('entertrainer_goal_planner')#initiate path finding node
		self._odom_list = tf.TransformListener()

		rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.updateNavigationStatus, queue_size=1) # handle nav goal events

		self.statusPub = rospy.Publisher('/navigation_status', Bool, queue_size=1) # handle nav goal events

		rospy.Subscriber('/navigation_goal', Point, self.updateGoal, queue_size=1) # handle nav goal events

		self.goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1) # handle nav goal events

		print "Listening for update"


	def updateNavigationStatus(self,status):


		goalReached = Bool()
		goalReached.data = True
		self.statusPub.publish(goalReached)
		print "published goal status when goal is reached" 



	def updateGoal(self,point):

		updated_point = Point(point.x,point.y,0)

		updated_quaternion = tf.transformations.quaternion_from_euler(0, 0, (self.robot_theta + point.z) % 2*math.pi)

		pStamped = PoseStamped()
		header = Header()
		header.stamp = rospy.Time.now()
		header.frame_id = 'map'
		pStamped.header = header
		pStamped.pose.orientation = Quaternion(0,0,0,1)
		pStamped.pose.position = updated_point

		self.goalPub.publish(pStamped)
		print "published goal point: " , pStamped.pose.position





if __name__ == "__main__":
    f = FrontierExplorer()

    # test_service = rospy.ServiceProxy('request_path',PathRequest)
    # Start = Point(10,10,0)
    # End = Point(5,5,0)
    # test_result = test_service(Start,End)
    # print(test_result)

    # rospy.wait_for_service('request_path')
    # rospy.spin()
    while  not rospy.is_shutdown():
        pass