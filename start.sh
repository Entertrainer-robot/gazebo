#!/usr/bin/env bash

export TURTLEBOT_3D_SENSOR=kinect
export TURTLEBOT3_MODEL=waffle

# start up the roslaunch
roslaunch entertrainer_test entertrainer_test.launch


# models for tv, sofa, dog.
# Pub # of balls, and in launcher.
# RVIZ markers
# -> Rectify points in xzy from local frame

# Gmapping find the edges
#
