Developed with ros-melodic

Create a folder that is your catkin workspace, here we will use {catkin_ws}
We will paraphrase the following link http://wiki.ros.org/catkin/Tutorials/create_a_workspace
> mkdir -p {catkin_ws}/Service
> cd {catkin_ws}/src
> git clone https://github.com/Entertrainer-robot/gazebo.git
# ake sure to have the following repositories in your {catkin_ws}}/src
> git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
> git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
> git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
> git clone https://github.com/ros-perception/slam_gmapping.git
# uild the workspace
> cd {catkin_ws}
> catkin_make

# Now to run the simulation
> cd {catkin_ws}/src/gazebo
> ./start.sh





# Don't think we need this, but leaving this here for the time being.
git clone https://github.com/turtlebot/turtlebot_apps.git
