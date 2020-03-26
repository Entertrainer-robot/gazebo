Developed with ros-melodic
Make sure to have the following packages installed
1) ros-melodic-openslam-gmapping
2) ros-melodic-turtle-actionlib
3) ros-melodic-turtle-tf
4) ros-melodic-turtle-tf2
5) ros-melodic-turtlesim

Create a folder that is your catkin workspace, here we will use ~/catkin_ws

We will paraphrase the following link http://wiki.ros.org/catkin/Tutorials/create_a_workspace

> mkdir -p ~/catkin_ws/src

> cd ~/catkin_ws/src

> git clone https://github.com/Entertrainer-robot/gazebo.git

# Make sure to have the following repositories in your {catkin_ws}}/src
> git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

> git clone https://github.com/ROBOTIS-GIT/turtlebot3.git

> git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

> git clone https://github.com/ros-perception/slam_gmapping.git

# Build the workspace
> cd ~/catkin_ws

> catkin_make

# Now to run the simulation
> cd ~/catkin_ws/src/gazebo
> ./start.sh





# Don't think we need this, but leaving this here for the time being.
git clone https://github.com/turtlebot/turtlebot_apps.git




# All ros packages installed in the developed environment...because I don't know what the dependencies are
ros-melodic-actionlib
ros-melodic-actionlib-msgs
ros-melodic-actionlib-tutorials
ros-melodic-amcl
ros-melodic-angles
ros-melodic-base-local-planner
ros-melodic-bond
ros-melodic-bond-core
ros-melodic-bondcpp
ros-melodic-bondpy
ros-melodic-camera-calibration
ros-melodic-camera-calibration-parsers
ros-melodic-camera-info-manager
ros-melodic-carrot-planner
ros-melodic-catkin
ros-melodic-class-loader
ros-melodic-clear-costmap-recovery
ros-melodic-cmake-modules
ros-melodic-common-msgs
ros-melodic-common-tutorials
ros-melodic-compressed-depth-image-transport
ros-melodic-compressed-image-transport
ros-melodic-control-msgs
ros-melodic-control-toolbox
ros-melodic-controller-interface
ros-melodic-controller-manager
ros-melodic-controller-manager-msgs
ros-melodic-costmap-2d
ros-melodic-cpp-common
ros-melodic-cv-bridge
ros-melodic-depth-image-proc
ros-melodic-depthimage-to-laserscan
ros-melodic-desktop
ros-melodic-desktop-full
ros-melodic-diagnostic-aggregator
ros-melodic-diagnostic-analysis
ros-melodic-diagnostic-common-diagnostics
ros-melodic-diagnostic-msgs
ros-melodic-diagnostic-updater
ros-melodic-diagnostics
ros-melodic-diff-drive-controller
ros-melodic-dwa-local-planner
ros-melodic-dynamic-reconfigure
ros-melodic-eigen-conversions
ros-melodic-executive-smach
ros-melodic-fake-localization
ros-melodic-filters
ros-melodic-forward-command-controller
ros-melodic-gazebo-dev
ros-melodic-gazebo-msgs
ros-melodic-gazebo-plugins
ros-melodic-gazebo-ros
ros-melodic-gazebo-ros-control
ros-melodic-gazebo-ros-pkgs
ros-melodic-gencpp
ros-melodic-geneus
ros-melodic-genlisp
ros-melodic-genmsg
ros-melodic-gennodejs
ros-melodic-genpy
ros-melodic-geometry
ros-melodic-geometry-msgs
ros-melodic-geometry-tutorials
ros-melodic-gl-dependency
ros-melodic-global-planner
ros-melodic-gmapping
ros-melodic-hardware-interface
ros-melodic-image-common
ros-melodic-image-geometry
ros-melodic-image-pipeline
ros-melodic-image-proc
ros-melodic-image-publisher
ros-melodic-image-rotate
ros-melodic-image-transport
ros-melodic-image-transport-plugins
ros-melodic-image-view
ros-melodic-interactive-marker-tutorials
ros-melodic-interactive-markers
ros-melodic-joint-limits-interface
ros-melodic-joint-state-controller
ros-melodic-joint-state-publisher
ros-melodic-joy
ros-melodic-kdl-conversions
ros-melodic-kdl-parser
ros-melodic-kdl-parser-py
ros-melodic-laser-assembler
ros-melodic-laser-filters
ros-melodic-laser-geometry
ros-melodic-laser-pipeline
ros-melodic-laser-proc
ros-melodic-librviz-tutorial
ros-melodic-map-msgs
ros-melodic-map-server
ros-melodic-media-export
ros-melodic-message-filters
ros-melodic-message-generation
ros-melodic-message-runtime
ros-melodic-mk
ros-melodic-move-base
ros-melodic-move-base-msgs
ros-melodic-move-slow-and-clear
ros-melodic-nav-core
ros-melodic-nav-msgs
ros-melodic-navfn
ros-melodic-navigation
ros-melodic-nodelet
ros-melodic-nodelet-core
ros-melodic-nodelet-topic-tools
ros-melodic-nodelet-tutorial-math
ros-melodic-openslam-gmapping
ros-melodic-orocos-kdl
ros-melodic-pcl-conversions
ros-melodic-pcl-msgs
ros-melodic-pcl-ros
ros-melodic-perception
ros-melodic-perception-pcl
ros-melodic-pluginlib
ros-melodic-pluginlib-tutorials
ros-melodic-polled-camera
ros-melodic-position-controllers
ros-melodic-python-orocos-kdl
ros-melodic-python-qt-binding
ros-melodic-qt-dotgraph
ros-melodic-qt-gui
ros-melodic-qt-gui-cpp
ros-melodic-qt-gui-py-common
ros-melodic-qwt-dependency
ros-melodic-realtime-tools
ros-melodic-resource-retriever
ros-melodic-rgbd-launch
ros-melodic-robot
ros-melodic-robot-state-publisher
ros-melodic-ros
ros-melodic-ros-base
ros-melodic-ros-comm
ros-melodic-ros-core
ros-melodic-ros-environment
ros-melodic-ros-tutorials
ros-melodic-rosbag
ros-melodic-rosbag-migration-rule
ros-melodic-rosbag-storage
ros-melodic-rosbash
ros-melodic-rosboost-cfg
ros-melodic-rosbuild
ros-melodic-rosclean
ros-melodic-rosconsole
ros-melodic-rosconsole-bridge
ros-melodic-roscpp
ros-melodic-roscpp-core
ros-melodic-roscpp-serialization
ros-melodic-roscpp-traits
ros-melodic-roscpp-tutorials
ros-melodic-roscreate
ros-melodic-rosgraph
ros-melodic-rosgraph-msgs
ros-melodic-roslang
ros-melodic-roslaunch
ros-melodic-roslib
ros-melodic-roslint
ros-melodic-roslisp
ros-melodic-roslz4
ros-melodic-rosmake
ros-melodic-rosmaster
ros-melodic-rosmsg
ros-melodic-rosnode
ros-melodic-rosout
ros-melodic-rospack
ros-melodic-rosparam
ros-melodic-rospy
ros-melodic-rospy-tutorials
ros-melodic-rosserial-arduino
ros-melodic-rosserial-client
ros-melodic-rosserial-msgs
ros-melodic-rosserial-python
ros-melodic-rosserial-server
ros-melodic-rosservice
ros-melodic-rostest
ros-melodic-rostime
ros-melodic-rostopic
ros-melodic-rosunit
ros-melodic-roswtf
ros-melodic-rotate-recovery
ros-melodic-rqt-action
ros-melodic-rqt-bag
ros-melodic-rqt-bag-plugins
ros-melodic-rqt-common-plugins
ros-melodic-rqt-console
ros-melodic-rqt-dep
ros-melodic-rqt-graph
ros-melodic-rqt-gui
ros-melodic-rqt-gui-cpp
ros-melodic-rqt-gui-py
ros-melodic-rqt-image-view
ros-melodic-rqt-launch
ros-melodic-rqt-logger-level
ros-melodic-rqt-moveit
ros-melodic-rqt-msg
ros-melodic-rqt-nav-view
ros-melodic-rqt-plot
ros-melodic-rqt-pose-view
ros-melodic-rqt-publisher
ros-melodic-rqt-py-common
ros-melodic-rqt-py-console
ros-melodic-rqt-reconfigure
ros-melodic-rqt-robot-dashboard
ros-melodic-rqt-robot-monitor
ros-melodic-rqt-robot-plugins
ros-melodic-rqt-robot-steering
ros-melodic-rqt-runtime-monitor
ros-melodic-rqt-rviz
ros-melodic-rqt-service-caller
ros-melodic-rqt-shell
ros-melodic-rqt-srv
ros-melodic-rqt-tf-tree
ros-melodic-rqt-top
ros-melodic-rqt-topic
ros-melodic-rqt-web
ros-melodic-rviz
ros-melodic-rviz-plugin-tutorials
ros-melodic-rviz-python-tutorial
ros-melodic-self-test
ros-melodic-sensor-msgs
ros-melodic-shape-msgs
ros-melodic-simulators
ros-melodic-smach
ros-melodic-smach-msgs
ros-melodic-smach-ros
ros-melodic-smclib
ros-melodic-stage
ros-melodic-stage-ros
ros-melodic-std-msgs
ros-melodic-std-srvs
ros-melodic-stereo-image-proc
ros-melodic-stereo-msgs
ros-melodic-teleop-twist-joy
ros-melodic-teleop-twist-keyboard
ros-melodic-tf
ros-melodic-tf-conversions
ros-melodic-tf2
ros-melodic-tf2-eigen
ros-melodic-tf2-geometry-msgs
ros-melodic-tf2-kdl
ros-melodic-tf2-msgs
ros-melodic-tf2-py
ros-melodic-tf2-ros
ros-melodic-theora-image-transport
ros-melodic-topic-tools
ros-melodic-trajectory-msgs
ros-melodic-transmission-interface
ros-melodic-turtle-actionlib
ros-melodic-turtle-tf
ros-melodic-turtle-tf2
ros-melodic-turtlesim
ros-melodic-urdf
ros-melodic-urdf-parser-plugin
ros-melodic-urdf-sim-tutorial
ros-melodic-urdf-tutorial
ros-melodic-urdfdom-py
ros-melodic-vision-opencv
ros-melodic-visualization-marker-tutorials
ros-melodic-visualization-msgs
ros-melodic-visualization-tutorials
ros-melodic-viz
ros-melodic-voxel-grid
ros-melodic-webkit-dependency
ros-melodic-xacro
ros-melodic-xmlrpcpp
