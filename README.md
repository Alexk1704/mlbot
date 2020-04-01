# mlbot
TurtleBot2 implementation with ROS/PCL/OpenCV for simulating a child-like learning scenario.
TESTED AND WORKING ON: Ubuntu 16.04.6 LTS Xenial, using ROS kinetic 1.12.14, PCL 1.7, Boost 1.58.0.

First configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse.", then setup source lists:
    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

Setup your keys:
    $ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

Update package sources & run the ROS install:
    $ sudo apt-get update
    $ sudo apt-get install ros-kinetic-desktop-full

Automatically add ROS environment variables to your bash sessions whenever a new shell is launched:
    $ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc

Install tools and other dependencies for building ROS packages:
    $ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

Initialize rosdep:
    $ sudo apt install python-rosdep
    $ sudo rosdep init
    $ rosdep update

Wildcard install of additional dependencies for this simulation: 
    $ sudo apt-get install ros-kinetic-turtlebot* ros-kinetic-kobuki* ros-kinetic-gazebo*

!!! IMPORTANT INFO !!!: You need to copy the .gazebo/ folder located in this repository into your gazebo installation folder usually located at ~/.gazebo/ to add custom models need for this simulation. Also we need to replace some urdf files, important for the turtlebot model used in the gazebo simulator. These files can be found in the ROS installation path usually located at /opt/ros/kinetic/share/turtlebot_description/urdf/ replace the folder sensors/ with the custom one from ./custom/sensors/ and also replace the default file turtlebot_gazebo.urdf.xacro with the one from ./custom/

Creating a catkin workspace, copy the project wherever it is located into the src folder and build it:
    $ mkdir -p ~/catkin_ws/src
    $ cp -R ~/mlbot ~/catkin_ws/src
    $ catkin_make

In the same terminal, add the workspace to ROS environment by sourcing the generated setup file:
    $ . ~/catkin_ws/devel/setup.bash

In another terminal, start the roscore service:
	$ roscore

Use the custom world file for the gazebo simulation:
	$ export TURTLEBOT_GAZEBO_WORLD_FILE=~/catkin_ws/src/mlbot/custom mlbot.world

In another terminal, launch the turtlebot simulation with gazebo:
	$ roslaunch turtlebot_gazebo turtlebot_world.launch

Start this node after a successful build:
	$ rosrun mlbot mlbot_node

(Optional) In another terminal, start RViz for visualization:
	$ roslaunch turtlebot_rviz_launchers view_robot.launch

(Optional) In another terminal, if you want to manually control the robot launch teleop:
	$ roslaunch turtlebot_teleop keyboard_teleop.launch