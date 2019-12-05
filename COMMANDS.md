Use a catkin workspace:
	$ mkdir -p ~/catkin_ws/src
	$ cd ~/catkin_ws/
	$ catkin_make

Add the workspace to ROS environment, source generated setup file:
	$ . ~/catkin_ws/devel/setup.bash

Start ROS core:
	$ roscore

Use custom world:
	$ export TURTLEBOT_GAZEBO_WORLD_FILE=~/catkin_ws/src/mlbot/mlbot.world

Launch TurtleBot2 Sim with Gazebo:
	$ roslaunch turtlebot_gazebo turtlebot_world.launch

Start RViz:
	$ roslaunch turtlebot_rviz_launchers view_robot.launch

Start this node after build:
	$ rosrun mlbot mlbot_node



