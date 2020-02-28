Use a catkin workspace:
	$ mkdir -p ~/catkin_ws/src
	$ cd ~/catkin_ws/
	$ catkin_make

Add the workspace to ROS environment, source generated setup file:
	$ . ~/catkin_ws/devel/setup.bash

Start ROS core:
	$ roscore

IMPORTANT INFO!!! copy whole .gazebo/models/ folder into your gazebo installation folder usually located at ~/.gazebo/models to add custom models used in this simulation.

Also we need to tweak the .urdf.xacro files that later get converted into the turtlebot model by gazebo, the directories are usually located at, overwrite these with files from src/

/opt/ros/kinetic/share/turtlebot_description/urdf/sensors/kinect.urdf.xacro
/opt/ros/kinetic/share/turtlebot_description/urdf/turtlebot_gazebo.urdf.xacro

Use custom world:
	$ export TURTLEBOT_GAZEBO_WORLD_FILE=~/catkin_ws/src/mlbot/mlbot.world

Launch TurtleBot2 Sim with Gazebo:
	$ roslaunch turtlebot_gazebo turtlebot_world.launch

Start RViz:
	$ roslaunch turtlebot_rviz_launchers view_robot.launch

Teleop:
	$ roslaunch turtlebot_teleop keyboard_teleop.launch

Start this node after build:
	$ rosrun mlbot mlbot_node
	