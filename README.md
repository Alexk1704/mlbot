# mlbot
TurtleBot2 implementation with ROS/PCL

IMPORTANT INFO!!! copy whole .gazebo/models/ folder into your gazebo installation folder usually located at ~/.gazebo/models to add custom models used in this simulation.

Also we need to tweak the .urdf.xacro files that will later get converted into the turtlebot model by gazebo, the directories are usually located at, overwrite these with files from src/

/opt/ros/kinetic/share/turtlebot_description/urdf/sensors/kinect.urdf.xacro
/opt/ros/kinetic/share/turtlebot_description/urdf/turtlebot_gazebo.urdf.xacro
