#!/bin/bash

export TURTLEBOT_GAZEBO_WORLD_FILE=$HOME/RoboND/catkin_ws/src/my_robot_world/worlds/my.world
export TURTLEBOT_BASE=kobuki
export TURTLEBOT_STACKS=hexagons
export TURTLEBOT_3D_SENSOR=kinect

#xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &
#sleep 5
#xterm  -e  "roslaunch turtlebot_gazebo gmapping_demo.launch"

roslaunch turtlebot_gazebo turtlebot_world.launch