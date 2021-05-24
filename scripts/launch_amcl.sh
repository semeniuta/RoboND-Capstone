#!/bin/bash

export TURTLEBOT_GAZEBO_MAP_FILE=$HOME/RoboND/catkin_ws/src/amcl_localize/maps/my_world_map.yaml

roslaunch turtlebot_gazebo amcl_demo.launch