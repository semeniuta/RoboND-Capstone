#!/bin/bash

#export TURTLEBOT_3D_SENSOR=kinect
#roslaunch turtlebot_gazebo gmapping_demo.launch

rosrun gmapping slam_gmapping _angularUpdate:=0.01 _linearUpdate:=0.02 _particles:=250 _map_update_interval:=5.0 _xmin:=-15 _ymin:=-15 _xmax:=15 _ymax:=15 _delta:=0.01