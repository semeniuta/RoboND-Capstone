#!/bin/bash

xterm -e "roscore" & 
sleep 2

xterm -e "./launch_world.sh" & 
sleep 2

xterm -e "./launch_amcl.sh" &
sleep 2

xterm -e "./launch_rviz.sh" &
sleep 5

#xterm -e "rosrun home_service_robot pick_objects"
xterm -e "roslaunch home_service_robot pick_objects.launch"