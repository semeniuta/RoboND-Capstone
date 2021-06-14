#!/bin/bash

SCRIPT_DIR=$(dirname $0)

xterm -e "roscore" & 
sleep 2

xterm -e "$SCRIPT_DIR/launch_world.sh" & 
sleep 2

xterm -e "$SCRIPT_DIR/launch_amcl.sh" &
sleep 2

xterm -e "$SCRIPT_DIR/launch_rviz.sh" &
sleep 5

xterm -e "roslaunch home_service_robot pick_objects.launch"