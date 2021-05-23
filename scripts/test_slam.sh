#!/bin/bash

xterm  -e  "roscore" & 
sleep 2

xterm  -e  "./launch_world.sh" & 
sleep 2

xterm  -e  "./launch_teleop.sh" & 
sleep 1

xterm  -e  "./launch_gmapping.sh" & 
sleep 1

xterm  -e  "./launch_rviz.sh"