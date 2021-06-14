#!/bin/bash

SCRIPT_DIR=$(dirname $0)

xterm  -e  "roscore" & 
sleep 2

xterm  -e  "$SCRIPT_DIR/launch_world.sh" & 
sleep 2

xterm  -e  "$SCRIPT_DIR/launch_gmapping.sh" & 
sleep 1

xterm  -e  "$SCRIPT_DIR/launch_rviz.sh" & 
sleep 1

xterm  -e  "$SCRIPT_DIR/launch_teleop.sh"
