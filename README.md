# Home service robot project

## Overview

The project implements a simulated home service robot application based Turtlebot 2 platform and a Kinect sensor. 

## Structure and dependencies

## Launching

## Other notes

rosdep -i install turtlebot_gazebo

catkin_make --cmake-args -DCMAKE_CXX_STANDARD=14

rospack depends turtlebot_gazebo 

rospack list | grep capstone_ws
rospack list | grep $(pwd)

rospack depends turtlebot_gazebo | xargs -L1 rospack find | grep $(pwd) | sort

Results:

/home/alex/RoboND/capstone_ws/src/dependencies/kobuki_desktop/kobuki_gazebo_plugins
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_auto_docking
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_bumper2pc
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_capabilities
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_description
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_keyop
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_node
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_random_walker
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_rapps
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_safety_controller
/home/alex/RoboND/capstone_ws/src/dependencies/slam_gmapping/gmapping
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot_apps/turtlebot_navigation
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot_create/create_description
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot_create/create_driver
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot_create/create_node
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot/turtlebot_bringup
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot/turtlebot_capabilities
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot/turtlebot_description
/home/alex/RoboND/capstone_ws/src/dependencies/yujin_ocs/yocs_cmd_vel_mux
/home/alex/RoboND/capstone_ws/src/dependencies/yujin_ocs/yocs_controllers
/home/alex/RoboND/capstone_ws/src/dependencies/yujin_ocs/yocs_velocity_smoother

The same for turtlebot_teleop:

/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_auto_docking
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_bumper2pc
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_capabilities
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_description
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_keyop
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_node
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_random_walker
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_rapps
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_safety_controller
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot_create/create_description
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot_create/create_driver
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot_create/create_node
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot/turtlebot_bringup
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot/turtlebot_capabilities
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot/turtlebot_description
/home/alex/RoboND/capstone_ws/src/dependencies/yujin_ocs/yocs_cmd_vel_mux
/home/alex/RoboND/capstone_ws/src/dependencies/yujin_ocs/yocs_controllers
/home/alex/RoboND/capstone_ws/src/dependencies/yujin_ocs/yocs_velocity_smoother

The same for turtlebot_rviz_launchers:

/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_auto_docking
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_bumper2pc
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_capabilities
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_description
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_keyop
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_node
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_random_walker
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_rapps
/home/alex/RoboND/capstone_ws/src/dependencies/kobuki/kobuki_safety_controller
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot_create/create_description
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot_create/create_driver
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot_create/create_node
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot/turtlebot_bringup
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot/turtlebot_capabilities
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot/turtlebot_description
/home/alex/RoboND/capstone_ws/src/dependencies/turtlebot/turtlebot_teleop
/home/alex/RoboND/capstone_ws/src/dependencies/yujin_ocs/yocs_cmd_vel_mux
/home/alex/RoboND/capstone_ws/src/dependencies/yujin_ocs/yocs_controllers
/home/alex/RoboND/capstone_ws/src/dependencies/yujin_ocs/yocs_velocity_smoother

Denendencies (Git repos) on the first successful attempt to launch urtlebot_gazebo/turtlebot_world.launch:

kobuki
kobuki_desktop
slam_gmapping
turtlebot
turtlebot_apps
turtlebot_create
turtlebot_interactions
turtlebot_msgs
turtlebot_simulator
yocs_msgs
yujin_ocs (with removed yocs_ar_pair_tracking, yocs_ar_marker_tracking)

https://answers.ros.org/question/287670/run-rviz-with-configuration-file-from-launch-file/

ROS packages called in launch scripts (src/script/launch_*.sh):

turtlebot_rviz_launchers (repo: turtlebot_interactions)
turtlebot_teleop (repo: turtlebot)
turtlebot_gazebo (repo: turtlebot_simulator)
gmapping (repo: slam_gmapping)

Copied and adapted files:

turtlebot_interactions/turtlebot_rviz_launchers/launch/view_navigation.launch
turtlebot_interactions/turtlebot_rviz_launchers/rviz/navigation.rviz

Unused code with ROS parameters:

```xml
<param name="/home_service_robot/pickup_x" type="double" value="0.0" /> 
<param name="/home_service_robot/pickup_y" type="double" value="1.0" />
<param name="/home_service_robot/dropoff_x" type="double" value="6.0" />
<param name="/home_service_robot/dropoff_y" type="double" value="-2.0" />
```

```c++
double pickup_x, pickup_y, dropoff_x, dropoff_y;
this_node.getParam("/home_service_robot/pickup_x", pickup_x);
this_node.getParam("/home_service_robot/pickup_y", pickup_y);
this_node.getParam("/home_service_robot/dropoff_x", dropoff_x);
this_node.getParam("/home_service_robot/dropoff_y", dropoff_y);
```