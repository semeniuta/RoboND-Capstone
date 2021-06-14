# Home service robot project

## Overview

The project implements a simulated home service robot application based Turtlebot 2 platform and a Kinect sensor. The project demonstrates the following tasks:

 - SLAM with `gmapping`
 - Localization with AMCL
 - Navigation with the ROS navigation stack
 - Virtual markers in RViz to simulate object pickup, transport and dropoff

## Structure and dependencies

This repository constitutes a Catkin workspace's `src` directory. As such, before building and launching the nodes, run the workspace initialization command:

```bash
catkin_init_workspace
```

The main functionality resides in the ́`home_service_robot` package. The package containing the model of the world is `my_robot_world` (it is added as a Git submodule). The `dependencies` dicrectory stores a collection of Git submodules containing ROS packages related to the Turtlebot platform.

Directories `launch` and `param` of the `home_service_robot` package contain copied and adapted versions of launch- and param-files from various Turtlebot packages. The original file paths are noted in the comments in the beggining of each adapted file. 

## Launching

Custom launch scripts relying on `xterm` are located in the `scripts` directory:

 - `test_slam.sh`: nodes required for performing SLAM
 - `test_navigation.sh`: nodes required to test navigation with RViz and "2D Nav Goal" tool
 - `pick_objects.sh`: robot moves to the pickup zone, then to the dropoff zone
 - `add_marker.sh`: simple application showcasing virtual markers (alternating between the pickup and the dropoff locations)
 - `home_service.sh`: the full home service robot scenario (robot moves to the pickup locations, picks up the virtual marker, naviages to the dtopoff locations through a series of via-points, and, once drops off the virtual marker in the dropoff location)

## Principle of the pickup/dropoff of the virtual markers

The `pick_objects` node sends a series of `move_base_msgs::MoveBaseGoal` commands to the robot:

1. Pickup location
2. Via-point 1 (out of the door)
3. Via-point 2 (before swinging to another corridor)
4. Via-point 3 (before entering the smaller room)
5. Dropoff location

When navigating the robot, the `pick_objects` publishes state changes at the `/home_service_robot/robot_state` topic: 

```
moving_to_pickup -> at pickup -> moving_to_dropoff -> at_dropoff
```

The `sync_markers` node subscribes to the `/home_service_robot/robot_state` topic and publishes the corresponding `visualization_msgs::Marker` messages to simulated object pickup and dropoff in RViz. 

## Other notes

The notes below were made on the way of working on the project, and are not well-organized. 

```bash
rosdep -i install turtlebot_gazebo
```

```bash
catkin_make --cmake-args -DCMAKE_CXX_STANDARD=14
```

```bash
rospack depends turtlebot_gazebo 
```

```bash
rospack list | grep capstone_ws
rospack list | grep $(pwd)
```

To list all dependencies of `turtlebot_gazebo` that are located in the Git submodules of this project (to remove unused ROS packages if needed):

```bash
rospack depends turtlebot_gazebo | xargs -L1 rospack find | grep $(pwd) | sort
```

Results:

```
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
```

The same for turtlebot_teleop:

```
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
```

The same for turtlebot_rviz_launchers:

```
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
```

Denendencies (Git repos) on the first successful attempt to launch urtlebot_gazebo/turtlebot_world.launch:

```
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
```

https://answers.ros.org/question/287670/run-rviz-with-configuration-file-from-launch-file/

ROS packages called in launch scripts (src/script/launch_*.sh):

turtlebot_rviz_launchers (repo: turtlebot_interactions)
turtlebot_teleop (repo: turtlebot)
turtlebot_gazebo (repo: turtlebot_simulator)
gmapping (repo: slam_gmapping)

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