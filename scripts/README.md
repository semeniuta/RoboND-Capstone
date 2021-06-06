rosdep -i install turtlebot_gazebo

catkin_make --cmake-args -DCMAKE_CXX_STANDARD=14

rospack depends turtlebot_gazebo 

rospack list | grep capstone_ws
rospack list | grep $(pwd)

rospack depends turtlebot_gazebo | xargs -L1 rospack find | grep $(pwd)

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