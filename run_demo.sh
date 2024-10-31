#!/bin/bash

# Source ROS environment if needed
# source /opt/ros/noetic/setup.bash
# source /catkin_ws/devel/setup.bash

# Start each process in the background
roslaunch arm_module_gazebo arm_module_ur5e_bringup.launch > /dev/null 2>&1 &
pid1=$!
roslaunch arm_module_gazebo depth_processing.launch always_pub_normals:=false > /dev/null 2>&1 &
pid2=$!
roslaunch arm_module_ur5e_moveit_config moveit_planning_execution.launch sim:=true > /dev/null 2>&1 &
pid3=$!
rosrun arm_module_ur5e_controller run_robot_controller.py > /dev/null 2>&1 &
pid4=$!
rosrun mapping_module mapping_module --config $(rospack find mapping_module)/cfg/dataset.json > /dev/null 2>&1 &
pid5=$!
roslaunch pointcloud_filter pointcloud_filter.launch &
pid6=$!

# Wait for all processes to finish
wait $pid1 $pid2 $pid3 $pid4 $pid5 $pid6
