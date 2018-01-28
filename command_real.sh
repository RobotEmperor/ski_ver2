#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rosrun diana_sensors ft_node &

roslaunch microstrain_3dm_gx5_45 microstrain.launch &

roslaunch ski_main_manager ski_main_manager.launch &

