#!/bin/bash

source /opt/ros/kinetic/setup.bash

sleep 1

source ~/catkin_ws/devel/setup.bash

sleep 1


rosrun diana_sensors ft_node &

roslaunch microstrain_3dm_gx5_45 microstrain.launch &

roslaunch ski_main_manager ski_main_manager.launch 







