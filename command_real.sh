#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roscore &

sleep 5

rosrun diana_sensors ft_node &

roslaunch microstrain_3dm_gx5_45 microstrain.launch&

cd

rosbag record -a &

sleep 4 

roslaunch ski_main_manager ski_main_manager.launch

