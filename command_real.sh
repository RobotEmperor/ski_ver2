#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

rosrun diana_sensors ft_node &

roslaunch microstrain_3dm_gx5_45 microstrain.launch &

roslaunch ski_main_manager ski_main_manager.launch &

sleep 4 

rosbag record -o /media/ski/sd64/bagfile/ /l_compensation_xyz /l_compensation_rpy /r_compensation_xyz /r_compensation_rpy /l_leg_point_xyz /r_leg_point_xyz /cop_fz /imu/data /diana/force_torque_data

