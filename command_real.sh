#!/bin/bash

source /opt/ros/kinetic/setup.bash

sleep 1

source ~/catkin_ws/devel/setup.bash

sleep 1

rosbag record -o /media/ski/sd64/bagfile/ /l_compensation_xyz /l_compensation_rpy /r_compensation_xyz /r_compensation_rpy /l_leg_point_xyz /r_leg_point_xyz /cop_fz /imu/data /diana/force_torque_data /current_flag_position1 /current_flag_position2 /tf_gyro_value /top_view_robot /init_top_view /init_top_view /gate_switch /current_orientation_z /turn_command &

rosrun diana_sensors ft_node &

roslaunch microstrain_3dm_gx5_45 microstrain.launch &

roslaunch ski_main_manager ski_main_manager.launch 







