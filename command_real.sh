#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roscore &

sleep 5

rosrun diana_sensor ft_node &

roslaunch microstrain_3dm_gx5_45 microstrain.launch&

cd /media/ski/sd64/bagfile &

rosbag record -a &

sleep 4 

echo -n "Wait yout command Motor torque enable (y/n)"

read input

if [ "$input" == "y" ];then
      roslaunch ski_main_manager ski_main_manager.launch 										
fi

