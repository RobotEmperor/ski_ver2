/*
 * decision_module.h
 *
 *  Created on: Jan 12, 2018
 *      Author: robotemperor
 */

#ifndef SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_DECISION_MODULE_H_
#define SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_DECISION_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdio.h>
#include "decision_module/decision_module_state.h"

//message
//m - standard
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>

//m - personal
#include "diana_msgs/CenterChange.h"
#include "diana_msgs/DesiredPoseCommand.h"
#include "diana_msgs/FlagDataArray.h"


using namespace decision_module;

bool ready_check;


// publisher
ros::Publisher desired_pose_leg_pub;
ros::Publisher desired_pose_waist_pub;
ros::Publisher desired_pose_head_pub;
ros::Publisher desired_pose_arm_pub;

ros::Publisher top_view_pub;
ros::Publisher top_view_robot_pub;

//pose publisher
ros::Publisher desired_pose_all_pub;

// subscriber


// msg
std_msgs::Float64MultiArray desired_pose_leg_msg; // desired_pose command msg
std_msgs::Float64MultiArray desired_pose_waist_msg; // desired_pose command msg
std_msgs::Float64MultiArray desired_pose_head_msg; // desired_pose command msg
std_msgs::Float64MultiArray desired_pose_arm_msg; // desired_pose command msg

//msg
diana_msgs::DesiredPoseCommand desired_pose_all_msg;

//msg
geometry_msgs::Vector3 top_view_msg;
geometry_msgs::Vector3 top_view_robot_msg;


void initialize();

// subscriber
void readyCheckMsgCallBack(const std_msgs::Bool::ConstPtr& msg);
void updateMsgCallBack(const std_msgs::Bool::ConstPtr& msg);
void desiredCenterChangeMsgCallback(const diana_msgs::CenterChange::ConstPtr& msg);
void currentFlagPositionMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg);
void modeChangeMsgCallback(const std_msgs::Bool::ConstPtr& msg);
void remoteTimeMsgCallBack(const std_msgs::Bool::ConstPtr& msg);

//control
void control_loop(const ros::TimerEvent&);
void motion_left(int motion_number);
void motion_right(int motion_number);
void motion_center(int motion_number);


std::string turn_type;
std::string change_type;

double change_value_center;
double time_center_change;
double time_edge_change;

std::string temp_turn_type;
std::string temp_change_type;

double temp_change_value_center;
double temp_time_center_change;
double temp_time_edge_change;

double motion_time_count;
bool center_change_moving_check;

//carving
double motion_time_count_carving;
int motion_seq;
int entire_motion_number_pflug;
int entire_motion_number_carving;
MotionChange *motion;

//decision module
DecisionModule *decision_algorithm;
std::string pre_command;

//remote or control
std::string mode;

//top_view_variables
double flag_position[5][2];
double robot_position[5][2];


//remote time cal
double remote_count_time;
int remote_count;
bool remote_update;
double remote_command[20][2];




#endif /* SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_DECISION_MODULE_H_ */
