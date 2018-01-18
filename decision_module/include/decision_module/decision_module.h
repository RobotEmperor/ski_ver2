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


using namespace decision_module;

bool ready_check;

DecisionModule decision_process;

// publisher
ros::Publisher desired_pose_leg_pub;
ros::Publisher desired_pose_waist_pub;
ros::Publisher desired_pose_head_pub;
ros::Publisher desired_pose_arm_pub;
// subscriber


// msg
std_msgs::Float64MultiArray desired_pose_leg_msg; // desired_pose command msg
std_msgs::Float64MultiArray desired_pose_waist_msg; // desired_pose command msg
std_msgs::Float64MultiArray desired_pose_head_msg; // desired_pose command msg
std_msgs::Float64MultiArray desired_pose_arm_msg; // desired_pose command msg


void initialize();


void readyCheckMsgCallBack(const std_msgs::Bool::ConstPtr& msg);
void desiredCenterChangeMsgCallback(const diana_msgs::CenterChange::ConstPtr& msg);


void control_loop(const ros::TimerEvent&);
void desired_leg_pose_pflug();
void desired_leg_pose_carving();

double leg_xyz_ypr_l[6];
double leg_xyz_ypr_r[6];
double leg_trj_time;

double waist_roll;
double waist_yaw;
double waist_roll_time, waist_yaw_time;




CenterChangeLeg *center_change_leg;
CenterChangeWaist *center_change_waist;

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

CarvingChange *carving_motion;



#endif /* SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_DECISION_MODULE_H_ */
