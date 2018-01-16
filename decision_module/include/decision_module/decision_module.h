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
#include <ros/package.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdio.h>
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
#include "decision_module/decision_module_state.h"

using namespace decision_module;

bool ready_check;

DecisionModule decision_process;


// publisher
ros::Publisher center_change_pub;
// subscriber

// msg
diana_msgs::CenterChange center_change_msg;

void readyCheckMsgCallBack(const std_msgs::Bool::ConstPtr& msg);
void control_loop(const ros::TimerEvent&);



#endif /* SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_DECISION_MODULE_H_ */
