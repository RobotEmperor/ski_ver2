/*
 * base_module.cpp
 *
 *  Created on: 2017. 10. 14.
 *      Author: robotemperor
 */


#define DegToRad M_PI/180
#define RadToDeg 180/M_PI


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <yaml-cpp/yaml.h>



/// Message file ///
#include "offset_module/dynamixel_info.h"
#include "offset_module/command.h"

#include <std_msgs/Int8.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
///////////////////

#include <vector>
#include <float.h> // nan check
#include <fstream> // log file
#include <iostream> // c++ output
#include <ctime>  // system time get
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h> // str -> int
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <sstream>

#include <boost/thread.hpp>
#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"

using namespace Eigen;
using namespace std;

namespace offset_module
{
class OffsetModule
		:public robotis_framework::MotionModule,
		 public robotis_framework::Singleton<OffsetModule>
{
private:
	int  control_cycle_msec_;
	boost::thread queue_thread_;
	void queueThread();

	int new_count_;

	bool running_;


  std::map<std::string, int> joint_name_to_id_;
  std::map<int, std::string> joint_id_to_name_;

	/// process variables ////

	// Data from GUI to offset module ////

	ros::Subscriber joint_select_sub_;  // joint number
	ros::Subscriber change_joint_value_sub_; // change value
	ros::Subscriber offset_joint_value_sub_; // change value
	ros::Subscriber save_onoff_sub_; // change value

	ros::ServiceServer read_joint_value_srv;



	// process variables ////
	bool offset_start_;
	bool save_onoff_;

	//offset variables each joint and all joint ////
	int8_t joint_select_;
  double change_joint_value_[30];
  double read_joint_value_[30];
  double offset_joint_value_[30];


public:
	OffsetModule();
	virtual ~OffsetModule();

	bool gazebo_check;

	void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
	void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

	void joint_select_sub_function(const std_msgs::Int8::ConstPtr& msg);
	void change_joint_value_sub_function(const std_msgs::Int16MultiArray::ConstPtr& msg);

	void offset_joint_value_sub_function(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void save_onoff_sub_function(const std_msgs::Bool::ConstPtr& msg);
	void parse_initial_offset_data();


	bool read_joint_value_srv_function(offset_module::command::Request  &req, offset_module::command::Response &res);

	void stop();
	bool isRunning();


};

}





