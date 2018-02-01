/*
 * decision_module_state.h
 *
 *  Created on: Jan 13, 2018
 *      Author: robotemperor
 */

#ifndef SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_DECISION_MODULE_STATE_H_
#define SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_DECISION_MODULE_STATE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdio.h>

#include "decision_module/center_change_lib_leg.h"
#include "decision_module/center_change_lib_waist.h"
#include "diana_balance_control/control_function.h"



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

using namespace diana_motion_leg;
using namespace diana_motion_waist;

namespace decision_module
{
class DecisionModule
{
public:
	DecisionModule();
	~DecisionModule();

	/* ROS Functions */
	void initialize(double sampling_time, double update_time);
	void process();
	bool gazebo_check;

	double temp_flag0[3]; // 0 :x 1 : y 2 : z;
	double temp_flag1[3]; // 0 :x 1 : y 2 : z;


	bool is_moving_check;

	std::string turn_direction;

	void parseMotionData();// update command
	void headFollowFlag(double x , double y);

	//head follow flag
	double head_follow_flag_yaw_compensation;
	double pre_head_follow_flag_yaw_compensation;
	control_function::Filter *filter_head;

	geometry_msgs::Vector3 temp_top_view_flag_position;
	geometry_msgs::Vector3 top_view_flag_position;
	geometry_msgs::Vector3 pre_top_view_flag_position;
	geometry_msgs::Vector3 top_view_robot_position;
	geometry_msgs::Vector3 pre_top_view_robot_position;

	bool flag_check;
	int flag_sequence;
	int direction_command;

	//initialize function
	bool init_complete_check;
	double flag_in_data[5][2];
	double flag_out_data[5][2];
	std_msgs::Float64MultiArray init_top_view_msg;

private:
	void classification_function(double flag0[3], double flag1[3]);
	void decision_function(double flag0[3], double flag1[3]);

	//map
	void top_view(double flag_position[3]);
	int pre_flag_sequence;

	double left_x_detect_margin, left_y_detect_margin_min, left_y_detect_margin_max;
	double right_x_detect_margin, right_y_detect_margin_min, right_y_detect_margin_max;


	//initialize function
	double initialize_time_count;

	//classification function
	int flag_count;
	double pre_temp_flag_0[3], pre_temp_flag_1[3]; // head
	double true_flag_0[3], true_flag_1[3];  // head
	double final_in_flag[3];  // head

	double flag0_on_robot_top[3], flag1_on_robot_top[3]; //robot





};

}




#endif /* SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_DECISION_MODULE_STATE_H_ */
