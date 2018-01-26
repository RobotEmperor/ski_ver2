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
	void initialize();
	void process();
	bool gazebo_check;

	double temp_flag0[3]; // 0 :x 1 : y 2 : z;
	double temp_flag1[3]; // 0 :x 1 : y 2 : z;
	double temp_flag2[3]; // 0 :x 1 : y 2 : z;
	double temp_flag3[3]; // 0 :x 1 : y 2 : z;

	double robot_position_on_flag_x;
	double robot_position_on_flag_y;

/*	double flag_0_x;
	double flag_0_y;

	double flag_1_x;
	double flag_1_y;

	double flag_2_x;
	double flag_2_y;

	double flag_3_x;
	double flag_3_y;*/

	bool is_moving_check;

	std::string turn_direction;

	void parseMotionData();// update command

	void headFollowFlag(double x , double y);
	//head follow flag
	double head_follow_flag_yaw_compensation;
	double pre_head_follow_flag_yaw_compensation;

	control_function::Filter *filter_head;

private:
	void point_on_graph(double first_flag[3], double second_flag[3]);
	void decision_function(double flag[3]);

	double left_x_detect_margin, left_y_detect_margin_min, left_y_detect_margin_max;
	double right_x_detect_margin, right_y_detect_margin_min, right_y_detect_margin_max;


	//head



};

}




#endif /* SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_DECISION_MODULE_STATE_H_ */
