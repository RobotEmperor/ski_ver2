/*
 * center_change_lib_leg.h
 *
 *  Created on: Jan 18, 2018
 *      Author: robotemperor
 */

#ifndef SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_CENTER_CHANGE_LIB_LEG_H_
#define SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_CENTER_CHANGE_LIB_LEG_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <stdio.h>


#include "robotis_math/robotis_math.h"

using namespace std;

namespace diana_motion_leg
{
class CenterChangeLeg
{
public:
	CenterChangeLeg();
	~CenterChangeLeg();

	void parseMotionData(std::string turn_type, std::string change_type);
	void calculateStepEndPointValue(double desired_value, double step_value, std::string change_type);

	double step_end_point_value[2][6];

private:
	double right_end_point_value_center[2][6]; // rigth turn
	double left_end_point_value_center[2][6]; // left turn
	double middle_end_point_value_center[2][6]; // The number 2 pose
};

class CarvingChange
{
public:
	CarvingChange();
	~CarvingChange();

	void parseMotionData();
	double calculate_velocity(double first_motion,double second_motion, double interval_time);
	double calculate_motion_velocity(double first_v1, double second_v2);
	void calculate_init_final_velocity(int motion_number);

	double motion_time[11];

	double motion_center_position[12];
	double motion_waist_center_position[12];
	double motion_arm_center_position[12];

	double motion_left_position[10][12];
	double motion_right_position[10][12];

	double motion_waist_left_position[10][2];
	double motion_waist_right_position[10][2];

	double motion_arm_left_position[10][6];
	double motion_arm_right_position[10][6];

	double motion_init_leg_left_vel[11][12];
	double motion_final_leg_left_vel[11][12];

	double motion_init_leg_right_vel[11][12];
	double motion_final_leg_right_vel[11][12];

	double motion_init_waist_left_vel[11][2];
	double motion_final_waist_left_vel[11][2];

	double motion_init_waist_right_vel[11][2];
	double motion_final_waist_right_vel[11][2];

	double motion_init_arm_left_vel[11][6];
	double motion_final_arm_left_vel[11][6];

	double motion_init_arm_right_vel[11][6];
	double motion_final_arm_right_vel[11][6];

private:
	double motion_left_vel[11][12];
	double motion_right_vel[11][12];

	double motion_waist_left_vel[11][2];
	double motion_waist_right_vel[11][2];

	double motion_arm_left_vel[11][6];
	double motion_arm_right_vel[11][6];

};
}

#endif /* SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_CENTER_CHANGE_LIB_LEG_H_ */
