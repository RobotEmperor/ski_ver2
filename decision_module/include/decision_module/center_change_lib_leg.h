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

	double motion_time[4];
	double motion_center[12];
	double motion_left_change[5][12];
	double motion_right_change[5][12];
	double motion_waist_left_change[5][2];
	double motion_waist_right_change[5][2];

	double motion_arm_left_change[5][6];
	double motion_arm_right_change[5][6];
private:

};
}

#endif /* SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_CENTER_CHANGE_LIB_LEG_H_ */
