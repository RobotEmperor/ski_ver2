/*
 * center_change_lib_waist.h
 *
 *  Created on: Jan 18, 2018
 *      Author: robotemperor
 */

#ifndef SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_CENTER_CHANGE_LIB_WAIST_H_
#define SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_CENTER_CHANGE_LIB_WAIST_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <stdio.h>

#include "robotis_math/robotis_math.h"

using namespace std;

namespace diana_motion_waist
{
  class CenterChangeWaist
	{
	public:
	  CenterChangeWaist();
  	~CenterChangeWaist();

  	void parseMotionData(std::string turn_type, std::string change_type);
  	void calculateStepEndPointValue(double desired_value, double step_value, std::string change_type);

  	double step_end_point_value[2];

  	private:
  	double right_end_point_value_center[2]; // rigth turn
  	double left_end_point_value_center[2]; // left turn
  	double middle_end_point_value_center[2]; // The number 2 pose

	};
}



#endif /* SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_CENTER_CHANGE_LIB_WAIST_H_ */
