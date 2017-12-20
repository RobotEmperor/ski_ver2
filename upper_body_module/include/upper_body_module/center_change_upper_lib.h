/*
 * center_change_upper_lib.h
 *
 *  Created on: Dec 20, 2017
 *      Author: robotemperor
 */

#ifndef SKI_VER2_UPPER_BODY_MODULE_INCLUDE_UPPER_BODY_MODULE_CENTER_CHANGE_UPPER_LIB_H_
#define SKI_VER2_UPPER_BODY_MODULE_INCLUDE_UPPER_BODY_MODULE_CENTER_CHANGE_UPPER_LIB_H_

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
  class CenterChange
	{
	public:
  	CenterChange();
  	~CenterChange();

  	void parseMotionData(std::string turn_type, std::string change_type);
  	void calculateStepEndPointValue(double desired_value, double step_value, std::string change_type);

  	double step_end_point_value[2];

  	private:
  	double right_end_point_value_center[2]; // rigth turn
  	double left_end_point_value_center[2]; // left turn
  	double middle_end_point_value_center[2]; // The number 2 pose

	};
}



#endif /* SKI_VER2_UPPER_BODY_MODULE_INCLUDE_UPPER_BODY_MODULE_CENTER_CHANGE_UPPER_LIB_H_ */
