/*
 * center_change_lib.h
 *
 *  Created on: Nov 21, 2017
 *      Author: robotemperor
 */

#ifndef SKI_VER2_MOTION_MODULE_INCLUDE_MOTION_MODULE_CENTER_CHANGE_LIB_H_
#define SKI_VER2_MOTION_MODULE_INCLUDE_MOTION_MODULE_CENTER_CHANGE_LIB_H_
#include <ros/ros.h>
#include <ros/package.h>

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <stdio.h>

#include "robotis_math/robotis_math.h"

using namespace std;

namespace diana_motion
{
  class CenterChange
	{
	public:
  	CenterChange();
  	~CenterChange();

  	void parseMotionData(std::string turn_type, std::string change_type);
  	void calculateStepEndPointValue(double desired_value, double step_value, std::string change_type);

  	double step_end_point_value[2][6];

  	private:
  	double right_end_point_value_center[2][6]; // rigth turn
  	double left_end_point_value_center[2][6]; // left turn
  	double middle_end_point_value_center[2][6]; // The number 2 pose


	};
}


#endif /* SKI_VER2_MOTION_MODULE_INCLUDE_MOTION_MODULE_CENTER_CHANGE_LIB_H_ */
