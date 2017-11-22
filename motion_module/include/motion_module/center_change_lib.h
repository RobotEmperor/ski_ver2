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
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <stdio.h>

#include "robotis_math/robotis_math.h"

using namespace std;

namespace diana
{
  class CenterChange
	{
	public:
  	CenterChange();
  	~CenterChange();

  	void parseMotionData(std::string turn_type);
  	void calculateStepEndPointValue(double desired_value, double step_value);

  	double step_end_point_value[2][6];

  	private:
  	double right_end_point_value[2][6]; // rigth turn
  	double left_end_point_value[2][6]; // left turn
  	double middle_end_point_value[2][6]; // The number 2 pose




	};
}





#endif /* SKI_VER2_MOTION_MODULE_INCLUDE_MOTION_MODULE_CENTER_CHANGE_LIB_H_ */
