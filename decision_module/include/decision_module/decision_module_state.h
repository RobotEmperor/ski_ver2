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

private:
	void desired_leg_motion();



};

}




#endif /* SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_DECISION_MODULE_STATE_H_ */
