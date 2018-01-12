/*
 * decision_module.h
 *
 *  Created on: Jan 12, 2018
 *      Author: robotemperor
 */

#ifndef SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_DECISION_MODULE_H_
#define SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_DECISION_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdio.h>

//#include "robotis_framework_common/sensor_module.h"
//library
//#include "robotis_math/robotis_math.h"
//#include "heroehs_math/fifth_order_trajectory_generate.h"
//#include "heroehs_math/kinematics.h"
//#include "heroehs_math/end_point_to_rad_cal.h"
//#include "diana_balance_control/control_function.h"
//#include "diana_balance_control/diana_balance_control.h"
//#include "diana_balance_control/cop_calculation_function.h"

//message
//m - standard
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>

//m - personal
#include "diana_msgs/CenterChange.h"

void control_loop(const ros::TimerEvent&);


namespace decision_module
{
class DecisionModule
{
public:
	DecisionModule();
	virtual ~DecisionModule();

	/* ROS Functions */
	void initialize();
	void process();
	bool gazebo_check;

	// publisher
	ros::Publisher center_change_pub;



	// subscriber

	// msg
	diana_msgs::CenterChange center_change_msg;
private:


};

}



#endif /* SKI_VER2_DECISION_MODULE_INCLUDE_DECISION_MODULE_DECISION_MODULE_H_ */
