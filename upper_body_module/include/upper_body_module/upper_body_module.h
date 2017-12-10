/*
 * upper_body_module.h
 *
 *  Created on: Dec 04, 2017
 *      Author: robotemperor
 */
#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdio.h>

#include "robotis_framework_common/motion_module.h"
//library
#include "robotis_math/robotis_math.h"
#include "heroehs_math/fifth_order_trajectory_generate.h"
#include "heroehs_math/kinematics.h"
#include "heroehs_math/end_point_to_rad_cal.h"
#include "diana_balance_control/diana_balance_control.h"
#include "diana_balance_control/cop_calculation_function.h"

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
#include "diana_msgs/BalanceParam.h"
#include "diana_msgs/ForceTorque.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "diana_msgs/CenterChange.h"


namespace upper_body_module
{

class UpperBodyModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<UpperBodyModule>
{
public:
	UpperBodyModule();
	virtual ~UpperBodyModule();

	/* ROS Framework Functions */
	void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
	void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

	void stop();
	bool isRunning();

	bool gazebo_check;

/*	// paper messages
	ros::Publisher state_end_point_pose_pub;
	ros::Publisher state_end_point_orientation_pub;
	ros::Publisher zmp_point_pub;
	ros::Publisher zmp_point_pub_temp;

	// sensor data & balance on off
	ros::Subscriber get_imu_data_sub_;
	ros::Subscriber get_ft_data_sub_;
  ros::Subscriber set_balance_param_sub_;*/

	/* ROS Topic Callback Functions */
/*	void desiredMotionMsgCallback(const std_msgs::Int32::ConstPtr& msg);
	void imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void ftDataMsgCallback(const diana_msgs::ForceTorque::ConstPtr& msg);
  void setBalanceParameterCallback(const diana_msgs::BalanceParam::ConstPtr& msg);
  void desiredCenterChangeMsgCallback(const diana_msgs::CenterChange::ConstPtr& msg);*/

private:
	void queueThread();
	bool running_;
	int control_cycle_msec_;

	//int new_count_;

	boost::thread queue_thread_;

	std::map<std::string, int> joint_name_to_id_;
	std::map<int, std::string> joint_id_to_name_;

	int new_count_;

};

}
