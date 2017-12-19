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
	double traj_time_test;
  // publisher
	ros::Publisher  current_waist_pose_pub;
	ros::Publisher cop_point_Fz_pub;
	ros::Publisher cop_point_Fy_pub;
	ros::Publisher cop_point_Fx_pub;

	// Subscriber
	ros::Subscriber head_test;
	ros::Subscriber waist_test;
	ros::Subscriber current_leg_pose_sub;
	ros::Subscriber get_ft_data_sub_;
	ros::Subscriber get_imu_data_sub_;

	void desiredPoseWaistMsgCallbackTEST(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void desiredPoseHeadMsgCallbackTEST(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void currentLegPoseMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	//sensor
	void imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void ftDataMsgCallback(const diana_msgs::ForceTorque::ConstPtr& msg);

	geometry_msgs::PointStamped cop_point_Fz_msg_;
	geometry_msgs::PointStamped cop_point_Fy_msg_;
	geometry_msgs::PointStamped cop_point_Fx_msg_;

private:
	void queueThread();
	bool running_;
	int control_cycle_msec_;

	boost::thread queue_thread_;

	std::map<std::string, int> joint_name_to_id_;
	std::map<int, std::string> joint_id_to_name_;

	int new_count_;
	bool is_moving_head_;
	bool is_moving_waist_;


	//waist kinematics
	heroehs_math::KinematicsEulerAngle *waist_kinematics_;
	heroehs_math::CalRad *end_to_rad_waist_;     // (6*8)
	Eigen::MatrixXd waist_end_point_;            // (6*1)
	Eigen::MatrixXd result_rad_waist_;           // (6*1)

	//head kinematics
	heroehs_math::KinematicsEulerAngle *head_kinematics_;
	heroehs_math::CalRad *end_to_rad_head_;      // (6*8)
	Eigen::MatrixXd head_end_point_;             // (6*1)
	Eigen::MatrixXd result_rad_head_;            // (6*1)

  //arm module data transmit
	std_msgs::Float64MultiArray current_waist_pose_msg;
	double temp_waist_yaw_rad, temp_waist_roll_rad;

	// cop calculation
  diana::CopCalculationFunc *cop_cal_waist;
  double currentFX_l,currentFY_l,currentFZ_l,currentTX_l,currentTY_l,currentTZ_l;
  double currentFX_r,currentFY_r,currentFZ_r,currentTX_r,currentTY_r,currentTZ_r;
  Eigen::MatrixXd l_leg_real_joint;
  Eigen::MatrixXd r_leg_real_joint;

  // gyro
  void gyroRotationTransformation(double gyro_z, double gyro_y, double gyro_x);
  double currentGyroX,currentGyroY,currentGyroZ;
  double tf_current_gyro_x, tf_current_gyro_y, tf_current_gyro_z;



};

}
