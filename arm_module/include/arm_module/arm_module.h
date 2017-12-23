/*
 * arm_module.h
 *
 *  Created on: Dec 13, 2017
 *      Author: robotemperor
 */

#ifndef SKI_VER2_ARM_MODULE_INCLUDE_ARM_MODULE_ARM_MODULE_H_
#define SKI_VER2_ARM_MODULE_INCLUDE_ARM_MODULE_ARM_MODULE_H_

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
#include "heroehs_math/control_function.h"
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
#include "diana_msgs/BalanceParamArm.h"

namespace arm_module
{
class ArmModule: public robotis_framework::MotionModule, public robotis_framework::Singleton<ArmModule>
{
public:
	ArmModule();
	virtual ~ArmModule();

	/* ROS Framework Functions */
	void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
	void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

	void stop();
	bool isRunning();

	bool gazebo_check;

	double traj_time_test;




	// sensor data & balance on off
	ros::Subscriber desired_pose_arm_sub_;
	ros::Subscriber current_waist_pose_sub_;
	ros::Subscriber get_imu_data_sub_;
	ros::Subscriber get_ft_data_sub_;
	ros::Subscriber balance_param_arm_sub;



	void imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void balanceParameterArmMsgCallback(const diana_msgs::BalanceParamArm::ConstPtr& msg);

	void desiredPoseArmMsgCallbackTEST(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void currentWaistPoseMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

	//void ftDataMsgCallback(const diana_msgs::ForceTorque::ConstPtr& msg);

private:
	void queueThread();
	bool running_;
	int control_cycle_msec_;

	//int new_count_;

	boost::thread queue_thread_;

	std::map<std::string, int> joint_name_to_id_;
	std::map<int, std::string> joint_id_to_name_;

	int new_count_;
	bool is_moving_l_arm_;
	bool is_moving_r_arm_;



	heroehs_math::KinematicsArm *l_arm_kinematics_;
	heroehs_math::CalRad *end_to_rad_l_arm_;
	Eigen::MatrixXd l_arm_end_point_;
	Eigen::MatrixXd result_end_l_arm_;

	heroehs_math::KinematicsArm *r_arm_kinematics_;
	heroehs_math::CalRad *end_to_rad_r_arm_;
	Eigen::MatrixXd r_arm_end_point_;
	Eigen::MatrixXd result_end_r_arm_;

	// waist pose to transform
	double waist_yaw_rad_;
	double waist_roll_rad_;
	double l_arm_desired_point_x_, l_arm_desired_point_y_, l_arm_desired_point_z_;
	double r_arm_desired_point_x_, r_arm_desired_point_y_, r_arm_desired_point_z_;

  // gyro compensation
	void gyroRotationTransformation(double gyro_z, double gyro_y, double gyro_x);
	void updateBalanceGyroParameter();
	double currentGyroX,currentGyroY,currentGyroZ;
	double tf_current_gyro_x, tf_current_gyro_y, tf_current_gyro_z;

	heroehs_math::FifthOrderTrajectory *gain_roll_p_adjustment;
	heroehs_math::FifthOrderTrajectory *gain_roll_d_adjustment;
	heroehs_math::FifthOrderTrajectory *gain_pitch_p_adjustment;
	heroehs_math::FifthOrderTrajectory *gain_pitch_d_adjustment;
	heroehs_math::FifthOrderTrajectory *gain_yaw_p_adjustment;
	heroehs_math::FifthOrderTrajectory *gain_yaw_d_adjustment;

	control_function::PID_function *gyro_roll_function;
	control_function::PID_function *gyro_pitch_function;
	control_function::PID_function *gyro_yaw_function;

	double updating_duration;
	double gyro_roll_p_gain;
	double gyro_roll_d_gain;
	double gyro_pitch_p_gain;
	double gyro_pitch_d_gain;
	double gyro_yaw_p_gain;
	double gyro_yaw_d_gain;






};

}




#endif /* SKI_VER2_ARM_MODULE_INCLUDE_ARM_MODULE_ARM_MODULE_H_ */
