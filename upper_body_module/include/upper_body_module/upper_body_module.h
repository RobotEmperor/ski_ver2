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
#include "diana_balance_control/control_function.h"
#include "diana_balance_control/diana_balance_control.h"
#include "diana_balance_control/cop_calculation_function.h"
#include "upper_body_module/center_change_upper_lib.h"

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
#include "diana_msgs/BalanceParamWaist.h"
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
	ros::Publisher  current_flag_position1_pub;
	ros::Publisher  current_flag_position2_pub;
	ros::Publisher  current_flag_position3_pub;
	ros::Publisher  current_flag_position4_pub;

	// Subscriber
	ros::Subscriber head_test;
	ros::Subscriber waist_test;
	ros::Subscriber get_imu_data_sub_;
	ros::Subscriber center_change_msg_sub;
	ros::Subscriber balance_param_waist_sub;

	//current cop and reference cop from leg module
	ros::Subscriber cop_fz_sub;


	//void currentLegPoseMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void desiredCenterChangeMsgCallback(const diana_msgs::CenterChange::ConstPtr& msg);
	void desiredPoseWaistMsgCallbackTEST(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void desiredPoseHeadMsgCallbackTEST(const std_msgs::Float64MultiArray::ConstPtr& msg);
	//sensor
	void imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg);
	//pid gain value for gyro
	void balanceParameterWaistMsgCallback(const diana_msgs::BalanceParamWaist::ConstPtr& msg);

	// current cop and reference cop from leg module
	void copFzMsgCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg);

	//flag position function
	void currentFlagPositionFunction(double x, double y, double z);


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

	double limitCheckHead(double calculated_value, double max, double min);
	//waist kinematics
	heroehs_math::KinematicsEulerAngle *waist_kinematics_;
	heroehs_math::CalRad *end_to_rad_waist_;     //
	Eigen::MatrixXd waist_end_point_;            // (6*8)
	Eigen::MatrixXd result_rad_waist_;           // (6*1)

	//head kinematics
	heroehs_math::KinematicsEulerAngle *head_kinematics_;
	heroehs_math::Kinematics *head_point_kinematics_;
	heroehs_math::CalRad *end_to_rad_head_;      //
	Eigen::MatrixXd head_end_point_;             // (6*8)
	Eigen::MatrixXd result_rad_head_;            // (6*1)

	//arm module data transmit
	std_msgs::Float64MultiArray current_waist_pose_msg;
	double temp_waist_yaw_rad, temp_waist_roll_rad;

	// cop compensation
	diana::CopCompensationFunc *cop_compensation_waist;
	heroehs_math::FifthOrderTrajectory *gain_copFz_p_adjustment;
	heroehs_math::FifthOrderTrajectory *gain_copFz_d_adjustment;
	double copFz_p_gain;
	double copFz_d_gain;

	double current_cop_fz_x, current_cop_fz_y;
	double reference_cop_fz_x, reference_cop_fz_y;

	// gyro
	void gyroRotationTransformation(double gyro_z, double gyro_y, double gyro_x);
	void updateBalanceGyroParameter();
	double currentGyroX,currentGyroY,currentGyroZ;
	double tf_current_gyro_x, tf_current_gyro_y, tf_current_gyro_z;
	heroehs_math::FifthOrderTrajectory *gain_roll_p_adjustment;
	heroehs_math::FifthOrderTrajectory *gain_roll_d_adjustment;
	heroehs_math::FifthOrderTrajectory *gain_yaw_p_adjustment;
	heroehs_math::FifthOrderTrajectory *gain_yaw_d_adjustment;
	control_function::PID_function *gyro_roll_function;
	control_function::PID_function *gyro_yaw_function;
	double updating_duration;
	double gyro_roll_p_gain;
	double gyro_roll_d_gain;
	double gyro_yaw_p_gain;
	double gyro_yaw_d_gain;


	//center change lib
	diana_motion_waist::CenterChange *center_change_;
	double temp_change_value_center;
	std::string temp_turn_type;
	std::string temp_change_type;

	//flag position
	double flag1_x, flag1_y, flag1_z;
	double flag2_x, flag2_y, flag2_z;
	double flag3_x, flag3_y, flag3_z;
	double flag4_x, flag4_y, flag4_z;
	geometry_msgs::Vector3 current_flag_position1_msg;
	geometry_msgs::Vector3 current_flag_position2_msg;
	geometry_msgs::Vector3 current_flag_position3_msg;
	geometry_msgs::Vector3 current_flag_position4_msg;


};

}
