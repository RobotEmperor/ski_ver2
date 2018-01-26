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
#include "diana_msgs/BalanceParam.h"
#include "diana_msgs/FlagDataArray.h"
#include "diana_msgs/BalanceParamWaist.h"
#include "diana_msgs/ForceTorque.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "diana_msgs/CenterChange.h"
#include "diana_msgs/DesiredPoseCommand.h"


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

	ros::Publisher  current_flag_position_pub;

	// Subscriber
	ros::Subscriber head_test;
	ros::Subscriber waist_test;
	ros::Subscriber get_imu_data_sub_;
	ros::Subscriber balance_param_waist_sub;
	ros::Subscriber head_balance_sub;
	ros::Subscriber flag_position_get_sub;

	ros::Subscriber desired_pose_all_sub;


	//current cop and reference cop from leg module
	ros::Subscriber cop_fz_sub;

	void desiredPoseWaistMsgCallbackTEST(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void desiredPoseHeadMsgCallbackTEST(const std_msgs::Float64MultiArray::ConstPtr& msg);
	void desiredPoseAllMsgCallback(const diana_msgs::DesiredPoseCommand::ConstPtr& msg);

	//sensor
	void imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg);
	//pid gain value for gyro
	void balanceParameterWaistMsgCallback(const diana_msgs::BalanceParamWaist::ConstPtr& msg);
	void headBalanceMsgCallback(const std_msgs::Bool::ConstPtr& msg);

	// current cop and reference cop from leg module
	void copFzMsgCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg);

	//flag position data get
	void flagPositionGetMsgCallback(const diana_msgs::FlagDataArray& msg);

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
	void quaternionToAngle(double q_w, double q_x, double q_y, double q_z);
	void updateBalanceGyroParameter();
	void headFollowFlag(double x , double y);
	double currentGyroX,currentGyroY,currentGyroZ;
	double currentGyroOrientationW, currentGyroOrientationX,currentGyroOrientationY,currentGyroOrientationZ;
	double tf_current_gyro_x, tf_current_gyro_y, tf_current_gyro_z;
	double tf_current_gyro_orientation_x, tf_current_gyro_orientation_y, tf_current_gyro_orientation_z;
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
	Eigen::MatrixXd tf_gyro_value;

	//flag position
	double flag[4][3];

	//
	double **flag_position;

	geometry_msgs::Vector3 current_flag_position1_msg;
	geometry_msgs::Vector3 current_flag_position2_msg;
	geometry_msgs::Vector3 current_flag_position3_msg;
	geometry_msgs::Vector3 current_flag_position4_msg;

	diana_msgs::FlagDataArray current_flag_position_msg;

	int flag_length;

	//head low pass filter variables
	control_function::Filter *filter_head;
	double temp_head_roll, temp_head_pitch, temp_head_yaw;
	double temp_pre_roll, temp_pre_pitch, temp_pre_yaw;

	// enable adjustment
	heroehs_math::FifthOrderTrajectory *head_balance_enable;
	double head_enable;
	double result_head_enable;
	double head_enable_time;



};

}
