/*
 * arm_module_state.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: robotemperor
 */

#include "arm_module/arm_module.h"
using namespace arm_module;

ArmModule::ArmModule()
: control_cycle_msec_(8)
{
	running_ = false;
	gazebo_check = false;
	is_moving_l_arm_ = false;
	is_moving_r_arm_ = false;

	enable_       = false;
	module_name_  = "arm_module";
	control_mode_ = robotis_framework::PositionControl;

	// Dynamixel initialize ////

	result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
	result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3
	result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5

/*
	result_["r_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 2
	result_["r_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 4
	result_["r_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 6
*/


	///////////////////////////
	// arm //
	// Left //
	l_arm_kinematics_  = new heroehs_math::KinematicsArm;
	end_to_rad_l_arm_  = new heroehs_math::CalRad;
	//Right //
	r_arm_kinematics_  = new heroehs_math::KinematicsArm;
	end_to_rad_r_arm_  = new heroehs_math::CalRad;

	waist_yaw_rad_  = 0;
	waist_roll_rad_ = 0;
	l_arm_desired_point_x_ = 0;
	l_arm_desired_point_y_ = 0;
	l_arm_desired_point_z_ = 0;

	r_arm_desired_point_x_ = 0;
	r_arm_desired_point_y_ = 0;
	r_arm_desired_point_z_ = 0;

	result_end_l_arm_.resize(6,1);
	result_end_l_arm_.fill(0);

	result_end_r_arm_.resize(6,1);
	result_end_r_arm_.fill(0);

	result_end_l_arm_.coeffRef(0,0) = 0;
	result_end_l_arm_.coeffRef(1,0) = 0;
	result_end_l_arm_.coeffRef(2,0) = -0.47;
	result_end_l_arm_.coeffRef(3,0) = 0;
	result_end_l_arm_.coeffRef(4,0) = 0;
	result_end_l_arm_.coeffRef(5,0) = 0;

	result_end_r_arm_.coeffRef(0,0) = 0;
	result_end_r_arm_.coeffRef(1,0) = 0;
	result_end_r_arm_.coeffRef(2,0) = -0.47;
	result_end_r_arm_.coeffRef(3,0) = 0;
	result_end_r_arm_.coeffRef(4,0) = 0;
	result_end_r_arm_.coeffRef(5,0) = 0;

	traj_time_test = 4;
	new_count_ = 1 ;
	// gyro compensation
	currentGyroX = 0;
	currentGyroY = 0;
	currentGyroZ = 0;
	tf_current_gyro_x = 0;
	tf_current_gyro_y = 0;
	tf_current_gyro_z = 0;
	gyro_roll_function      = new control_function::PID_function(0.008,5*DEGREE2RADIAN,-5*DEGREE2RADIAN,0,0,0);
	gyro_pitch_function     = new control_function::PID_function(0.008,5*DEGREE2RADIAN,-5*DEGREE2RADIAN,0,0,0);
	gyro_yaw_function       = new control_function::PID_function(0.008,5*DEGREE2RADIAN,-5*DEGREE2RADIAN,0,0,0);
	gain_roll_p_adjustment  = new heroehs_math::FifthOrderTrajectory;
	gain_roll_d_adjustment  = new heroehs_math::FifthOrderTrajectory;
	gain_pitch_p_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_pitch_d_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_yaw_p_adjustment   = new heroehs_math::FifthOrderTrajectory;
	gain_yaw_d_adjustment   = new heroehs_math::FifthOrderTrajectory;
	updating_duration = 0;
	gyro_roll_p_gain  = 0;
	gyro_roll_d_gain  = 0;
	gyro_pitch_p_gain = 0;
	gyro_pitch_d_gain = 0;
	gyro_yaw_p_gain   = 0;
	gyro_yaw_d_gain   = 0;
}
ArmModule::~ArmModule()
{
	queue_thread_.join();
}
void ArmModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;
	ros_node.setCallbackQueue(&callback_queue);

	// subscriber topics
	desired_pose_arm_sub_   = ros_node.subscribe("/desired_pose_arm", 5, &ArmModule::desiredPoseArmMsgCallbackTEST, this);
	current_waist_pose_sub_ = ros_node.subscribe("/current_waist_pose", 5, &ArmModule::currentWaistPoseMsgCallback, this);
	get_imu_data_sub_       = ros_node.subscribe("/imu/data", 100, &ArmModule::imuDataMsgCallback, this);
	balance_param_arm_sub   = ros_node.subscribe("/diana/balance_parameter_arm", 5, &ArmModule::balanceParameterArmMsgCallback, this);
	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);
}
void ArmModule::currentWaistPoseMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	waist_yaw_rad_  = msg->data[0];// waist yaw
	waist_roll_rad_ = msg->data[1];// waist roll
}
void ArmModule::desiredPoseArmMsgCallbackTEST(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	l_arm_end_point_(0, 1) = msg->data[0];// yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	l_arm_end_point_(1, 1) = msg->data[1];
	l_arm_end_point_(2, 1) = msg->data[2];
	r_arm_end_point_(0, 1) = msg->data[3];
	r_arm_end_point_(1, 1) = msg->data[4];
	r_arm_end_point_(2, 1) = msg->data[5];
	is_moving_l_arm_ = true;
	is_moving_r_arm_ = true;
}

// sensor data get///////////////////////////////////
void ArmModule::imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg) // gyro data get
{
	currentGyroX = (double) msg->angular_velocity.x;
	currentGyroY = (double) msg->angular_velocity.y;
	currentGyroZ = (double) msg->angular_velocity.z;
	gyroRotationTransformation(currentGyroZ, currentGyroY, currentGyroX);
}
void ArmModule::gyroRotationTransformation(double gyro_z, double gyro_y, double gyro_x)
{
	Eigen::MatrixXd tf_gyro_value;
	tf_gyro_value.resize(3,1);
	tf_gyro_value.fill(0);
	tf_gyro_value(0,0) =  gyro_x;
	tf_gyro_value(1,0) =  gyro_y;
	tf_gyro_value(2,0) =  gyro_z;
	tf_gyro_value = (robotis_framework::getRotationZ(M_PI/2)*robotis_framework::getRotationY(-M_PI))*tf_gyro_value;
	tf_current_gyro_x = tf_gyro_value(0,0);
	tf_current_gyro_y = tf_gyro_value(1,0);
	tf_current_gyro_z = tf_gyro_value(2,0);
}

//balance message for gyro///////////////////////////////////
void ArmModule::balanceParameterArmMsgCallback(const diana_msgs::BalanceParamArm::ConstPtr& msg)
{
	updating_duration  = msg -> updating_duration;
	gyro_roll_p_gain   = msg -> arm_roll_gyro_p_gain;
	gyro_roll_d_gain   = msg -> arm_roll_gyro_d_gain;
	gyro_pitch_p_gain  = msg -> arm_pitch_gyro_p_gain;
	gyro_pitch_d_gain  = msg -> arm_pitch_gyro_d_gain;
	gyro_yaw_p_gain    = msg -> arm_yaw_gyro_p_gain;
	gyro_yaw_d_gain    = msg -> arm_yaw_gyro_d_gain;
}


