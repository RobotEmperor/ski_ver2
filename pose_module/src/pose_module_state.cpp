/*
 * pose_module_state.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: robotemperor
 */
#include <stdio.h>
#include "pose_module/pose_module.h"

using namespace pose_module;

PoseModule::PoseModule()
: control_cycle_msec_(8)
{
	new_count_ = 0;
	running_ = false;
	gazebo_check = false;
	enable_       = false;
	module_name_  = "pose_module";
	control_mode_ = robotis_framework::PositionControl;

	// Dynamixel initialize ////




	result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
	result_["r_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 2
	result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3

	result_["r_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 4
	result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5
	result_["r_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 6

	result_["waist_yaw"]        = new robotis_framework::DynamixelState();  // joint 9
	result_["waist_roll"]       = new robotis_framework::DynamixelState();  // joint 10

	result_["l_hip_pitch"]      = new robotis_framework::DynamixelState();  // joint 11
	result_["l_hip_roll"]       = new robotis_framework::DynamixelState();  // joint 13
	result_["l_hip_yaw"]        = new robotis_framework::DynamixelState();  // joint 15
	result_["l_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 17
	result_["l_ankle_pitch"]    = new robotis_framework::DynamixelState();  // joint 19
	result_["l_ankle_roll"]     = new robotis_framework::DynamixelState();  // joint 21

	result_["r_hip_pitch"]      = new robotis_framework::DynamixelState();  // joint 12
	result_["r_hip_roll"]       = new robotis_framework::DynamixelState();  // joint 14
	result_["r_hip_yaw"]        = new robotis_framework::DynamixelState();  // joint 16
	result_["r_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 18
	result_["r_ankle_pitch"]    = new robotis_framework::DynamixelState();  // joint 20
	result_["r_ankle_roll"]     = new robotis_framework::DynamixelState();  // joint 22


	result_["head_yaw"]         = new robotis_framework::DynamixelState();  // joint 23
	result_["head_pitch"]       = new robotis_framework::DynamixelState();  // joint 24
	result_["head_roll"]        = new robotis_framework::DynamixelState();  // joint 25



	// TEST

/*
	result_["waist_yaw"]        = new robotis_framework::DynamixelState();  // joint 9
	result_["waist_roll"]       = new robotis_framework::DynamixelState();  // joint 10

	result_["head_yaw"]         = new robotis_framework::DynamixelState();  // joint 23

	result_["l_ankle_pitch"]    = new robotis_framework::DynamixelState();  // joint 19
	result_["r_ankle_pitch"]     = new robotis_framework::DynamixelState();  // joint 20

	result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
	result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3
	result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5

	result_["r_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 2
	result_["r_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 4
	result_["r_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 6
*/




	//leg ///////////////////////////
	l_kinematics_ = new heroehs_math::Kinematics;
	r_kinematics_ = new heroehs_math::Kinematics;
	end_to_rad_l_ = new heroehs_math::CalRad;
	end_to_rad_r_ = new heroehs_math::CalRad;
	is_moving_l_  = false;
	is_moving_r_  = false;
	// waist //
	waist_kinematics_ = new heroehs_math::KinematicsEulerAngle;
	end_to_rad_waist_ = new heroehs_math::CalRad;
	is_moving_waist   = false;

	// head //
	head_kinematics_  = new heroehs_math::KinematicsEulerAngle;
	end_to_rad_head_  = new heroehs_math::CalRad;
	is_moving_head    = false;
	// arm //
	// Left //
	l_arm_kinematics_  = new heroehs_math::KinematicsArm;
	end_to_rad_l_arm_  = new heroehs_math::CalRad;
	is_moving_l_arm    = false;
	//Right //
	r_arm_kinematics_  = new heroehs_math::KinematicsArm;
	end_to_rad_r_arm_  = new heroehs_math::CalRad;
	is_moving_r_arm    = false;

	l_arm_desired_point_x_ = 0;
	l_arm_desired_point_y_ = 0;
	l_arm_desired_point_z_ = 0;

	r_arm_desired_point_x_ = 0;
	r_arm_desired_point_y_ = 0;
	r_arm_desired_point_z_ = 0;

	traj_time_ = 4.0;
	////
	p_gain_adjust_check = false;
	id_select = 23;
	//cop calculate
	cop_cal = new diana::CopCalculationFunc;
	currentFX_l = 0;
	currentFY_l = 0;
	currentFZ_l = 0;
	currentTX_l = 0;
	currentTY_l = 0;
	currentTZ_l = 0;
	currentFX_r = 0;
	currentFY_r = 0;
	currentFZ_r = 0;
	currentTX_r = 0;
	currentTY_r = 0;
	currentTZ_r = 0;
}
PoseModule::~PoseModule()
{
	queue_thread_.join();
}

void PoseModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;

	ros_node.setCallbackQueue(&callback_queue);

	current_arm_state_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/current_arm_state",100);
	/* publisher topics */
	cop_point_Fz_pub = ros_node.advertise<geometry_msgs::PointStamped>("/cop_point_Fz",100);
	cop_point_Fy_pub = ros_node.advertise<geometry_msgs::PointStamped>("/cop_point_Fy",100);
	cop_point_Fx_pub = ros_node.advertise<geometry_msgs::PointStamped>("/cop_point_Fx",100);

	cop_fz_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/cop_fz",100);

	l_leg_point_xyz_pub = ros_node.advertise<geometry_msgs::Vector3>("/l_leg_point_xyz",100);
	l_leg_point_rpy_pub = ros_node.advertise<geometry_msgs::Vector3>("/l_leg_point_rpy",100);

	r_leg_point_xyz_pub = ros_node.advertise<geometry_msgs::Vector3>("/r_leg_point_xyz",100);
	r_leg_point_rpy_pub = ros_node.advertise<geometry_msgs::Vector3>("/r_leg_point_rpy",100);

	/* subscribe topics */
	// for gui
	ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/desired_pose_leg", 5, &PoseModule::desiredPoseMsgCallback, this);
	ros::Subscriber desired_pose_waist_sub = ros_node.subscribe("/desired_pose_waist", 5, &PoseModule::desiredPoseWaistMsgCallback, this);
	ros::Subscriber desired_pose_head_sub = ros_node.subscribe("/desired_pose_head", 5, &PoseModule::desiredPoseHeadMsgCallback, this);
	ros::Subscriber desired_pose_arm_sub = ros_node.subscribe("/desired_pose_arm", 5, &PoseModule::desiredPoseArmMsgCallback, this);

	ros::Subscriber gain_adjustment_sub = ros_node.subscribe("/gain_adjustment", 5, &PoseModule::gainAdjustmentMsgCallback, this);
	ros::Subscriber final_gain_save_sub = ros_node.subscribe("/final_gain_save", 5, &PoseModule::finalGainSaveMsgCallback, this);
	ros::Subscriber get_ft_data_sub_ = ros_node.subscribe("/diana/force_torque_data", 100, &PoseModule::ftDataMsgCallback, this);

	read_p_gain_value_srv = ros_node.advertiseService("/read_p_gain_value", &PoseModule::readPgainSrvFunction, this);

	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);
}

void PoseModule::desiredPoseMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) // GUI 에서 init pose topic을 sub 받아 실행
{
	for(int joint_num_= 0; joint_num_< 6; joint_num_++)
	{
		leg_end_point_l_(joint_num_, 1) = msg->data[joint_num_]; // left leg
		leg_end_point_r_(joint_num_, 1) = msg->data[joint_num_+6]; // right leg
	}
	is_moving_l_ = true;
	is_moving_r_ = true;
}
void PoseModule::desiredPoseWaistMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) // GUI 에서 init pose topic을 sub 받아 실행
{
	waist_end_point_(3, 1) = msg->data[0]; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	waist_end_point_(5, 1) = msg->data[1]; // roll
	is_moving_waist = true;
}
void PoseModule::desiredPoseHeadMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) // GUI 에서 init pose topic을 sub 받아 실행
{
	head_end_point_(3, 1) = msg->data[0]; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	head_end_point_(4, 1) = msg->data[1];
	head_end_point_(5, 1) = msg->data[2]; // roll
	is_moving_head = true;
}
void PoseModule::desiredPoseArmMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) // GUI 에서 init pose topic을 sub 받아 실행
{
	l_arm_end_point_(0, 1) = msg->data[0];// yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	l_arm_end_point_(1, 1) = msg->data[1];
	l_arm_end_point_(2, 1) = msg->data[2];
	r_arm_end_point_(0, 1) = msg->data[3];
	r_arm_end_point_(1, 1) = msg->data[4];
	r_arm_end_point_(2, 1) = msg->data[5];

	is_moving_l_arm = true;
	is_moving_r_arm = true;
}
void PoseModule::gainAdjustmentMsgCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) // GUI 에서 init pose topic을 sub 받아 실행
{
	id_select = msg->data[0];
	p_gain_data_[id_select] = msg->data[1];
	p_gain_adjust_check = true;
}
void PoseModule::finalGainSaveMsgCallback(const std_msgs::Bool::ConstPtr& msg) // GUI 에서 init pose topic을 sub 받아 실행
{
	if(msg->data)
	{
		savePgainValue();
	}
	else
		return;
}
bool PoseModule::readPgainSrvFunction(pose_module::command::Request  &req, pose_module::command::Response &res)
{
	for(int id =1; id<30; id++)
	{
		res.dxl_state[id] = p_gain_data_[id]; // GUI 에서 요청한 모든 조인트의 p gain 값을 저장함 .
	}
	return true;
}
void PoseModule::ftDataMsgCallback(const diana_msgs::ForceTorque::ConstPtr& msg)// force torque sensor data get
{
	currentFX_l = (double) msg->force_x_raw_l;
	currentFY_l = (double) msg->force_y_raw_l;
	currentFZ_l = (double) msg->force_z_raw_l;

	currentTX_l = (double) msg->torque_x_raw_l;
	currentTY_l = (double) msg->torque_y_raw_l;
	currentTZ_l = (double) msg->torque_z_raw_l;
	currentFX_r = (double) msg->force_x_raw_r;
	currentFY_r = (double) msg->force_y_raw_r;
	currentFZ_r = (double) msg->force_z_raw_r;

	currentTX_r = (double) msg->torque_x_raw_r;
	currentTY_r = (double) msg->torque_y_raw_r;
	currentTZ_r = (double) msg->torque_z_raw_r;


	cop_cal->ftSensorDataLeftGet(currentFX_l, currentFY_l, currentFZ_l, currentTX_l, currentTY_l, currentTZ_l);
	cop_cal->ftSensorDataRightGet(currentFX_r, currentFY_r, currentFZ_r, currentTX_r, currentTY_r, currentTZ_r);

	cop_cal->jointStateGetForTransForm(l_kinematics_->joint_radian, r_kinematics_->joint_radian);
	cop_cal->copCalculationResult();
}




