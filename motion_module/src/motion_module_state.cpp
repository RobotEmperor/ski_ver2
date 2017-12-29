/*
 * motion_module_state.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: robotemperor
 */
#include <stdio.h>
#include "motion_module/motion_module.h"

using namespace motion_module;

MotionModule::MotionModule()
: control_cycle_msec_(8)
{
	running_ = false;
	gazebo_check = false;
	is_moving_l_ = false;
	is_moving_r_ = false;
	is_moving_one_joint_ = false;
	enable_       = false;
	module_name_  = "motion_module";
	control_mode_ = robotis_framework::PositionControl;

	new_count_ = 0;
	// Dynamixel initialize ////
	result_["l_hip_pitch"] = new robotis_framework::DynamixelState();  // joint 11
	result_["l_hip_roll"]  = new robotis_framework::DynamixelState();  // joint 13

	result_["l_hip_yaw"]   = new robotis_framework::DynamixelState();  // joint 15
	result_["l_knee_pitch"] = new robotis_framework::DynamixelState();  // joint 17
	result_["l_ankle_pitch"] = new robotis_framework::DynamixelState();  // joint 19
	result_["l_ankle_roll"]  = new robotis_framework::DynamixelState();  // joint 21

	result_["r_hip_pitch"] = new robotis_framework::DynamixelState();  // joint 12
	result_["r_hip_roll"]  = new robotis_framework::DynamixelState();  // joint 14
	result_["r_hip_yaw"]   = new robotis_framework::DynamixelState();  // joint 16
	result_["r_knee_pitch"] = new robotis_framework::DynamixelState();  // joint 18
	result_["r_ankle_pitch"] = new robotis_framework::DynamixelState();  // joint 20
	result_["r_ankle_roll"]  = new robotis_framework::DynamixelState();  // joint 22
	// test
	/*
		result_["l_ankle_pitch"] = new robotis_framework::DynamixelState();  // joint 19
		result_["r_ankle_pitch"] = new robotis_framework::DynamixelState();  // joint 20
	 */
	///////////////////////////
	l_kinematics_ = new heroehs_math::Kinematics;
	r_kinematics_ = new heroehs_math::Kinematics;
	end_to_rad_l_ = new heroehs_math::CalRad;
	end_to_rad_r_ = new heroehs_math::CalRad;

	//////////////////////////
	currentGyroX = 0;
	currentGyroY = 0;
	currentGyroZ = 0;
	//////////////////////////
	traj_time_ = 4.0;

	result_end_l_.resize(6,1);
	result_end_r_.resize(6,1);
	result_end_l_.fill(0);
	result_end_r_.fill(0);

	result_end_l_.coeffRef(0,0) = 0.1;
	result_end_l_.coeffRef(1,0) = 0.255;
	result_end_l_.coeffRef(2,0) = -0.51;
	result_end_l_.coeffRef(3,0) = -15*DEGREE2RADIAN;
	result_end_l_.coeffRef(4,0) = -10*DEGREE2RADIAN;
	result_end_l_.coeffRef(5,0) = 15*DEGREE2RADIAN;

	result_end_r_.coeffRef(0,0) = 0.1;
	result_end_r_.coeffRef(1,0) = -0.255;
	result_end_r_.coeffRef(2,0) = -0.51;
	result_end_r_.coeffRef(3,0) = 15*DEGREE2RADIAN;
	result_end_r_.coeffRef(4,0) = -10*DEGREE2RADIAN;
	result_end_r_.coeffRef(5,0) = -15*DEGREE2RADIAN;

	result_mat_cob_ = result_mat_cob_modified_ = Eigen::Matrix4d::Identity();

	result_mat_l_ = robotis_framework::getTransformationXYZRPY(result_end_l_.coeff(0,0), result_end_l_.coeff(1,0), result_end_l_.coeff(2,0),
			result_end_l_.coeff(3,0), result_end_l_.coeff(4,0), result_end_l_.coeff(5,0));
	result_mat_r_ = robotis_framework::getTransformationXYZRPY(result_end_r_.coeff(0,0), result_end_r_.coeff(1,0), result_end_r_.coeff(2,0),
			result_end_r_.coeff(3,0), result_end_r_.coeff(4,0), result_end_r_.coeff(5,0));

	result_mat_l_modified_ = result_mat_l_;
	result_mat_r_modified_ = result_mat_r_;

	balance_updating_duration_sec_ = 2.0;
	balance_updating_sys_time_sec_ = 2.0;
	balance_update_= false;
	tf_current_gyro_x = 0;
	tf_current_gyro_y = 0;
	tf_current_gyro_z = 0;

	center_change_ = new diana_motion::CenterChange;
	cop_cal = new  diana::CopCalculationFunc;
	temp_change_value_center = 0;
	temp_change_value_edge = 0;
	temp_turn_type = "basic";
	temp_change_type = "basic";
	currentFX_l=0.0;
	currentFY_l=0.0;
	currentFZ_l=0.0;
	currentTX_l=0.0;
	currentTY_l=0.0;
	currentTZ_l=0.0;
	currentFX_r=0.0;
	currentFY_r=0.0;
	currentFZ_r=0.0;
	currentTX_r=0.0;
	currentTY_r=0.0;
	currentTZ_r=0.0;
	// cop compensation
	cop_compensation = new diana::CopCompensationFunc;
	cop_compensation->max_value_x = 0.05;
	cop_compensation->min_value_x = -0.05;
	cop_compensation->max_value_y = 0.05;
	cop_compensation->min_value_y = -0.05;
	gain_copFz_p_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_copFz_d_adjustment = new heroehs_math::FifthOrderTrajectory;
	updating_duration_cop = 0;
	copFz_p_gain = 0;
	copFz_d_gain = 0;
}
MotionModule::~MotionModule()
{
	queue_thread_.join();
}

void MotionModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;
	ros_node.setCallbackQueue(&callback_queue);
	/* publisher topics */
	l_leg_point_xyz_pub = ros_node.advertise<geometry_msgs::Vector3>("/l_leg_point_xyz",100);
	l_leg_point_rpy_pub = ros_node.advertise<geometry_msgs::Vector3>("/l_leg_point_rpy",100);

	r_leg_point_xyz_pub = ros_node.advertise<geometry_msgs::Vector3>("/r_leg_point_xyz",100);
	r_leg_point_rpy_pub = ros_node.advertise<geometry_msgs::Vector3>("/r_leg_point_rpy",100);

	cop_fz_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/cop_fz",100);

	current_leg_pose_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/current_leg_pose",100);

	/* subscribe topics */
	get_imu_data_sub_ = ros_node.subscribe("/imu/data", 100, &MotionModule::imuDataMsgCallback, this);
	get_ft_data_sub_ = ros_node.subscribe("/diana/force_torque_data", 100, &MotionModule::ftDataMsgCallback, this);

	// for gui
	set_balance_param_sub_ = ros_node.subscribe("/diana/balance_parameter", 5, &MotionModule::setBalanceParameterCallback, this);
	ros::Subscriber center_change_msg_sub = ros_node.subscribe("/diana/center_change", 5, &MotionModule::desiredCenterChangeMsgCallback, this);

	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);
}

void MotionModule::desiredCenterChangeMsgCallback(const diana_msgs::CenterChange::ConstPtr& msg) // GUI 에서 motion_num topic을 sub 받아 실행 모션 번호 디텍트
{
	is_moving_l_ = true;
	is_moving_r_ = true;

	if (temp_change_value_center != msg->center_change || temp_change_value_edge != msg->edge_change|| temp_turn_type.compare(msg->turn_type) || temp_change_type.compare(msg->change_type))
	{
		center_change_->parseMotionData(msg->turn_type, msg->change_type);

		if(!msg->change_type.compare("edge_change"))
			center_change_->calculateStepEndPointValue(msg->edge_change,100,msg->change_type); // 0.01 단위로 조정 가능.
		else
			center_change_->calculateStepEndPointValue(msg->center_change,100,msg->change_type); // 0.01 단위로 조정 가능.

		for(int m = 0 ; m<6 ; m++)
		{
			leg_end_point_l_(m,1) = center_change_->step_end_point_value[0][m];
			leg_end_point_r_(m,1) = center_change_->step_end_point_value[1][m];
			leg_end_point_l_(m,7) = msg->time_change;
			leg_end_point_r_(m,7) = msg->time_change;
		}
		temp_change_value_center = msg->center_change;
		temp_change_value_edge = msg->edge_change;
		temp_turn_type    = msg->turn_type;
		temp_change_type  = msg->change_type; // 이전값 저장
		ROS_INFO("Turn !!  Change");
	} // 변한 것이 있으면 값을 계산
	else
	{  ROS_INFO("Nothing to change");
	return;
	}
}
void MotionModule::imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg) // gyro data get
{
	currentGyroX = (double) msg->angular_velocity.x;
	currentGyroY = (double) msg->angular_velocity.y;
	currentGyroZ = (double) msg->angular_velocity.z;
	gyroRotationTransformation(currentGyroX, currentGyroY, currentGyroZ);
	balance_ctrl_.setCurrentGyroSensorOutput(tf_current_gyro_x, tf_current_gyro_y);
}
void MotionModule::gyroRotationTransformation(double gyro_z, double gyro_y, double gyro_x)
{
	Eigen::MatrixXd tf_gyro_value;
	tf_gyro_value.resize(3,1);
	tf_gyro_value.fill(0);
	tf_gyro_value(0,0) =  gyro_x;
	tf_gyro_value(1,0) =  gyro_y;
	tf_gyro_value(2,0) =  gyro_z;

	tf_gyro_value = (robotis_framework::getRotationZ(M_PI/2)*robotis_framework::getRotationZ(-M_PI))*tf_gyro_value;
	tf_current_gyro_x = tf_gyro_value(0,0);
	tf_current_gyro_y = tf_gyro_value(1,0);
	tf_current_gyro_z = tf_gyro_value(2,0);
}
void MotionModule::ftDataMsgCallback(const diana_msgs::ForceTorque::ConstPtr& msg)// force torque sensor data get
{
	currentFX_l = (double) msg->force_x_raw_l;
	currentFY_l = (double) msg->force_y_raw_l;
	currentFZ_l = -(double) msg->force_z_raw_l;


	currentTX_l = (double) msg->torque_x_raw_l;
	currentTY_l = (double) msg->torque_y_raw_l;
	currentTZ_l = -(double) msg->torque_z_raw_l;

	currentFX_r = (double) msg->force_x_raw_r;
	currentFY_r = (double) msg->force_y_raw_r;
	currentFZ_r = -(double) msg->force_z_raw_r;


	currentTX_r = (double) msg->torque_x_raw_r;
	currentTY_r = (double) msg->torque_y_raw_r;
	currentTZ_r = -(double) msg->torque_z_raw_r;

	cop_cal->ftSensorDataLeftGet(currentFX_l, currentFY_l, currentFZ_l, currentTX_l, currentTY_l, currentTZ_l);
	cop_cal->ftSensorDataRightGet(currentFX_r, currentFY_r, currentFZ_r, currentTX_r, currentTY_r, currentTZ_r);
}
void MotionModule::setBalanceParameterCallback(const diana_msgs::BalanceParam::ConstPtr& msg)
{
	if(balance_update_ == true)
	{
		ROS_ERROR("the previous task is not finished");
		return;
	}

	ROS_INFO("SET BALANCE_PARAM");

	balance_updating_duration_sec_ = 2.0;
	if(msg->updating_duration < 0)
		balance_updating_duration_sec_ = 2.0;
	else
		balance_updating_duration_sec_ = msg->updating_duration;

	desired_balance_param_.cob_x_offset_m         = msg->cob_x_offset_m        ;
	desired_balance_param_.cob_y_offset_m         = msg->cob_y_offset_m        ;
	desired_balance_param_.foot_roll_gyro_p_gain  = msg->foot_roll_gyro_p_gain ;
	desired_balance_param_.foot_roll_gyro_d_gain  = msg->foot_roll_gyro_d_gain ;
	desired_balance_param_.foot_pitch_gyro_p_gain = msg->foot_pitch_gyro_p_gain;
	desired_balance_param_.foot_pitch_gyro_d_gain = msg->foot_pitch_gyro_d_gain;

	updating_duration_cop = msg->updating_duration;
	copFz_p_gain = msg->foot_copFz_p_gain;
	copFz_d_gain = msg->foot_copFz_d_gain;


	balance_param_update_coeff_.changeTrajectory(0,0,0,0, balance_updating_duration_sec_, 1, 0, 0);

	balance_update_ = true;
	balance_updating_sys_time_sec_ = 0;
}

