/*
 * upper_body_module_state.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: robotemperor
 */
#include "upper_body_module/upper_body_module.h"

using namespace upper_body_module;

UpperBodyModule::UpperBodyModule()
: control_cycle_msec_(8)
{
	running_ = false;
	gazebo_check = false;
	is_moving_head_ = false;
	is_moving_waist_ = false;

	enable_       = false;
	module_name_  = "upper_body_module";
	control_mode_ = robotis_framework::PositionControl;

	// Dynamixel initialize ////
	result_["waist_yaw"]  = new robotis_framework::DynamixelState(); // joint 9
	result_["waist_roll"] = new robotis_framework::DynamixelState(); // joint 10

	result_["head_yaw"]   = new robotis_framework::DynamixelState(); // joint 23

	result_["head_pitch"]   = new robotis_framework::DynamixelState(); // joint 24
	result_["head_roll"]   = new robotis_framework::DynamixelState(); // joint 25

	///////////////////////////
	//center change waist
	center_change_ = new diana_motion_waist::CenterChange;
	temp_change_value_center = 0.0;
	temp_turn_type = "basic";
	temp_change_type = "basic";

	// motion control variables
	waist_kinematics_ = new heroehs_math::KinematicsEulerAngle;
	end_to_rad_waist_ = new heroehs_math::CalRad;

	head_kinematics_  = new heroehs_math::KinematicsEulerAngle;
	head_point_kinematics_ = new heroehs_math::Kinematics;
	end_to_rad_head_  = new heroehs_math::CalRad;

	traj_time_test = 4;
	new_count_ = 1 ;
	temp_waist_yaw_rad   = 0;
	temp_waist_roll_rad  = 0;

	end_to_rad_waist_ -> cal_end_point_tra_alpha -> current_time = traj_time_test;
	end_to_rad_waist_ -> cal_end_point_tra_betta -> current_time = 0;
	end_to_rad_waist_ -> cal_end_point_tra_kamma -> current_time = traj_time_test;

	end_to_rad_head_ -> cal_end_point_tra_alpha -> current_time = traj_time_test;
	end_to_rad_head_ -> cal_end_point_tra_betta -> current_time = traj_time_test;
	end_to_rad_head_ -> cal_end_point_tra_kamma -> current_time = traj_time_test;

	end_to_rad_waist_ -> cal_end_point_tra_alpha -> final_time = traj_time_test;
	end_to_rad_waist_ -> cal_end_point_tra_betta -> final_time = 0;
	end_to_rad_waist_ -> cal_end_point_tra_kamma -> final_time = traj_time_test;

	end_to_rad_head_ -> cal_end_point_tra_alpha -> final_time = traj_time_test;
	end_to_rad_head_ -> cal_end_point_tra_betta -> final_time = traj_time_test;
	end_to_rad_head_ -> cal_end_point_tra_kamma -> final_time = traj_time_test;

	// gyro control variables
	currentGyroX = 0;
	currentGyroY = 0;
	currentGyroZ = 0;
	tf_current_gyro_x = 0;
	tf_current_gyro_y = 0;
	tf_current_gyro_z = 0;
	gyro_roll_function = new control_function::PID_function(0.008,5*DEGREE2RADIAN,-5*DEGREE2RADIAN,0,0,0);
	gyro_yaw_function  = new control_function::PID_function(0.008,5*DEGREE2RADIAN,-5*DEGREE2RADIAN,0,0,0);
	gain_roll_p_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_roll_d_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_yaw_p_adjustment  = new heroehs_math::FifthOrderTrajectory;
	gain_yaw_d_adjustment  = new heroehs_math::FifthOrderTrajectory;
	updating_duration = 0;
	gyro_roll_p_gain  = 0;
	gyro_roll_d_gain  = 0;
	gyro_yaw_p_gain   = 0;
	gyro_yaw_d_gain   = 0;

	//cop compensation variables
	cop_compensation_waist  = new diana::CopCompensationFunc;
	gain_copFz_p_adjustment = new heroehs_math::FifthOrderTrajectory;
	gain_copFz_d_adjustment = new heroehs_math::FifthOrderTrajectory;
	current_cop_fz_x = 0;
	current_cop_fz_y = 0;
	reference_cop_fz_x = 0;
	reference_cop_fz_y = 0;

	copFz_p_gain = 0;
	copFz_d_gain = 0;
}
UpperBodyModule::~UpperBodyModule()
{
	queue_thread_.join();
}
// ros message communication thread
void UpperBodyModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;
	ros_node.setCallbackQueue(&callback_queue);
	// publish topics
	current_waist_pose_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/current_waist_pose",100);


	// subscribe topics
	//	current_leg_pose_sub = ros_node.subscribe("/current_leg_pose", 5, &UpperBodyModule::currentLegPoseMsgCallback, this);
	get_imu_data_sub_ = ros_node.subscribe("/imu/data", 100, &UpperBodyModule::imuDataMsgCallback, this);
	//	get_ft_data_sub_ = ros_node.subscribe("/diana/force_torque_data", 100, &UpperBodyModule::ftDataMsgCallback, this);

	center_change_msg_sub = ros_node.subscribe("/diana/center_change", 5, &UpperBodyModule::desiredCenterChangeMsgCallback, this);
	balance_param_waist_sub = ros_node.subscribe("/diana/balance_parameter_waist", 5, &UpperBodyModule::balanceParameterWaistMsgCallback, this);

	//current cop and reference cop from leg module
	cop_fz_sub = ros_node.subscribe("/cop_fz", 5, &UpperBodyModule::copFzMsgCallBack, this);

	// test desired pose
	head_test = ros_node.subscribe("/desired_pose_head", 5, &UpperBodyModule::desiredPoseHeadMsgCallbackTEST, this);
	waist_test = ros_node.subscribe("/desired_pose_waist", 5, &UpperBodyModule::desiredPoseWaistMsgCallbackTEST, this);

	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);

}
// TEST /////////////////////////////////////////////
void UpperBodyModule::desiredPoseWaistMsgCallbackTEST(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	waist_end_point_(3, 1) = msg->data[0]; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	waist_end_point_(5, 1) = msg->data[1]; // roll
	is_moving_waist_ = true;
}
void UpperBodyModule::desiredPoseHeadMsgCallbackTEST(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	head_end_point_(3, 1) = msg->data[0]; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	head_end_point_(4, 1) = msg->data[1];
	head_end_point_(5, 1) = msg->data[2]; // roll
	is_moving_head_ = true;
}
/////////////////////////////////////////////////////
// sensor data get///////////////////////////////////
void UpperBodyModule::imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg) // gyro data get
{
	currentGyroX = (double) msg->angular_velocity.x;
	currentGyroY = (double) msg->angular_velocity.y;
	currentGyroZ = (double) msg->angular_velocity.z;
	gyroRotationTransformation(currentGyroZ, currentGyroY, currentGyroX);
}
void UpperBodyModule::gyroRotationTransformation(double gyro_z, double gyro_y, double gyro_x)
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
//////////////////////////////////////////////////////////////////////
//balance message for gyro///////////////////////////////////
void UpperBodyModule::balanceParameterWaistMsgCallback(const diana_msgs::BalanceParamWaist::ConstPtr& msg)
{
	updating_duration = msg->updating_duration;
	gyro_roll_p_gain  = msg->waist_roll_gyro_p_gain;
	gyro_roll_d_gain  = msg->waist_roll_gyro_d_gain;
	gyro_yaw_p_gain   = msg->waist_yaw_gyro_p_gain;
	gyro_yaw_d_gain   = msg->waist_yaw_gyro_d_gain;
	copFz_p_gain = msg->waist_copFz_p_gain;
	copFz_d_gain = msg->waist_copFz_d_gain;
}
//////////////////////////////////////////////////////////////////////
//current cop and reference cop from leg module
void UpperBodyModule::copFzMsgCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	current_cop_fz_x   = msg->data[0];
	current_cop_fz_y   = msg->data[1];
	reference_cop_fz_x = msg->data[2];
	reference_cop_fz_y = msg->data[3];
}
// leg state ///////////////////////////////////
/*void UpperBodyModule::currentLegPoseMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	for(int joint_num = 1; joint_num<7; joint_num++)
	{
		l_leg_real_joint(joint_num, 0) = msg->data[joint_num - 1];
		r_leg_real_joint(joint_num, 0) = msg->data[joint_num + 5];
	}
}*/
//////////////////////////////////////////////////////////////////////
// center change msg ///////////////////////////////////
void UpperBodyModule::desiredCenterChangeMsgCallback(const diana_msgs::CenterChange::ConstPtr& msg) // GUI 에서 motion_num topic을 sub 받아 실행 모션 번호 디텍트
{
	if (temp_change_value_center != msg->center_change || temp_turn_type.compare(msg->turn_type) || temp_change_type.compare(msg->change_type))
	{
		center_change_->parseMotionData(msg->turn_type, msg->change_type);
		center_change_->calculateStepEndPointValue(msg->center_change,100,msg->change_type); // 0.01 단위로 조정 가능.

		for(int m = 0 ; m<2 ; m++)
		{
			waist_end_point_(2*m+3,1) = center_change_->step_end_point_value[m];
			waist_end_point_(2*m+3,7) = msg->time_change_waist;
		}
		temp_change_value_center = msg->center_change;
		temp_turn_type    = msg->turn_type;
		temp_change_type  = msg->change_type; // 이전값 저장
		ROS_INFO("Turn !!  Change");
		is_moving_waist_ = true;
	} // 변한 것이 있으면 값을 계산
	else
	{  ROS_INFO("Nothing to change");
	return;
	}
}
//////////////////////////////////////////////////////////////////////



