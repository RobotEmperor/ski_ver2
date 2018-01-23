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
	currentGyroOrientationW = 0;
	currentGyroOrientationX = 0;
	currentGyroOrientationY = 0;
	currentGyroOrientationZ = 0;
	tf_current_gyro_x = 0;
	tf_current_gyro_y = 0;
	tf_current_gyro_z = 0;
	tf_current_gyro_orientation_x = 0;
	tf_current_gyro_orientation_y = 0;
	tf_current_gyro_orientation_z = 0;
	tf_gyro_value.resize(3,1);
	tf_gyro_value.fill(0);
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

    for(int i = 0; i<4 ;i++)
    {
    	flag[i][0] = 0;
    	flag[i][1] = 0;
    	flag[i][2] = 0;
    }

	filter_head = new control_function::Filter;
	temp_head_roll  = 0;
	temp_head_pitch = 0;
	temp_head_yaw   = 0;
	temp_pre_roll = 0;
	temp_pre_pitch = 0;
	temp_pre_yaw = 0;

	head_balance_enable = new heroehs_math::FifthOrderTrajectory ;
	head_enable = 0;
	result_head_enable = 0;
	head_enable_time = 2.0;


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
	current_flag_position1_pub = ros_node.advertise<geometry_msgs::Vector3>("/current_flag_position1",100);
	current_flag_position2_pub = ros_node.advertise<geometry_msgs::Vector3>("/current_flag_position2",100);
	current_flag_position3_pub = ros_node.advertise<geometry_msgs::Vector3>("/current_flag_position3",100);
	current_flag_position4_pub = ros_node.advertise<geometry_msgs::Vector3>("/current_flag_position4",100);

	// subscribe topics
	flag_position_get_sub = ros_node.subscribe("/gate_watcher/flag_data", 100, &UpperBodyModule::flagPositionGetMsgCallback, this);
	get_imu_data_sub_ = ros_node.subscribe("/imu/data", 100, &UpperBodyModule::imuDataMsgCallback, this);

	balance_param_waist_sub = ros_node.subscribe("/diana/balance_parameter_waist", 5, &UpperBodyModule::balanceParameterWaistMsgCallback, this);
	head_balance_sub = ros_node.subscribe("/head_balance", 5, &UpperBodyModule::headBalanceMsgCallback, this);

	//current cop and reference cop from leg module
	cop_fz_sub = ros_node.subscribe("/cop_fz", 5, &UpperBodyModule::copFzMsgCallBack, this);

	// test desired pose
	head_test = ros_node.subscribe("/desired_pose_head", 5, &UpperBodyModule::desiredPoseHeadMsgCallbackTEST, this);
	waist_test = ros_node.subscribe("/desired_pose_waist", 5, &UpperBodyModule::desiredPoseWaistMsgCallbackTEST, this);

	desired_pose_all_sub = ros_node.subscribe("/desired_pose_all", 5, &UpperBodyModule::desiredPoseAllMsgCallback, this);


	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);

}
// TEST /////////////////////////////////////////////
void UpperBodyModule::desiredPoseWaistMsgCallbackTEST(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	waist_end_point_(3, 1) = msg->data[0]; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	waist_end_point_(5, 1) = msg->data[1]; // roll
	waist_end_point_(3, 7) = msg->data[2];
	waist_end_point_(5, 7) = msg->data[3];
	is_moving_waist_ = true;
}
void UpperBodyModule::desiredPoseHeadMsgCallbackTEST(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	head_end_point_(3, 1) = msg->data[0]; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	head_end_point_(4, 1) = msg->data[1];
	head_end_point_(5, 1) = msg->data[2]; // roll

	for(int joint_num_= 3; joint_num_< 6 ; joint_num_ ++)  // waist 3, 5번 // head 345 초기화
	{
		head_end_point_ (joint_num_, 7) = msg->data[3];
	}

	is_moving_head_ = true;

}
/////////////////////////////////////////////////////
// flag position data get////////////////////////////
void UpperBodyModule::flagPositionGetMsgCallback(const diana_msgs::FlagDataArray& msg)
{
	// head point get
	if(msg.length >= 1)
	{
		for(int num = 0; num < msg.length; num++)
		{
			currentFlagPositionFunction(msg.data[num].position.x, msg.data[num].position.y, msg.data[num].position.z);// 천유 좌표를 넣어야함
			flag[num][0] = head_point_kinematics_->head_point_on_origin_x*0.01;  // x
			flag[num][1]  = head_point_kinematics_->head_point_on_origin_y*0.01; // y
			flag[num][2]  = head_point_kinematics_->head_point_on_origin_z*0.01; // z
		}
	}
	else
	{
		printf("No data!!!!");
	}
}
/////////////////////////////////////////////////////

// sensor data get///////////////////////////////////
void UpperBodyModule::imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg) // gyro data get
{
	currentGyroX = (double) msg->angular_velocity.x;
	currentGyroY = (double) msg->angular_velocity.y;
	currentGyroZ = (double) msg->angular_velocity.z;
	currentGyroOrientationW = (double) msg->orientation.w;
	currentGyroOrientationX = (double) msg->orientation.x;
	currentGyroOrientationY = (double) msg->orientation.y;
	currentGyroOrientationZ = (double) msg->orientation.z;
	gyroRotationTransformation(currentGyroZ, currentGyroY, currentGyroX);
	tf_current_gyro_x = tf_gyro_value(0,0);
	tf_current_gyro_y = tf_gyro_value(1,0);
	tf_current_gyro_z = tf_gyro_value(2,0);
	//	quaternionToAngle(currentGyroOrientationW, currentGyroOrientationX, currentGyroOrientationY, currentGyroOrientationZ);
	//	gyroRotationTransformation(tf_gyro_value(2,0), tf_gyro_value(1,0), tf_gyro_value(0,0));
	//	tf_current_gyro_orientation_x = tf_gyro_value(0,0);
	//	tf_current_gyro_orientation_y = tf_gyro_value(1,0);
	//	tf_current_gyro_orientation_z = tf_gyro_value(2,0);
}
void UpperBodyModule::gyroRotationTransformation(double gyro_z, double gyro_y, double gyro_x)
{
	tf_gyro_value(0,0) =  gyro_x;
	tf_gyro_value(1,0) =  gyro_y;
	tf_gyro_value(2,0) =  gyro_z;

	tf_gyro_value = (robotis_framework::getRotationZ(-M_PI/2)*robotis_framework::getRotationY(-M_PI))*tf_gyro_value;
}
void UpperBodyModule::quaternionToAngle(double q_w, double q_x, double q_y, double q_z)
{
	double sinr = 2.0 * (q_w * q_x + q_y * q_z);
	double cosr = 1.0 - 2.0 * (q_x * q_x + q_y * q_y);
	tf_gyro_value(0,0) = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = 2.0 * (q_w * q_y - q_z * q_x);
	if (fabs(sinp) >= 1)
		tf_gyro_value(1,0) = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		tf_gyro_value(1,0) = asin(sinp);

	// yaw (z-axis rotation)
	double siny = 2.0 * (q_w * q_z + q_x * q_y);
	double cosy = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
	tf_gyro_value(2,0) = atan2(siny, cosy);
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
void UpperBodyModule::headBalanceMsgCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data == 1)
	{
		head_enable = 1.0;
	}
	else
	{
		head_enable = 0;
	}
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
//////////////////////////////////////////////////////////////////////
void UpperBodyModule::desiredPoseAllMsgCallback(const diana_msgs::DesiredPoseCommand::ConstPtr& msg)
{
	waist_end_point_(3, 1) = msg->waist_final_position[0];
	waist_end_point_(5, 1) = msg->waist_final_position[1];

	waist_end_point_(3, 2) = msg->waist_init_vel[0];
	waist_end_point_(5, 2) = msg->waist_init_vel[1];

	waist_end_point_(3, 3) = msg->waist_final_vel[0];
	waist_end_point_(5, 3) = msg->waist_final_vel[1];

	waist_end_point_(3, 7) = msg->time_waist;
	waist_end_point_(5, 7) = msg->time_waist;

	is_moving_waist_ = true;

}
//////////////////////////////////////////////////////////////////////



