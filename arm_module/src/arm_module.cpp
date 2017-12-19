/*
 * arm_module.cpp
 *
 *  Created on: Dec 13, 2017
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

	result_["r_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 2
	result_["r_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 4
	result_["r_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 6


/*
	result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
	result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3
	result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5
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

	traj_time_test = 4;
	new_count_ = 1 ;
}
ArmModule::~ArmModule()
{
	queue_thread_.join();
}
void ArmModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	is_moving_l_arm_ = false;
	is_moving_r_arm_ = false;

	control_cycle_msec_ = control_cycle_msec;
	queue_thread_ = boost::thread(boost::bind(&ArmModule::queueThread, this));

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
			it != robot->dxls_.end(); it++)
	{
		std::string joint_name = it->first;
		robotis_framework::Dynamixel* dxl_info = it->second;

		joint_name_to_id_[joint_name] = dxl_info->id_;
		joint_id_to_name_[dxl_info->id_] = joint_name;
	}
	// arm initialize value in local frame
	// left //
	l_arm_end_point_.resize(6,8);
	l_arm_end_point_.fill(0);
	//l_arm_end_point_.(1,0) = ; // y 초기값
	//l_arm_end_point_.(1,1) = ; //
	l_arm_end_point_(2,0) = -0.47;
	l_arm_end_point_(2,1) = -0.47;
	//	end_to_rad_l_arm_->cal_end_point_tra_py->current_pose = 0.105;
	//	end_to_rad_l_arm_->current_pose_change(1,0) = 0.105;
	end_to_rad_l_arm_->cal_end_point_tra_pz->current_pose = -0.47;
	end_to_rad_l_arm_->current_pose_change(2,0) = -0.47;
	result_end_l_arm_.resize(6,1);
	result_end_l_arm_.fill(0);
	//right //
	r_arm_end_point_.resize(6,8);
	r_arm_end_point_.fill(0);
	result_end_r_arm_.resize(6,1);
	result_end_r_arm_.fill(0);
	//l_arm_end_point_.(1,0) = ; // y 초기값
	//l_arm_end_point_.(1,1) = ; //
	r_arm_end_point_(2,0) = -0.47;
	r_arm_end_point_(2,1) = -0.47;
	//	end_to_rad_l_arm_->cal_end_point_tra_py->current_pose = 0.105;
	//	end_to_rad_l_arm_->current_pose_change(1,0) = 0.105;
	end_to_rad_r_arm_->cal_end_point_tra_pz->current_pose = -0.47;
	end_to_rad_r_arm_->current_pose_change(2,0) = -0.47;

	for(int joint_num_= 0; joint_num_< 6 ; joint_num_ ++)
	{
		l_arm_end_point_ (joint_num_, 7) = traj_time_test;
		r_arm_end_point_ (joint_num_, 7) = traj_time_test;
	}
	ROS_INFO("< -------  Initialize Module : Arm Module  !!  ------->");
}
void ArmModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;
	ros_node.setCallbackQueue(&callback_queue);

	/* publisher topics
	 subscribe topics
	get_imu_data_sub_ = ros_node.subscribe("/imu/data", 100, &MotionModule::imuDataMsgCallback, this);
	get_ft_data_sub_ = ros_node.subscribe("/diana/force_torque_data", 100, &MotionModule::ftDataMsgCallback, this);
	 */
	// subscriber topics
	desired_pose_arm_sub_   = ros_node.subscribe("/desired_pose_arm", 5, &ArmModule::desiredPoseArmMsgCallbackTEST, this);
	current_waist_pose_sub_ = ros_node.subscribe("/current_waist_pose", 5, &ArmModule::currentWaistPoseMsgCallbackTEST, this);
	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);
}
void ArmModule::currentWaistPoseMsgCallbackTEST(const std_msgs::Float64MultiArray::ConstPtr& msg)
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
bool ArmModule::isRunning()
{
	return running_;
}
void ArmModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
		std::map<std::string, double> sensors)
{

	if (enable_ == false)
	{
		return;
	}
	//// read current position ////
	if(new_count_ == 1)
	{
		is_moving_l_arm_ = false;
		is_moving_r_arm_ = false;
		new_count_ ++;
		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
			if(!joint_name.compare("l_shoulder_pitch")|| !joint_name.compare("l_shoulder_roll") || !joint_name.compare("l_elbow_pitch") ||
					!joint_name.compare("r_shoulder_pitch") || !joint_name.compare("r_shoulder_roll") || !joint_name.compare("r_elbow_pitch"))
			{
				if(gazebo_check == true)
					result_[joint_name]->goal_position_ = result_[joint_name]->present_position_; // 가제보 상 초기위치 0
			}
		} // 등록된 다이나믹셀의 위치값을 읽어와서 goal position 으로 입력
		ROS_INFO("Arm Start");
		result_end_l_arm_ = end_to_rad_l_arm_ -> cal_end_point_to_rad(l_arm_end_point_);
		result_end_r_arm_ = end_to_rad_r_arm_ -> cal_end_point_to_rad(r_arm_end_point_);
	}
	if(is_moving_l_arm_ == false && is_moving_r_arm_ == false)
	{
		ROS_INFO("Arm Stay");
	}
	else
	{
		ROS_INFO("Arm Module Trajectory Start");
		//l_arm_kinematics_ -> ArmToOriginTransformation(waist_yaw_rad_ , waist_roll_rad_, l_arm_desired_point_x_, l_arm_desired_point_y_, l_arm_desired_point_z_);
		//r_arm_kinematics_ -> ArmToOriginTransformation(waist_yaw_rad_ , waist_roll_rad_, r_arm_desired_point_x_, r_arm_desired_point_y_, r_arm_desired_point_z_);

		result_end_l_arm_ = end_to_rad_l_arm_ -> cal_end_point_to_rad(l_arm_end_point_);
		result_end_r_arm_ = end_to_rad_r_arm_ -> cal_end_point_to_rad(r_arm_end_point_);

		is_moving_l_arm_  = end_to_rad_l_arm_  -> is_moving_check;
		is_moving_r_arm_  = end_to_rad_r_arm_  -> is_moving_check;
	}
	///////////////////////////////////////////////////// control //////////////////////////////////////////////////////////
	l_arm_kinematics_ -> InverseKinematicsArm(result_end_l_arm_(0,0), result_end_l_arm_(1,0), result_end_l_arm_(2,0));
	r_arm_kinematics_ -> InverseKinematicsArm(result_end_r_arm_(0,0), result_end_r_arm_(1,0), result_end_r_arm_(2,0));


	result_[joint_id_to_name_[1]]->goal_position_ = l_arm_kinematics_->joint_radian(1,0);
	result_[joint_id_to_name_[3]]->goal_position_ = l_arm_kinematics_->joint_radian(2,0);
	result_[joint_id_to_name_[5]]->goal_position_ = l_arm_kinematics_->joint_radian(3,0);

	result_[joint_id_to_name_[2]]->goal_position_ = r_arm_kinematics_->joint_radian(1,0);
	result_[joint_id_to_name_[4]]->goal_position_ = r_arm_kinematics_->joint_radian(2,0);
	result_[joint_id_to_name_[6]]->goal_position_ = r_arm_kinematics_->joint_radian(3,0);
}
void ArmModule::stop()
{
	return;
}
