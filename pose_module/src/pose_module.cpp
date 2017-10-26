/*
 * pose_module.cpp
 *
 *  Created on: Oct 24, 2017
 *      Author: robotemperor
 */
/*
 * base_module.cpp
 *
 *  Created on: 2017. 10. 14.
 *      Author: robotemperor
 */
#include <stdio.h>
#include "pose_module/pose_module.h"

using namespace pose_module;

PoseModule::PoseModule()
: control_cycle_msec_(8)
{
	running_ = false;
	gazebo_check = false;
	is_moving_l_ = false;
	is_moving_r_ = false;
	is_moving_one_joint_ = false;
	enable_       = false;
	module_name_  = "pose_module";
	control_mode_ = robotis_framework::PositionControl;

	// Dynamixel initialize ////

	result_["head"]        = new robotis_framework::DynamixelState(); // joint 1
	result_["waist_roll"]  = new robotis_framework::DynamixelState(); // joint 10

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

	///////////////////////////

	end_to_rad_l_ = new heroehs_math::CalRad;
	end_to_rad_r_ = new heroehs_math::CalRad;
	one_joint_ = new heroehs_math::CalRad;

	//////////////////////////
  result_rad_one_joint_= 0;
	traj_time_ = 4.0;
}
PoseModule::~PoseModule()
{
	queue_thread_.join();
}
void PoseModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	control_cycle_msec_ = control_cycle_msec;
	queue_thread_ = boost::thread(boost::bind(&PoseModule::queueThread, this));

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
			it != robot->dxls_.end(); it++)
	{
		std::string joint_name = it->first;
		robotis_framework::Dynamixel* dxl_info = it->second;

		joint_name_to_id_[joint_name] = dxl_info->id_;
		joint_id_to_name_[dxl_info->id_] = joint_name;
	}


	leg_end_point_l_.resize(6,8);
	leg_end_point_l_.fill(0);
	leg_end_point_l_(2,0) = -0.55; // 초기값
	leg_end_point_l_(2,1) = -0.55;
	end_to_rad_l_->cal_end_point_tra_pz->current_pose = -0.55;
	end_to_rad_l_->current_pose_change(2,0) = -0.55;


	leg_end_point_r_.resize(6,8);
	leg_end_point_r_.fill(0);
	leg_end_point_r_(2,0) = -0.55;  // 초기값
	leg_end_point_r_(2,1) = -0.55;
	end_to_rad_r_->cal_end_point_tra_pz->current_pose = -0.55;
	end_to_rad_r_->current_pose_change(2,0) = -0.55;

	result_rad_l_.resize(7,1);
	result_rad_r_.resize(7,1);
	result_rad_l_.fill(0);
	result_rad_r_.fill(0);

	one_joint_ctrl_.resize(1,8);
	one_joint_ctrl_.fill(0);
	one_joint_ctrl_(0,1) = 0;
	result_rad_one_joint_ = 0;

	for(int joint_num_=0; joint_num_< 6 ; joint_num_ ++)
	{
		leg_end_point_l_(joint_num_, 7) = traj_time_;
		leg_end_point_r_(joint_num_, 7) = traj_time_;
	}
	one_joint_ctrl_(0,7) = traj_time_;

	ROS_INFO("< -------  Initialize Module : Pose Module !!  ------->");
}
void PoseModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;

	ros_node.setCallbackQueue(&callback_queue);

	/* subscribe topics */
	// for gui
	ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/desired_pose", 5, &PoseModule::desiredPoseMsgCallback, this);
	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);
}
bool PoseModule::isRunning()
{
	return running_;
}
void PoseModule::desiredPoseMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) // GUI 에서 init pose topic을 sub 받아 실행
{
	for(int joint_num_= 0; joint_num_< 6; joint_num_++)
	{
		leg_end_point_l_(joint_num_, 1) = msg->data[joint_num_]; // left leg
		leg_end_point_r_(joint_num_, 1) = msg->data[joint_num_+6]; // right leg
		//leg_end_point_l_(joint_num_, 7) = msg-> data[12]*RADIAN2DEGREE;
		//leg_end_point_r_(joint_num_, 7) = msg-> data[12]*RADIAN2DEGREE;
	}
	one_joint_ctrl_(0,1) = msg-> data[12]; // waist roll

	is_moving_l_ = true;
	is_moving_r_ = true;
	is_moving_one_joint_ = true;
}

void PoseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
		std::map<std::string, double> sensors)
{
	if (enable_ == false)
	{
		return;
	}
	//// read current position ////

	if(is_moving_l_ == false && is_moving_r_ == false && is_moving_one_joint_ == false)
	{
		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
			if(gazebo_check == true)
				result_[joint_name]->goal_position_ = result_[joint_name]->present_position_; // 가제보 상 초기위치 0
			else
			{
				result_[joint_name]->goal_position_ = dxls[joint_name]->dxl_state_->present_position_; // 다이나믹셀에서 읽어옴
			}
		} // 등록된 다이나믹셀의 위치값을 읽어와서 goal position 으로 입력
		ROS_INFO("stay");
	}
	else
	{
		ROS_INFO("Trajectory start");

		// trajectory is working cartesian space control
		result_rad_l_ = end_to_rad_l_->cal_end_point_to_rad(leg_end_point_l_);
		result_rad_r_ = end_to_rad_r_->cal_end_point_to_rad(leg_end_point_r_);
		result_rad_one_joint_ = one_joint_ -> cal_one_joint_rad(one_joint_ctrl_);

		//<---  joint space control --->
		result_[joint_id_to_name_[1]]->goal_position_ = 0;// head
		result_[joint_id_to_name_[10]]->goal_position_ = -result_rad_one_joint_; // waist roll

		//<---  cartesian space control  --->
		result_[joint_id_to_name_[11]]->goal_position_ = -result_rad_l_(1,0);
		result_[joint_id_to_name_[13]]->goal_position_ = result_rad_l_(2,0);
		result_[joint_id_to_name_[15]]->goal_position_ = result_rad_l_(3,0);

		result_[joint_id_to_name_[17]]->goal_position_ = -result_rad_l_(4,0);
		result_[joint_id_to_name_[19]]->goal_position_ = -result_rad_l_(5,0);
		result_[joint_id_to_name_[21]]->goal_position_ = result_rad_l_(6,0);

		result_[joint_id_to_name_[12]]->goal_position_ = result_rad_r_(1,0);
		result_[joint_id_to_name_[14]]->goal_position_ = result_rad_r_(2,0);
		result_[joint_id_to_name_[16]]->goal_position_ = result_rad_r_(3,0);

		result_[joint_id_to_name_[18]]->goal_position_ = result_rad_r_(4,0);
		result_[joint_id_to_name_[20]]->goal_position_ = result_rad_r_(5,0);
		result_[joint_id_to_name_[22]]->goal_position_ = result_rad_r_(6,0);

		//<---  read for gazebo  --->
		for(int id=10 ; id<23 ; id++)
		{
			result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
		}

		is_moving_l_ = end_to_rad_l_-> is_moving_check;
		is_moving_r_ = end_to_rad_r_-> is_moving_check;
		is_moving_one_joint_ = one_joint_ ->is_moving_check;
	}

}
void PoseModule::stop()
{
	return;
}


