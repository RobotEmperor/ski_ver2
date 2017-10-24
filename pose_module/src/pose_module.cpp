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

	ROS_INFO("< -------  Initialize Module : Base Module !!  ------->");

}



void PoseModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;

	ros_node.setCallbackQueue(&callback_queue);

	/* subscribe topics */
	// for gui


	ros::WallDuration duration(control_cycle_msec_ / 1000.0);

	while(ros_node.ok())
		callback_queue.callAvailable(duration);
}
bool PoseModule::isRunning()
{
	return running_;
}

void PoseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
		std::map<std::string, double> sensors)
{
	if (enable_ == false)
	{
		return;
	}
	//// read current position ////

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
			state_iter != dxls.end(); state_iter++)
	{
		std::string joint_name = state_iter->first;
		if(gazebo_check == true)
			result_[joint_name]->goal_position_ = result_[joint_name]->present_position_; // 가제보 상 초기위치 0
		else
			result_[joint_name]->goal_position_ = dxls[joint_name]->dxl_state_->present_position_; // 다이나믹셀에서 읽어옴

		// 초기위치 저장
	} // 등록된 다이나믹셀의 위치값을 읽어와서 goal position 으로 입력
	ROS_INFO("stay");

	// trajectory is working joint space control

	ROS_INFO("Trajectory start");

	result_[joint_id_to_name_[1]]->goal_position_ = 0;// 머리

	for(int id=10 ; id<23 ; id++)
	{
		if(id == 10 || id ==11 || id ==17 || id ==19) // 방향 반대인 다이나믹셀
		{
			result_[joint_id_to_name_[id]]->goal_position_ =0;
			result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
		}
		else
		{
			result_[joint_id_to_name_[id]]->goal_position_ =  0;
			result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
		}
	}


}
void PoseModule::stop()
{
	return;
}


