/*
 * base_module.cpp
 *
 *  Created on: 2017. 10. 14.
 *      Author: robotemperor
 */
#include <stdio.h>
#include "base_module/base_module.h"
using namespace base_module;

BaseModule::BaseModule()
: control_cycle_msec_(8)
{
	running_ = false;
	enable_       = false;
	module_name_  = "base_module";
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

	base_module_state  = new BaseModuleState();

	motion_trajectory_11 = new FifthOrderTrajectory();
	motion_trajectory_13 = new FifthOrderTrajectory();




}

BaseModule::~BaseModule()
{
	queue_thread_.join();
}

void BaseModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{

	control_cycle_msec_ = control_cycle_msec;
	queue_thread_ = boost::thread(boost::bind(&BaseModule::queueThread, this));

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
			it != robot->dxls_.end(); it++)
	{
		std::string joint_name = it->first;
		robotis_framework::Dynamixel* dxl_info = it->second;

		joint_name_to_id_[joint_name] = dxl_info->id_;
		joint_id_to_name_[dxl_info->id_] = joint_name;

		base_module_state->MAX_JOINT_ID_STATE ++; // joint 개수 확인
	}
	base_module_state->joint_ini_pose_state.resize(23,1);
	base_module_state->joint_ini_pose_state.fill(0);
	//	base_module_state->is_moving_state = motion_trajectory->is_moving_traj;
	ROS_INFO("< -------  Initialize Module : Base Module !!  ------->");

}

void BaseModule::parse_init_pose_data_(const std::string &path)
{

	YAML::Node doc; // YAML file class 선언!
	try
	{
		// load yaml
		doc = YAML::LoadFile(path.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.

	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	double mov_time_ = 0.0;


	mov_time_= doc["mov_time"].as<double>(); // YAML 에 string "mov_time"을 읽어온다.
	base_module_state->mov_time_state = mov_time_;

	YAML::Node tar_pose_node = doc["tar_pose"];// YAML 에 string "tar_pose"을 읽어온다.
	for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it) //tar_pose_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int id;
		double value;
		// 한 줄에서 int 와 double 을 분리한다.
		id = it->first.as<int>();
		value = it->second.as<double>();

		base_module_state->joint_ini_pose_state.coeffRef(id, 0) = value * DEGREE2RADIAN; // YAML에 로드된 초기 포즈 값을 라디안으로 바꾸고, eigen matrix 에 id 개수만큼 열을 생성한다.
		//ROS_INFO("ID : %d, Value : %f", id, base_module_state->joint_ini_pose_state.coeffRef(id, 0)*RADIAN2DEGREE/0.088);
	}

}

void BaseModule::initPoseMsgCallback(const std_msgs::String::ConstPtr& msg) // GUI 에서 init pose topic을 sub 받아 실
{
	std::string init_pose_path = ros::package::getPath("base_module") + "/data/ini_pose.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	parse_init_pose_data_(init_pose_path); // YAML 파일 로드 및 값 저장.
	base_module_state->is_moving_state = true;
}

void BaseModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;

	ros_node.setCallbackQueue(&callback_queue);

	/* subscribe topics */
	// for gui
	ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/init_pose", 5, &BaseModule::initPoseMsgCallback, this);


	ros::WallDuration duration(control_cycle_msec_ / 1000.0);

	while(ros_node.ok())
		callback_queue.callAvailable(duration);
}


bool BaseModule::isRunning()
{
	return running_;
}

void BaseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
		std::map<std::string, double> sensors)
{
	if (enable_ == false)
	{
		return;
	}

	//// read current position ////
	if(base_module_state->is_moving_state == false)
	{
		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
			//result_[joint_name]->goal_position_ = dxls[joint_name]->dxl_state_->present_position_;
		} // 등록된 다이나믹셀의 위치값을 읽어와서 goal position 으로 입력
	}

	else // trajectory is working
	{
		ROS_INFO("Trajectory start");

		/*result_[joint_id_to_name_[1]]->goal_position_ = motion_trajectory->fifth_order_traj_gen(dxls[joint_id_to_name_[1]]->dxl_state_->present_position_ ,
							base_module_state->joint_ini_pose_state(1,0) ,
							0,0,0,0,0,base_module_state->mov_time_state);*/ // 머리


		/*	for(int id=6;id<12;id++)
		{
			result_[joint_id_to_name_[2*id-1]]->goal_position_ =  motion_trajectory->fifth_order_traj_gen(0 ,
					base_module_state->joint_ini_pose_state(2*id-1,0) ,
					0,0,0,0,0,base_module_state->mov_time_state);
		}*/

		result_[joint_id_to_name_[11]]->goal_position_ =  -motion_trajectory_11->fifth_order_traj_gen(0 ,
				base_module_state->joint_ini_pose_state(11,0) ,
				0,0,0,0,0,base_module_state->mov_time_state);

		result_[joint_id_to_name_[13]]->goal_position_ =  motion_trajectory_13->fifth_order_traj_gen(0 ,
									base_module_state->joint_ini_pose_state(13,0) ,
									0,0,0,0,0,base_module_state->mov_time_state);



	}

	//base_module_state->is_moving_state = motion_trajectory_11->is_moving_traj;
	//base_module_state->is_moving_state = motion_trajectory_13->is_moving_traj;
}

void BaseModule::stop()
{
	return;
}


