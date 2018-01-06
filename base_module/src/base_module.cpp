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
	gazebo_check  = false;
	module_name_  = "base_module";
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

	result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
	result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3
	result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5

	result_["waist_yaw"]        = new robotis_framework::DynamixelState();  // joint 9
	result_["waist_roll"]       = new robotis_framework::DynamixelState();  // joint 10

	result_["head_yaw"]         = new robotis_framework::DynamixelState();  // joint 23

	result_["l_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 17
	result_["r_knee_pitch"]     = new robotis_framework::DynamixelState();  // joint 18





	new_count_ = 1;
	///////////////////////////

	for(int i=1;i<26;i++)
	{
		motion_trajectory[i]= new heroehs_math::FifthOrderTrajectory();
	}

	base_module_state  = new BaseModuleState();
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
	base_module_state -> joint_ini_pose_state.resize(26,1);
	base_module_state -> joint_ini_pose_goal.resize(26,1);
	base_module_state -> joint_ini_pose_state.fill(0);
	base_module_state -> joint_ini_pose_goal.fill(0);

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

		base_module_state->joint_ini_pose_goal.coeffRef(id, 0) = value * DEGREE2RADIAN; // YAML에 로드된 초기 포즈 값을 라디안으로 바꾸고, eigen matrix 에 id 개수만큼 열을 생성한다.
	}
}
void BaseModule::initPoseMsgCallback(const std_msgs::String::ConstPtr& msg) // GUI 에서 init pose topic을 sub 받아 실
{
	std::string init_pose_path = ros::package::getPath("base_module") + "/data/ini_pose.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	parse_init_pose_data_(init_pose_path); // YAML 파일 로드
	base_module_state->is_moving_state = true;
	ROS_INFO("FILE LOAD");
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

	//new_count_ ++;
	//// read current position ////
	if(new_count_ == 1)
	{
		ROS_INFO("Base_Process Start!");
		new_count_ ++;

		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
			if(gazebo_check == true)
				result_[joint_name]->goal_position_ = result_[joint_name]->present_position_; // 가제보 상 초기위치 0
			else
				result_[joint_name]->goal_position_ = dxls[joint_name]->dxl_state_->present_position_; // 다이나믹셀에서 읽어옴
			base_module_state->joint_ini_pose_state.coeffRef(joint_name_to_id_[joint_name], 0) = dxls[joint_name]->dxl_state_->present_position_; // 초기위치 저장
		} // 등록된 다이나믹셀의 위치값을 읽어와서 goal position 으로 입력
		ROS_INFO("Base module :: Read position and Send goal position");
	}
	// trajectory is working joint space control
	if(base_module_state->is_moving_state == false)
	{
		ROS_INFO("Base Stay");
	}
	else
	{
		ROS_INFO("Base Trajectory Start");

		// 허리 (9 , 10), 다리 (11 ~ 22), 머리 (23 ~ 25) 다이나믹셀 초기화
		for(int id=9 ; id<26 ; id++)
		{
			if(id == 9 || id == 10 || id == 11 || id == 17 || id == 19 || id == 23 || id == 24 || id == 25) // 방향 반대인 다이나믹셀
			{
				result_[joint_id_to_name_[id]]->goal_position_ = - motion_trajectory[id]->fifth_order_traj_gen(-base_module_state->joint_ini_pose_state(id,0),
						base_module_state->joint_ini_pose_goal(id,0),0,0,0,0,0,base_module_state->mov_time_state);
				if(gazebo_check == true)
					result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
				ROS_INFO("id :: %d , value %f", id , result_[joint_id_to_name_[id]]->goal_position_);
			}
			else
			{
				result_[joint_id_to_name_[id]]->goal_position_ =  motion_trajectory[id]->fifth_order_traj_gen(base_module_state->joint_ini_pose_state(id,0),
						base_module_state->joint_ini_pose_goal(id,0),0,0,0,0,0,base_module_state->mov_time_state);
				if(gazebo_check == true)
					result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
				ROS_INFO("id :: %d , value %f", id , result_[joint_id_to_name_[id]]->goal_position_);
			}
		}
		// 팔 다이나믹셀 초기화
		for(int id=1 ; id<7 ; id++)
		{
			if(id == 1 || id == 3 || id == 4 || id == 6) // 방향 반대인 다이나믹셀
			{
				result_[joint_id_to_name_[id]]->goal_position_ = - motion_trajectory[id]->fifth_order_traj_gen(-base_module_state->joint_ini_pose_state(id,0),
						base_module_state->joint_ini_pose_goal(id,0),0,0,0,0,0,base_module_state->mov_time_state);
				if(gazebo_check == true)
									result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
				ROS_INFO("id :: %d , value %f", id , result_[joint_id_to_name_[id]]->goal_position_);
			}
			else
			{
				result_[joint_id_to_name_[id]]->goal_position_ =  motion_trajectory[id]->fifth_order_traj_gen(base_module_state->joint_ini_pose_state(id,0),
						base_module_state->joint_ini_pose_goal(id,0),0,0,0,0,0,base_module_state->mov_time_state);
				if(gazebo_check == true)
									result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
				ROS_INFO("id :: %d , value %f", id , result_[joint_id_to_name_[id]]->goal_position_);
			}
		}

		//test
	/*	result_[joint_id_to_name_[10]]->goal_position_ =  -motion_trajectory[10]->fifth_order_traj_gen(-base_module_state->joint_ini_pose_state(10,0),
				base_module_state->joint_ini_pose_goal(10,0),0,0,0,0,0,base_module_state->mov_time_state);
		result_[joint_id_to_name_[9]]->goal_position_ =  -motion_trajectory[9]->fifth_order_traj_gen(-base_module_state->joint_ini_pose_state(9,0),
				base_module_state->joint_ini_pose_goal(9,0),0,0,0,0,0,base_module_state->mov_time_state);

		result_[joint_id_to_name_[17]]->goal_position_ =  -motion_trajectory[17]->fifth_order_traj_gen(-base_module_state->joint_ini_pose_state(17,0),
				base_module_state->joint_ini_pose_goal(17,0),0,0,0,0,0,base_module_state->mov_time_state);
		result_[joint_id_to_name_[18]]->goal_position_ =  motion_trajectory[18]->fifth_order_traj_gen(base_module_state->joint_ini_pose_state(18,0),
				base_module_state->joint_ini_pose_goal(18,0),0,0,0,0,0,base_module_state->mov_time_state);

		result_[joint_id_to_name_[23]]->goal_position_ =  motion_trajectory[23]->fifth_order_traj_gen(base_module_state->joint_ini_pose_state(23,0),
				base_module_state->joint_ini_pose_goal(23,0),0,0,0,0,0,base_module_state->mov_time_state);

		result_[joint_id_to_name_[1]]->goal_position_ = - motion_trajectory[1]->fifth_order_traj_gen(-base_module_state->joint_ini_pose_state(1,0),
				base_module_state->joint_ini_pose_goal(1,0),0,0,0,0,0,base_module_state->mov_time_state);
		result_[joint_id_to_name_[3]]->goal_position_ = - motion_trajectory[3]->fifth_order_traj_gen(-base_module_state->joint_ini_pose_state(3,0),
				base_module_state->joint_ini_pose_goal(3,0),0,0,0,0,0,base_module_state->mov_time_state);
		result_[joint_id_to_name_[5]]->goal_position_ =  motion_trajectory[5]->fifth_order_traj_gen(base_module_state->joint_ini_pose_state(5,0),
				base_module_state->joint_ini_pose_goal(5,0),0,0,0,0,0,base_module_state->mov_time_state);*/


		base_module_state->is_moving_state = motion_trajectory[10]->is_moving_traj;// trajectory end
	}
}
void BaseModule::stop()
{
	return;
}


