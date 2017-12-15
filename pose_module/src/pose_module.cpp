/*
 * pose_module.cpp
 *
 *  Created on: Oct 24, 2017
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

	/*
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
	 */
	result_["waist_yaw"]        = new robotis_framework::DynamixelState();  // joint 9
	result_["waist_roll"]       = new robotis_framework::DynamixelState();  // joint 10

	result_["head_yaw"]         = new robotis_framework::DynamixelState();  // joint 23

	result_["l_ankle_pitch"]    = new robotis_framework::DynamixelState();  // joint 19
	result_["r_ankle_pitch"]     = new robotis_framework::DynamixelState();  // joint 20

	result_["l_shoulder_pitch"] = new robotis_framework::DynamixelState();  // joint 1
	result_["l_shoulder_roll"]  = new robotis_framework::DynamixelState();  // joint 3
	result_["l_elbow_pitch"]    = new robotis_framework::DynamixelState();  // joint 5

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

	traj_time_ = 4.0;
	////
	p_gain_adjust_check = false;
	id_select = 23;
}
PoseModule::~PoseModule()
{
	queue_thread_.join();
}
void PoseModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	control_cycle_msec_ = control_cycle_msec;
	queue_thread_ = boost::thread(boost::bind(&PoseModule::queueThread, this));

	new_count_ = 1;

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
			it != robot->dxls_.end(); it++)
	{
		std::string joint_name = it->first;
		robotis_framework::Dynamixel* dxl_info = it->second;

		joint_name_to_id_[joint_name] = dxl_info->id_;
		joint_id_to_name_[dxl_info->id_] = joint_name;
		parsePgainValue(joint_name);
	}
	leg_end_point_l_.resize(6,8);
	leg_end_point_l_.fill(0);
	leg_end_point_l_(1,0) = 0.105; // y 초기값
	leg_end_point_l_(1,1) = 0.105;
	leg_end_point_l_(2,0) = -0.55; // z 초기값
	leg_end_point_l_(2,1) = -0.55;
	end_to_rad_l_->cal_end_point_tra_py->current_pose = 0.105;
	end_to_rad_l_->current_pose_change(1,0) = 0.105;
	end_to_rad_l_->cal_end_point_tra_pz->current_pose = -0.55;
	end_to_rad_l_->current_pose_change(2,0) = -0.55;

	leg_end_point_r_.resize(6,8);
	leg_end_point_r_.fill(0);
	leg_end_point_r_(1,0) = -0.105;  // 초기값
	leg_end_point_r_(1,1) = -0.105;
	leg_end_point_r_(2,0) = -0.55;  // 초기값
	leg_end_point_r_(2,1) = -0.55;
	end_to_rad_r_->cal_end_point_tra_py->current_pose = -0.105;
	end_to_rad_r_->current_pose_change(1,0) = -0.105;
	end_to_rad_r_->cal_end_point_tra_pz->current_pose = -0.55;
	end_to_rad_r_->current_pose_change(2,0) = -0.55;

	result_end_l_.resize(6,1);
	result_end_r_.resize(6,1);
	result_end_l_.fill(0);
	result_end_r_.fill(0);

	// waist yaw roll
	waist_end_point_.resize(6,8);
	waist_end_point_.fill(0);
	result_end_waist_.resize(6,1);
	result_end_waist_.fill(0);

	// head
	head_end_point_.resize(6,8);
	head_end_point_.fill(0);
	result_end_head_.resize(6,1);
	result_end_head_.fill(0);

	// arm
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

	/*one_joint_ctrl_.resize(1,8);
	one_joint_ctrl_.fill(0);
	one_joint_ctrl_(0,1) = 0;
	result_rad_one_joint_ = 0;*/

	for(int joint_num_= 0; joint_num_< 6 ; joint_num_ ++)
	{
		leg_end_point_l_(joint_num_, 7)  = traj_time_;
		leg_end_point_r_(joint_num_, 7)  = traj_time_;
		waist_end_point_(joint_num_, 7)  = traj_time_;
		head_end_point_ (joint_num_, 7)  = traj_time_;
		l_arm_end_point_ (joint_num_, 7) = traj_time_;
		r_arm_end_point_ (joint_num_, 7) = traj_time_;
	}
	//one_joint_ctrl_(0,7) = traj_time_;

	ROS_INFO("< -------  Initialize Module : Pose Module !!  ------->");
}
void PoseModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;

	ros_node.setCallbackQueue(&callback_queue);

	/* subscribe topics */
	// for gui
	ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/desired_pose_leg", 5, &PoseModule::desiredPoseMsgCallback, this);
	ros::Subscriber desired_pose_waist_sub = ros_node.subscribe("/desired_pose_waist", 5, &PoseModule::desiredPoseWaistMsgCallback, this);
	ros::Subscriber desired_pose_head_sub = ros_node.subscribe("/desired_pose_head", 5, &PoseModule::desiredPoseHeadMsgCallback, this);
	ros::Subscriber desired_pose_arm_sub = ros_node.subscribe("/desired_pose_arm", 5, &PoseModule::desiredPoseArmMsgCallback, this);

	ros::Subscriber gain_adjustment_sub = ros_node.subscribe("/gain_adjustment", 5, &PoseModule::gainAdjustmentMsgCallback, this);
	ros::Subscriber final_gain_save_sub = ros_node.subscribe("/final_gain_save", 5, &PoseModule::finalGainSaveMsgCallback, this);

	read_p_gain_value_srv = ros_node.advertiseService("/read_p_gain_value", &PoseModule::readPgainSrvFunction, this);

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

	l_arm_end_point_(0, 1) = msg->data[0]; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	l_arm_end_point_(1, 1) = msg->data[1];
	l_arm_end_point_(2, 1) = msg->data[2]; // roll

	r_arm_end_point_(0, 1) = msg->data[3]; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	r_arm_end_point_(1, 1) = msg->data[4];
	r_arm_end_point_(2, 1) = msg->data[5];

	is_moving_l_arm = true;
	is_moving_r_arm = true;

	printf("Receive the value!!!");
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
void PoseModule::parsePgainValue(std::string joint_name_)
{
	std::string path_ = ros::package::getPath("ski_main_manager") + "/config/dxl_init.yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
	YAML::Node doc; // YAML file class 선언!
	try
	{
		// load yaml
		doc = YAML::LoadFile(path_.c_str()); // 파일 경로를 입력하여 파일을 로드 한다.
	}catch(const std::exception& e) // 에러 점검
	{
		ROS_ERROR("Fail to load yaml file!");
		return;
	}
	YAML::Node pose_node = doc[joint_name_];
	for (YAML::iterator it = pose_node.begin(); it != pose_node.end(); ++it) //motion_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		std::string standard;
		int p_gain_ = 0;
		standard = it->first.as<std::string>();
		if(!standard.compare("position_p_gain"))
			p_gain_data_[joint_name_to_id_[joint_name_]] = it->second.as<int>();
	}
}
void PoseModule::savePgainValue()
{
	// YAML SAVE
	ROS_INFO("YAML_SAVE");
	YAML::Emitter yaml_out;
	std::map<std::string, double> dxl_init_data;

	dxl_init_data["return_delay_time"] = 10;

	yaml_out << YAML::BeginMap;

	for(int id=1; id<26; id++)
	{
		if(id == 7 || id == 8)
			printf("There is not id");
		else{
			dxl_init_data["position_p_gain"] = p_gain_data_[id];
			yaml_out << YAML::Key << joint_id_to_name_[id] << YAML::Value << dxl_init_data;
		}
	}
	yaml_out << YAML::EndMap;

	std::string dxl_init_file_path = ros::package::getPath("ski_main_manager") + "/config/dxl_init.yaml";
	std::ofstream fout(dxl_init_file_path.c_str());
	fout << yaml_out.c_str();  // dump it back into the file
}
void PoseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
		std::map<std::string, double> sensors)
{
	if (enable_ == false)
	{
		return;
	}
	//// read current position ////
	if(new_count_ == 1)
	{
		ROS_INFO("Pose_Process Start!");
		new_count_ ++;
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
		ROS_INFO("Pose module :: Read position and Send goal position");

		result_end_l_ = end_to_rad_l_         -> cal_end_point_to_rad(leg_end_point_l_);
		result_end_r_ = end_to_rad_r_         -> cal_end_point_to_rad(leg_end_point_r_);
		result_end_waist_ = end_to_rad_waist_ -> cal_end_point_to_rad(waist_end_point_);
		result_end_head_ = end_to_rad_head_   -> cal_end_point_to_rad(head_end_point_);
		result_end_l_arm_ = end_to_rad_l_arm_ -> cal_end_point_to_rad(l_arm_end_point_);
		result_end_r_arm_ = end_to_rad_r_arm_ -> cal_end_point_to_rad(r_arm_end_point_);

		//result_rad_one_joint_ = one_joint_ -> cal_one_joint_rad(one_joint_ctrl_);
	}

	if(is_moving_l_ == false && is_moving_r_ == false && is_moving_waist == false && is_moving_head == false && is_moving_l_arm == false && is_moving_r_arm == false)
	{
		if(p_gain_adjust_check == true)
		{
			result_[joint_id_to_name_[id_select]]->position_p_gain_ = p_gain_data_[id_select]; // 움직이지 않을때 게인 값 조정
			printf("id :: %d value :: %d adjust!", id_select, p_gain_data_[id_select]);
			p_gain_adjust_check = false;
		}
		ROS_INFO("Pose Stay");
	}
	else
	{
		ROS_INFO("Pose Trajectory Start");

		// trajectory is working cartesian space control LEG
		result_end_l_ = end_to_rad_l_         -> cal_end_point_to_rad(leg_end_point_l_);
		result_end_r_ = end_to_rad_r_         -> cal_end_point_to_rad(leg_end_point_r_);
		// trajectory is working cartesian space control WAIST
		result_end_waist_ = end_to_rad_waist_ -> cal_end_point_to_rad(waist_end_point_);
		// trajectory is working cartesian space control HEAD
		result_end_head_ = end_to_rad_head_   -> cal_end_point_to_rad(head_end_point_);
		// trajectory is working cartesian space control ARM
		result_end_l_arm_ = end_to_rad_l_arm_ -> cal_end_point_to_rad(l_arm_end_point_);
		result_end_r_arm_ = end_to_rad_r_arm_ -> cal_end_point_to_rad(r_arm_end_point_);
		/*//<---  read   --->
		for(int id=10 ; id<23 ; id++)
		{
			if(gazebo_check == true)
				result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
			else
			{
				result_[joint_id_to_name_[id]]->present_position_ = dxls[joint_id_to_name_[id]]->dxl_state_->present_position_; // real robot
			}
		}*/

		/*
		cop_cal.ftSensorDataLeftGet(0, 0, 98.1, 0, 0, 0);
		cop_cal.ftSensorDataRightGet(0, 0, 98.1, 0, 0, 0);
		cop_cal.jointStateGetForTransForm(l_kinematics_->joint_radian, r_kinematics_->joint_radian);
		cop_cal.copCalculationResult();
		 */
		is_moving_l_     = end_to_rad_l_      -> is_moving_check;
		is_moving_r_     = end_to_rad_r_      -> is_moving_check;
		is_moving_waist  = end_to_rad_waist_  -> is_moving_check;
		is_moving_head   = end_to_rad_head_   -> is_moving_check;
		is_moving_l_arm  = end_to_rad_l_arm_  -> is_moving_check;
		is_moving_r_arm  = end_to_rad_r_arm_  -> is_moving_check;
	}

	//cartesian space control LEG
	l_kinematics_->InverseKinematics(result_end_l_(0,0), result_end_l_(1,0) - 0.105, result_end_l_(2,0), result_end_l_(3,0), result_end_l_(4,0), result_end_l_(5,0)); // pX pY pZ alpha betta kamma
	r_kinematics_->InverseKinematics(result_end_r_(0,0), result_end_r_(1,0) + 0.105, result_end_r_(2,0), result_end_r_(3,0), result_end_r_(4,0), result_end_r_(5,0)); // pX pY pZ alpha betta kamma
	//cartesian space control WAIST
	waist_kinematics_->XYZEulerAnglesSolution(result_end_waist_ (5,0) , 0, result_end_waist_ (3,0));
	//cartesian space control HEAD
	head_kinematics_->ZYXEulerAnglesSolution(result_end_head_(3,0),result_end_head_(4,0),result_end_head_(5,0));
	//cartesian space control ARM
	l_arm_kinematics_ ->InverseKinematicsArm(result_end_l_arm_(0,0), result_end_l_arm_(1,0), result_end_l_arm_(2,0));
	r_arm_kinematics_ ->InverseKinematicsArm(result_end_r_arm_(0,0), result_end_r_arm_(1,0), result_end_r_arm_(2,0));


	//<---  catesian space control test --->
	result_[joint_id_to_name_[9]]  -> goal_position_  = waist_kinematics_->xyz_euler_angle_z;// waist yaw
	result_[joint_id_to_name_[10]] -> goal_position_ = waist_kinematics_->xyz_euler_angle_x; // waist roll

	result_[joint_id_to_name_[19]] -> goal_position_ = -l_kinematics_->joint_radian(5,0);
	result_[joint_id_to_name_[20]] -> goal_position_ = r_kinematics_->joint_radian(5,0);

	result_[joint_id_to_name_[23]] -> goal_position_ = head_kinematics_->zyx_euler_angle_z;

	result_[joint_id_to_name_[1]]  -> goal_position_ = l_arm_kinematics_->joint_radian(1,0);
	result_[joint_id_to_name_[3]]  -> goal_position_ = l_arm_kinematics_->joint_radian(2,0);
	result_[joint_id_to_name_[5]]  -> goal_position_ = l_arm_kinematics_->joint_radian(3,0);


	//<---  cartesian space control  --->
	/*


	result_[joint_id_to_name_[9]]->goal_position_  = waist_kinematics_->xyz_euler_angle_z;// waist yaw
	result_[joint_id_to_name_[10]]->goal_position_   = waist_kinematics_->xyz_euler_angle_x; // waist roll

	result_[joint_id_to_name_[11]]->goal_position_   = -l_kinematics_->joint_radian(1,0);
	result_[joint_id_to_name_[13]]->goal_position_   =  l_kinematics_->joint_radian(2,0);
	result_[joint_id_to_name_[15]]->goal_position_   =  l_kinematics_->joint_radian(3,0);

	result_[joint_id_to_name_[17]]->goal_position_   = -l_kinematics_->joint_radian(4,0);
	result_[joint_id_to_name_[19]]->goal_position_   = -l_kinematics_->joint_radian(5,0);
	result_[joint_id_to_name_[21]]->goal_position_   =  l_kinematics_->joint_radian(6,0);

	result_[joint_id_to_name_[12]]->goal_position_   =  r_kinematics_->joint_radian(1,0);
	result_[joint_id_to_name_[14]]->goal_position_   =  r_kinematics_->joint_radian(2,0);
	result_[joint_id_to_name_[16]]->goal_position_   =  r_kinematics_->joint_radian(3,0);

	result_[joint_id_to_name_[18]]->goal_position_   =  r_kinematics_->joint_radian(4,0);
	result_[joint_id_to_name_[20]]->goal_position_   =  r_kinematics_->joint_radian(5,0);
	result_[joint_id_to_name_[22]]->goal_position_   =  r_kinematics_->joint_radian(6,0);

	result_[joint_id_to_name_[23]]->goal_position_   = head_kinematics_->zyx_euler_angle_z;
	result_[joint_id_to_name_[24]]->goal_position_   = head_kinematics_->zyx_euler_angle_y;
	result_[joint_id_to_name_[25]]->goal_position_   = head_kinematics_->zyx_euler_angle_x;
	 */

}
void PoseModule::stop()
{
	return;
}


