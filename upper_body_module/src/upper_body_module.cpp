/*
 * upper_body_module.cpp
 *
 *  Created on: Dec 04, 2017
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
//	result_["head_pitch"]   = new robotis_framework::DynamixelState(); // joint 24
//	result_["head_roll"]   = new robotis_framework::DynamixelState(); // joint 25
	///////////////////////////

	waist_kinematics_ = new heroehs_math::Kinematics;
	end_to_rad_waist_ = new heroehs_math::CalRad;

	head_kinematics_  = new heroehs_math::Kinematics;
	end_to_rad_head_  = new heroehs_math::CalRad;

	traj_time_test = 4;
	new_count_ = 1 ;

}
UpperBodyModule::~UpperBodyModule()
{
	queue_thread_.join();
}
void UpperBodyModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	control_cycle_msec_ = control_cycle_msec;
	queue_thread_ = boost::thread(boost::bind(&UpperBodyModule::queueThread, this));

	//new_count_ = 1;

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
			it != robot->dxls_.end(); it++)
	{
		std::string joint_name = it->first;
		robotis_framework::Dynamixel* dxl_info = it->second;

		joint_name_to_id_[joint_name] = dxl_info->id_;
		joint_id_to_name_[dxl_info->id_] = joint_name;
	}

	// waist yaw roll
	waist_end_point_.resize(6,8);
	waist_end_point_.fill(0);
	result_rad_waist_.resize(6,1);
	result_rad_waist_.fill(0);

	// head
	head_end_point_.resize(6,8);
	head_end_point_.fill(0);
	result_rad_head_.resize(6,1);
	result_rad_head_.fill(0);

	for(int joint_num_= 0; joint_num_< 6 ; joint_num_ ++)
	{
		waist_end_point_(joint_num_, 7) = traj_time_test;
		head_end_point_ (joint_num_, 7) = traj_time_test;
	}

	ROS_INFO("< -------  Initialize Module : Upper Body Module  [HEAD  && WAIST] !!  ------->");
}
void UpperBodyModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;
	ros_node.setCallbackQueue(&callback_queue);

	/* publisher topics
	state_end_point_pose_pub = ros_node.advertise<geometry_msgs::Vector3>("/state_end_point_pose",100);
	state_end_point_orientation_pub = ros_node.advertise<geometry_msgs::Vector3>("/state_end_point_orientation",100);
	zmp_point_pub = ros_node.advertise<geometry_msgs::PointStamped>("/zmp_point",100);
	zmp_point_pub_temp = ros_node.advertise<geometry_msgs::PointStamped>("/zmp_point_temp",100);

	 subscribe topics
	get_imu_data_sub_ = ros_node.subscribe("/imu/data", 100, &MotionModule::imuDataMsgCallback, this);
	get_ft_data_sub_ = ros_node.subscribe("/diana/force_torque_data", 100, &MotionModule::ftDataMsgCallback, this);

	// for gui
	set_balance_param_sub_ = ros_node.subscribe("/diana/balance_parameter", 5, &MotionModule::setBalanceParameterCallback, this);
	ros::Subscriber motion_num_msg_sub = ros_node.subscribe("/motion_num", 5, &MotionModule::desiredMotionMsgCallback, this);
	ros::Subscriber center_change_msg_sub = ros_node.subscribe("/diana/center_change", 5, &MotionModule::desiredCenterChangeMsgCallback, this);
	 */

	head_test = ros_node.subscribe("/desired_pose_head", 5, &UpperBodyModule::desiredPoseHeadMsgCallbackTEST, this);
	waist_test = ros_node.subscribe("/desired_pose_waist", 5, &UpperBodyModule::desiredPoseWaistMsgCallbackTEST, this);

	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);

}
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
bool UpperBodyModule::isRunning()
{
	return running_;
}
void UpperBodyModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
		std::map<std::string, double> sensors)
{

	if (enable_ == false)
	{
		return;
	}

	//// read current position ////
	if(new_count_ == 1)
	{
		new_count_ ++;
		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
			if(gazebo_check == true)
				result_[joint_name]->goal_position_ = result_[joint_name]->present_position_; // 가제보 상 초기위치 0
			ROS_INFO("Upper Start");
		} // 등록된 다이나믹셀의 위치값을 읽어와서 goal position 으로 입력

		result_rad_waist_ = end_to_rad_waist_ -> cal_end_point_to_rad(waist_end_point_);
		waist_kinematics_->XYZEulerAnglesSolution(result_rad_waist_ (5,0),0,result_rad_waist_ (3,0));

		result_rad_head_ = end_to_rad_head_   -> cal_end_point_to_rad(head_end_point_);
		head_kinematics_->ZYXEulerAnglesSolution(result_rad_head_(3,0),result_rad_head_(4,0),result_rad_head_(5,0));
	}
	if(is_moving_waist_ == false && is_moving_head_ == false) // desired pose
	{
		ROS_INFO("Upper Stay");
	}
	else
	{
		ROS_INFO("Upper Module Trajectory Start");

		result_rad_waist_ = end_to_rad_waist_ -> cal_end_point_to_rad(waist_end_point_);
		waist_kinematics_->XYZEulerAnglesSolution(result_rad_waist_(5,0),0,result_rad_waist_ (3,0));

		result_rad_head_ = end_to_rad_head_   -> cal_end_point_to_rad(head_end_point_);
		head_kinematics_->ZYXEulerAnglesSolution(result_rad_head_(3,0),result_rad_head_(4,0),result_rad_head_(5,0));

		is_moving_waist_ = end_to_rad_waist_ -> is_moving_check;
		is_moving_head_  = end_to_rad_head_  -> is_moving_check;
	}
	///////////////////////////////////////////////////// control //////////////////////////////////////////////////////////
	result_[joint_id_to_name_[9]]->goal_position_  = waist_kinematics_ -> xyz_euler_angle_z;// waist yaw
	result_[joint_id_to_name_[10]]->goal_position_ = waist_kinematics_ -> xyz_euler_angle_x; // waist roll


	result_["head_yaw"]->goal_position_ = head_kinematics_ -> zyx_euler_angle_z;
	//result_[joint_id_to_name_[24]]->goal_position_ = head_kinematics_->zyx_euler_angle_y;
	//result_[joint_id_to_name_[25]]->goal_position_ = head_kinematics_->zyx_euler_angle_x;
}
void UpperBodyModule::stop()
{
	return;
}



