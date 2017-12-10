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
//	is_moving_l_ = false;
//	is_moving_r_ = false;
//	is_moving_one_joint_ = false;
	enable_       = false;
	module_name_  = "upper_body_module";
	control_mode_ = robotis_framework::PositionControl;

	// Dynamixel initialize ////
	result_["head_yaw"]   = new robotis_framework::DynamixelState(); // joint 23
	result_["waist_yaw"]  = new robotis_framework::DynamixelState(); // joint 9
	result_["waist_roll"] = new robotis_framework::DynamixelState(); // joint 10

	///////////////////////////
/*	l_kinematics_ = new heroehs_math::Kinematics;
	r_kinematics_ = new heroehs_math::Kinematics;
	end_to_rad_l_ = new heroehs_math::CalRad;
	end_to_rad_r_ = new heroehs_math::CalRad;
	one_joint_ = new heroehs_math::CalRad;*/
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
	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);
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
		ROS_INFO("Upper Start");
		new_count_ ++;
		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
			if(gazebo_check == true)
				result_[joint_name]->goal_position_ = result_[joint_name]->present_position_; // 가제보 상 초기위치 0
		} // 등록된 다이나믹셀의 위치값을 읽어와서 goal position 으로 입력

	}
/*	if(is_moving_l_ == false && is_moving_r_ == false && is_moving_one_joint_ == false) // desired pose
	{
		//ROS_INFO("Upper Stay");
	}
	else
	{
		//ROS_INFO("Upper Module Trajectory Start");


	}*/
	///////////////////////////////////////////////////// control //////////////////////////////////////////////////////////

  ROS_INFO("Upper Module Trajectory Start");
	result_[joint_id_to_name_[23]]->goal_position_ = 0;
	result_[joint_id_to_name_[9]]->goal_position_  = 0;
	result_[joint_id_to_name_[10]]->goal_position_ = 0;
	//<---  joint space control --->
	//result_[joint_id_to_name_[1]]->goal_position_ = 3.14;

	/*if(result_[joint_id_to_name_[1]]->goal_position_ - dxls["head"]->dxl_state_->present_position_ > 0.005)
		result_[joint_id_to_name_[1]]->goal_position_ = 3;// head
	else if (result_[joint_id_to_name_[1]]->goal_position_ - dxls["head"]->dxl_state_->present_position_ < 0.005)
		result_[joint_id_to_name_[1]]->goal_position_ = -3;// head*/


	//result_[joint_id_to_name_[10]]->goal_position_ = -result_rad_one_joint_; // waist roll

	//<---  cartesian space control  --->

	//result_[joint_id_to_name_[1]]->goal_position_ = 0;// head
	//result_[joint_id_to_name_[10]]->goal_position_ = -result_rad_one_joint_; // waist roll

}
void UpperBodyModule::stop()
{
	return;
}



