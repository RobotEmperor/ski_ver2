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
	result_["head_pitch"]   = new robotis_framework::DynamixelState(); // joint 24
	result_["head_roll"]   = new robotis_framework::DynamixelState(); // joint 25

//	result_["waist_yaw"]  = new robotis_framework::DynamixelState(); // joint 9
//	result_["waist_roll"] = new robotis_framework::DynamixelState(); // joint 10

//	result_["head_yaw"]   = new robotis_framework::DynamixelState(); // joint 23

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
	end_to_rad_head_  = new heroehs_math::CalRad;

	traj_time_test = 4;
	new_count_ = 1 ;
	temp_waist_yaw_rad   = 0;
	temp_waist_roll_rad  = 0;

	// cop control variables
	cop_cal_waist = new  diana::CopCalculationFunc;
	currentFX_l = 0;
	currentFY_l = 0;
	currentFZ_l = 0;
	currentTX_l = 0;
	currentTY_l = 0;
	currentTZ_l = 0;
	currentFX_r = 0;
	currentFY_r = 0;
	currentFZ_r = 0;
	currentTX_r = 0;
	currentTY_r = 0;
	currentTZ_r = 0;

	// gyro control variables
  currentGyroX = 0;
	currentGyroY = 0;
	currentGyroZ = 0;
  tf_current_gyro_x = 0;
	tf_current_gyro_y = 0;
	tf_current_gyro_z = 0;

	//leg state
	l_leg_real_joint.resize(7,1);
	l_leg_real_joint.fill(0);
	r_leg_real_joint.resize(7,1);
	r_leg_real_joint.fill(0);
}
UpperBodyModule::~UpperBodyModule()
{
	queue_thread_.join();
}
void UpperBodyModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	control_cycle_msec_ = control_cycle_msec;
	queue_thread_ = boost::thread(boost::bind(&UpperBodyModule::queueThread, this));

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

	for(int joint_num_= 3; joint_num_< 6 ; joint_num_ ++)  // waist 3, 5번 // head 345 초기화
	{
		waist_end_point_(joint_num_, 7) = traj_time_test;
		head_end_point_ (joint_num_, 7) = traj_time_test;
	}
	waist_end_point_(4, 7) = 0;
	ROS_INFO("< -------  Initialize Module : Upper Body Module  [HEAD  && WAIST] !!  ------->");
}
void UpperBodyModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;
	ros_node.setCallbackQueue(&callback_queue);
	// publish topics
	current_waist_pose_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/current_waist_pose",100);
	cop_point_Fz_pub = ros_node.advertise<geometry_msgs::PointStamped>("/cop_point_Fz",100);
	cop_point_Fy_pub = ros_node.advertise<geometry_msgs::PointStamped>("/cop_point_Fy",100);
	cop_point_Fx_pub = ros_node.advertise<geometry_msgs::PointStamped>("/cop_point_Fx",100);

	// subscribe topics
	current_leg_pose_sub = ros_node.subscribe("/current_leg_pose", 5, &UpperBodyModule::currentLegPoseMsgCallback, this);
	get_imu_data_sub_ = ros_node.subscribe("/imu/data", 100, &UpperBodyModule::imuDataMsgCallback, this);
	get_ft_data_sub_ = ros_node.subscribe("/diana/force_torque_data", 100, &UpperBodyModule::ftDataMsgCallback, this);

	center_change_msg_sub = ros_node.subscribe("/diana/center_change", 5, &UpperBodyModule::desiredCenterChangeMsgCallback, this);

	// test desired pose
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
	//is_moving_waist_ = true;
}
void UpperBodyModule::desiredPoseHeadMsgCallbackTEST(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	head_end_point_(3, 1) = msg->data[0]; // yaw  트레젝토리 6 * 8 은 xyz yaw(z) pitch(y) roll(x) 이며 8은 처음 위치 나중 위치 / 속도 속도 / 가속도 가속도 / 시간 시간 / 임
	head_end_point_(4, 1) = msg->data[1];
	head_end_point_(5, 1) = msg->data[2]; // roll
	//is_moving_head_ = true;
}

void UpperBodyModule::currentLegPoseMsgCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	for(int joint_num = 1; joint_num<7; joint_num++)
	{
		l_leg_real_joint(joint_num, 0) = msg->data[joint_num - 1];
		r_leg_real_joint(joint_num, 0) = msg->data[joint_num + 5];
	}
}
void UpperBodyModule::imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg) // gyro data get
{
	currentGyroX = (double) msg->angular_velocity.x;
	currentGyroY = (double) msg->angular_velocity.y;
	currentGyroZ = (double) msg->angular_velocity.z;
	gyroRotationTransformation(currentGyroX, currentGyroY, currentGyroZ);
}
void UpperBodyModule::gyroRotationTransformation(double gyro_z, double gyro_y, double gyro_x)
{
	Eigen::MatrixXd tf_gyro_value;
	tf_gyro_value.resize(3,1);
	tf_gyro_value.fill(0);
	tf_gyro_value(0,0) =  gyro_x;
	tf_gyro_value(1,0) =  gyro_y;
	tf_gyro_value(2,0) =  gyro_z;

	tf_gyro_value = (robotis_framework::getRotationZ(M_PI/2)*robotis_framework::getRotationZ(-M_PI))*tf_gyro_value;
	tf_current_gyro_x = tf_gyro_value(0,0);
	tf_current_gyro_y = tf_gyro_value(1,0);
	tf_current_gyro_z = tf_gyro_value(2,0);
}
void UpperBodyModule::ftDataMsgCallback(const diana_msgs::ForceTorque::ConstPtr& msg)// force torque sensor data get
{
	currentFX_l =  (double) msg->force_x_raw_l;
	currentFY_l =  (double) msg->force_y_raw_l;
	currentFZ_l = -(double) msg->force_z_raw_l;

	currentTX_l =  (double) msg->torque_x_raw_l;
	currentTY_l =  (double) msg->torque_y_raw_l;
	currentTZ_l = -(double) msg->torque_z_raw_l;
	//		currentFX_r = (double) msg->force_x_raw_r;
	//		currentFY_r = (double) msg->force_y_raw_r;
	//		currentFZ_r = (double) msg->force_z_raw_r;

	currentFX_r = 1;
	currentFY_r = 1;
	currentFZ_r = 98.1;
	//		currentTX_r = (double) msg->torque_x_raw_r;
	//		currentTY_r = (double) msg->torque_y_raw_r;
	//		currentTZ_r = (double) msg->torque_z_raw_r;
	currentTX_r = 0;
	currentTY_r = 0;
	currentTZ_r = 0;

	cop_cal_waist->ftSensorDataLeftGet(currentFX_l, currentFY_l, currentFZ_l, currentTX_l, currentTY_l, currentTZ_l);
	cop_cal_waist->ftSensorDataRightGet(currentFX_r, currentFY_r, currentFZ_r, currentTX_r, currentTY_r, currentTZ_r);
}
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
		//is_moving_waist_ = false;
		//is_moving_head_ = false;
		new_count_ ++;
		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
			if(!joint_name.compare("waist_yaw")|| !joint_name.compare("waist_roll") || !joint_name.compare("head_yaw") ||
					!joint_name.compare("head_pitch") || !joint_name.compare("head_roll"))
			{
				if(gazebo_check == true)
					result_[joint_name]->goal_position_ = result_[joint_name]->present_position_; // 가제보 상 초기위치 0
			}
		} // 등록된 다이나믹셀의 위치값을 읽어와서 goal position 으로 입력
		ROS_INFO("Upper Start");
	}
	if(is_moving_waist_ == false && is_moving_head_ == false) // desired pose
	{
		ROS_INFO("Upper Stay");
	}
	else
	{
		ROS_INFO("Upper Module Trajectory Start");

		result_rad_waist_ = end_to_rad_waist_ -> cal_end_point_to_rad(waist_end_point_);
		result_rad_head_  = end_to_rad_head_  -> cal_end_point_to_rad(head_end_point_);

		is_moving_waist_ = end_to_rad_waist_ -> is_moving_check;
		is_moving_head_  = end_to_rad_head_  -> is_moving_check;
	}
	///////////////////////////////////////////////////// control //////////////////////////////////////////////////////////
	waist_kinematics_-> XYZEulerAnglesSolution(result_rad_waist_ (5,0),0,result_rad_waist_ (3,0));
	head_kinematics_ -> ZYXEulerAnglesSolution(result_rad_head_(3,0),result_rad_head_(4,0),result_rad_head_(5,0));

	// real test
	result_[joint_id_to_name_[9]] -> goal_position_  = waist_kinematics_ -> xyz_euler_angle_z;// waist yaw
	result_[joint_id_to_name_[10]]-> goal_position_  = waist_kinematics_ -> xyz_euler_angle_x; // waist roll


	result_[joint_id_to_name_[23]]-> goal_position_ = head_kinematics_ -> zyx_euler_angle_z;
	//gazebo
/*	result_[joint_id_to_name_[24]]-> goal_position_ = head_kinematics_ -> zyx_euler_angle_y;
	result_[joint_id_to_name_[25]]-> goal_position_ = head_kinematics_ -> zyx_euler_angle_x;*/

	//arm module transmitted
	temp_waist_yaw_rad   =  dxls["waist_yaw"] -> dxl_state_->present_position_;
	temp_waist_roll_rad  = -dxls["waist_roll"]-> dxl_state_->present_position_; // direction must define!

	// compensation control Algorithm //




	// cop
	cop_cal_waist->jointStateGetForTransForm(l_leg_real_joint, r_leg_real_joint);
	cop_cal_waist->copCalculationResult();


	// display cop to rviz
	cop_point_Fz_msg_.header.stamp = ros::Time();
	cop_point_Fy_msg_.header.stamp = ros::Time();
	cop_point_Fx_msg_.header.stamp = ros::Time();

	std::string frame = "/pelvis";
	cop_point_Fz_msg_.header.frame_id = frame.c_str();
	cop_point_Fy_msg_.header.frame_id = frame.c_str();
	cop_point_Fx_msg_.header.frame_id = frame.c_str();

	cop_point_Fz_msg_.point.x = cop_cal_waist -> cop_fz_point_x;
	cop_point_Fz_msg_.point.y = cop_cal_waist -> cop_fz_point_y;
	cop_point_Fz_msg_.point.z = 0;
	cop_point_Fz_pub.publish(cop_point_Fz_msg_);

	cop_point_Fy_msg_.point.x = cop_cal_waist -> cop_fy_point_x;
	cop_point_Fy_msg_.point.y = 0;
	cop_point_Fy_msg_.point.z = cop_cal_waist -> cop_fy_point_z;
	cop_point_Fy_pub.publish(cop_point_Fy_msg_);

	cop_point_Fx_msg_.point.x = 0;
	cop_point_Fx_msg_.point.y = cop_cal_waist -> cop_fx_point_y;
	cop_point_Fx_msg_.point.z = cop_cal_waist -> cop_fx_point_z;
	cop_point_Fx_pub.publish(cop_point_Fx_msg_);


	current_waist_pose_msg.data.push_back(temp_waist_yaw_rad);
	current_waist_pose_msg.data.push_back(temp_waist_roll_rad);
	current_waist_pose_pub.publish(current_waist_pose_msg);
	current_waist_pose_msg.data.clear();
}
void UpperBodyModule::stop()
{
	return;
}



