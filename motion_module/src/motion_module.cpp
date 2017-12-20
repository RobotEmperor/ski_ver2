/*
 * motion_module.cpp
 *
 *  Created on: Oct 27, 2017
 *      Author: robotemperor
 */
#include <stdio.h>
#include "motion_module/motion_module.h"

using namespace motion_module;
using namespace diana;

MotionModule::MotionModule()
: control_cycle_msec_(8)
{
	running_ = false;
	gazebo_check = false;
	is_moving_l_ = false;
	is_moving_r_ = false;
	is_moving_one_joint_ = false;
	enable_       = false;
	module_name_  = "motion_module";
	control_mode_ = robotis_framework::PositionControl;

	new_count_ = 0;
	// Dynamixel initialize ////




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


	// test
	/*
		result_["l_ankle_pitch"] = new robotis_framework::DynamixelState();  // joint 19
		result_["r_ankle_pitch"] = new robotis_framework::DynamixelState();  // joint 20
	 */
	///////////////////////////
	l_kinematics_ = new heroehs_math::Kinematics;
	r_kinematics_ = new heroehs_math::Kinematics;
	end_to_rad_l_ = new heroehs_math::CalRad;
	end_to_rad_r_ = new heroehs_math::CalRad;

	//////////////////////////
	currentGyroX = 0;
	currentGyroY = 0;
	currentGyroZ = 0;
	//////////////////////////
	traj_time_ = 4.0;

	result_end_l_.resize(6,1);
	result_end_r_.resize(6,1);
	result_end_l_.fill(0);
	result_end_r_.fill(0);

	result_end_l_.coeffRef(0,0) = 0.1;
	result_end_l_.coeffRef(1,0) = 0.255;
	result_end_l_.coeffRef(2,0) = -0.51;
	result_end_l_.coeffRef(3,0) = -15*DEGREE2RADIAN;
	result_end_l_.coeffRef(4,0) = -10*DEGREE2RADIAN;
	result_end_l_.coeffRef(5,0) = 15*DEGREE2RADIAN;

	result_end_r_.coeffRef(0,0) = 0.1;
	result_end_r_.coeffRef(1,0) = -0.255;
	result_end_r_.coeffRef(2,0) = -0.51;
	result_end_r_.coeffRef(3,0) = 15*DEGREE2RADIAN;
	result_end_r_.coeffRef(4,0) = -10*DEGREE2RADIAN;
	result_end_r_.coeffRef(5,0) = -15*DEGREE2RADIAN;

	result_mat_cob_ = result_mat_cob_modified_ = Eigen::Matrix4d::Identity();

	result_mat_l_ = robotis_framework::getTransformationXYZRPY(result_end_l_.coeff(0,0), result_end_l_.coeff(1,0), result_end_l_.coeff(2,0),
			result_end_l_.coeff(3,0), result_end_l_.coeff(4,0), result_end_l_.coeff(5,0));
	result_mat_r_ = robotis_framework::getTransformationXYZRPY(result_end_r_.coeff(0,0), result_end_r_.coeff(1,0), result_end_r_.coeff(2,0),
			result_end_r_.coeff(3,0), result_end_r_.coeff(4,0), result_end_r_.coeff(5,0));

	result_mat_l_modified_ = result_mat_l_;
	result_mat_r_modified_ = result_mat_r_;

	balance_updating_duration_sec_ = 2.0;
	balance_updating_sys_time_sec_ = 2.0;
	balance_update_= false;
	tf_current_gyro_x = 0;
	tf_current_gyro_y = 0;
	tf_current_gyro_z = 0;

	center_change_ = new diana_motion::CenterChange;
	cop_cal = new  diana::CopCalculationFunc;
	temp_change_value_center = 0;
	temp_change_value_edge = 0;
	temp_turn_type = "basic";
	temp_change_type = "basic";
	currentFX_l=0.0;
	currentFY_l=0.0;
	currentFZ_l=0.0;
	currentTX_l=0.0;
	currentTY_l=0.0;
	currentTZ_l=0.0;
	currentFX_r=0.0;
	currentFY_r=0.0;
	currentFZ_r=0.0;
	currentTX_r=0.0;
	currentTY_r=0.0;
	currentTZ_r=0.0;
}
MotionModule::~MotionModule()
{
	queue_thread_.join();
}
void MotionModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	control_cycle_msec_ = control_cycle_msec;
	queue_thread_ = boost::thread(boost::bind(&MotionModule::queueThread, this));

	new_count_ = 1;

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
			it != robot->dxls_.end(); it++)
	{
		std::string joint_name = it->first;
		robotis_framework::Dynamixel* dxl_info = it->second;

		joint_name_to_id_[joint_name] = dxl_info->id_;
		joint_id_to_name_[dxl_info->id_] = joint_name;
	}
	//초기화 //
	leg_end_point_l_.resize(6,8);
	leg_end_point_l_.fill(0);
	leg_end_point_l_(0,0) = 0.1;
	leg_end_point_l_(1,0) = 0.255;
	leg_end_point_l_(2,0) = -0.51;
	leg_end_point_l_(3,0) = -15*DEGREE2RADIAN;
	leg_end_point_l_(4,0) = -10*DEGREE2RADIAN;
	leg_end_point_l_(5,0) = 15*DEGREE2RADIAN;
	for(int i=0;i<6;i++)
	{
		leg_end_point_l_(i,1) = leg_end_point_l_(i,0); // final value
	}
	end_to_rad_l_->cal_end_point_tra_px->current_pose = 0.1;
	end_to_rad_l_->cal_end_point_tra_py->current_pose = 0.255;
	end_to_rad_l_->cal_end_point_tra_pz->current_pose = -0.51;

	end_to_rad_l_->cal_end_point_tra_alpha->current_pose = -15*DEGREE2RADIAN;
	end_to_rad_l_->cal_end_point_tra_betta->current_pose = -10*DEGREE2RADIAN;
	end_to_rad_l_->cal_end_point_tra_kamma->current_pose = 15*DEGREE2RADIAN;

	end_to_rad_l_->current_pose_change(0,0) = 0.1;
	end_to_rad_l_->current_pose_change(1,0) = 0.255;
	end_to_rad_l_->current_pose_change(2,0) = -0.51;

	end_to_rad_l_->current_pose_change(3,0) = -15*DEGREE2RADIAN;
	end_to_rad_l_->current_pose_change(4,0) = -10*DEGREE2RADIAN;
	end_to_rad_l_->current_pose_change(5,0) = -15*DEGREE2RADIAN;

	leg_end_point_r_.resize(6,8);
	leg_end_point_r_.fill(0);

	leg_end_point_r_(0,0) = 0.1;
	leg_end_point_r_(1,0) = -0.255;
	leg_end_point_r_(2,0) = -0.51;
	leg_end_point_r_(3,0) = 15*DEGREE2RADIAN;
	leg_end_point_r_(4,0) = -10*DEGREE2RADIAN;
	leg_end_point_r_(5,0) = -15*DEGREE2RADIAN;

	for(int i=0;i<6;i++)
	{
		leg_end_point_r_(i,1) = leg_end_point_r_(i,0);// final value
	}
	end_to_rad_r_->cal_end_point_tra_px->current_pose = 0.1;
	end_to_rad_r_->cal_end_point_tra_py->current_pose = -0.255;
	end_to_rad_r_->cal_end_point_tra_pz->current_pose = -0.51;

	end_to_rad_r_->cal_end_point_tra_alpha->current_pose = 15*DEGREE2RADIAN;
	end_to_rad_r_->cal_end_point_tra_betta->current_pose = -10*DEGREE2RADIAN;
	end_to_rad_r_->cal_end_point_tra_kamma->current_pose = -15*DEGREE2RADIAN;

	end_to_rad_r_->current_pose_change(2,0) = 0.1;
	end_to_rad_r_->current_pose_change(2,0) = -0.255;
	end_to_rad_r_->current_pose_change(2,0) = -0.51;

	end_to_rad_r_->current_pose_change(2,0) = 15*DEGREE2RADIAN;
	end_to_rad_r_->current_pose_change(2,0) = -10*DEGREE2RADIAN;
	end_to_rad_r_->current_pose_change(2,0) = -15*DEGREE2RADIAN;

	/*	one_joint_ctrl_.resize(1,8);
	one_joint_ctrl_.fill(0);
	one_joint_ctrl_(0,1) = 0;
	result_rad_one_joint_ = 0;*/

	for(int joint_num_=0; joint_num_< 6 ; joint_num_ ++)
	{
		leg_end_point_l_(joint_num_, 7) = traj_time_;
		leg_end_point_r_(joint_num_, 7) = traj_time_;
	}
	//one_joint_ctrl_(0,7) = traj_time_;
	//
	balance_ctrl_.initialize(control_cycle_msec);
	previous_balance_param_.cob_x_offset_m = 0;
	previous_balance_param_.cob_y_offset_m = 0;
	previous_balance_param_.foot_roll_gyro_p_gain = 0;
	previous_balance_param_.foot_roll_gyro_d_gain = 0;
	previous_balance_param_.foot_pitch_gyro_p_gain = 0;
	previous_balance_param_.foot_pitch_gyro_d_gain = 0;

	tf_current_gyro_x = 0;
	tf_current_gyro_y = 0;
	tf_current_gyro_z = 0;

	ROS_INFO("< -------  Initialize Module : Motion Module !!  ------->");
}
void MotionModule::queueThread()
{
	ros::NodeHandle ros_node;
	ros::CallbackQueue callback_queue;
	ros_node.setCallbackQueue(&callback_queue);
	/* publisher topics */
	state_end_point_pose_pub = ros_node.advertise<geometry_msgs::Vector3>("/state_end_point_pose",100);
	state_end_point_orientation_pub = ros_node.advertise<geometry_msgs::Vector3>("/state_end_point_orientation",100);
	current_leg_pose_pub = ros_node.advertise<std_msgs::Float64MultiArray>("/current_leg_pose",100);

	/* subscribe topics */
	get_imu_data_sub_ = ros_node.subscribe("/imu/data", 100, &MotionModule::imuDataMsgCallback, this);
	get_ft_data_sub_ = ros_node.subscribe("/diana/force_torque_data", 100, &MotionModule::ftDataMsgCallback, this);

	// for gui
	set_balance_param_sub_ = ros_node.subscribe("/diana/balance_parameter", 5, &MotionModule::setBalanceParameterCallback, this);
	ros::Subscriber center_change_msg_sub = ros_node.subscribe("/diana/center_change", 5, &MotionModule::desiredCenterChangeMsgCallback, this);

	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);
}
bool MotionModule::isRunning()
{
	return running_;
}
void MotionModule::desiredCenterChangeMsgCallback(const diana_msgs::CenterChange::ConstPtr& msg) // GUI 에서 motion_num topic을 sub 받아 실행 모션 번호 디텍트
{
	is_moving_l_ = true;
	is_moving_r_ = true;

	if (temp_change_value_center != msg->center_change || temp_change_value_edge != msg->edge_change|| temp_turn_type.compare(msg->turn_type) || temp_change_type.compare(msg->change_type))
	{
		center_change_->parseMotionData(msg->turn_type, msg->change_type);

		if(!msg->change_type.compare("edge_change"))
			center_change_->calculateStepEndPointValue(msg->edge_change,100,msg->change_type); // 0.01 단위로 조정 가능.
		else
			center_change_->calculateStepEndPointValue(msg->center_change,100,msg->change_type); // 0.01 단위로 조정 가능.

		for(int m = 0 ; m<6 ; m++)
		{
			leg_end_point_l_(m,1) = center_change_->step_end_point_value[0][m];
			leg_end_point_r_(m,1) = center_change_->step_end_point_value[1][m];
			leg_end_point_l_(m,7) = msg->time_change;
			leg_end_point_r_(m,7) = msg->time_change;
		}
		temp_change_value_center = msg->center_change;
		temp_change_value_edge = msg->edge_change;
		temp_turn_type    = msg->turn_type;
		temp_change_type  = msg->change_type; // 이전값 저장
		ROS_INFO("Turn !!  Change");
	} // 변한 것이 있으면 값을 계산
	else
	{  ROS_INFO("Nothing to change");
	return;
	}
}
void MotionModule::imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg) // gyro data get
{
	currentGyroX = (double) msg->angular_velocity.x;
	currentGyroY = (double) msg->angular_velocity.y;
	currentGyroZ = (double) msg->angular_velocity.z;
	gyroRotationTransformation(currentGyroX, currentGyroY, currentGyroZ);
	balance_ctrl_.setCurrentGyroSensorOutput(tf_current_gyro_x, tf_current_gyro_y);
}
void MotionModule::gyroRotationTransformation(double gyro_z, double gyro_y, double gyro_x)
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
void MotionModule::ftDataMsgCallback(const diana_msgs::ForceTorque::ConstPtr& msg)// force torque sensor data get
{
	currentFX_l = (double) msg->force_x_raw_l;
	currentFY_l = (double) msg->force_y_raw_l;
	currentFZ_l = -(double) msg->force_z_raw_l;


	currentTX_l = (double) msg->torque_x_raw_l;
	currentTY_l = (double) msg->torque_y_raw_l;
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

	cop_cal->ftSensorDataLeftGet(currentFX_l, currentFY_l, currentFZ_l, currentTX_l, currentTY_l, currentTZ_l);
	cop_cal->ftSensorDataRightGet(currentFX_r, currentFY_r, currentFZ_r, currentTX_r, currentTY_r, currentTZ_r);
}
void MotionModule::setBalanceParameterCallback(const diana_msgs::BalanceParam::ConstPtr& msg)
{
	if(balance_update_ == true)
	{
		ROS_ERROR("the previous task is not finished");
		return;
	}

	ROS_INFO("SET BALANCE_PARAM");

	balance_updating_duration_sec_ = 2.0;
	if(msg->updating_duration < 0)
		balance_updating_duration_sec_ = 2.0;
	else
		balance_updating_duration_sec_ = msg->updating_duration;

	desired_balance_param_.cob_x_offset_m         = msg->cob_x_offset_m        ;
	desired_balance_param_.cob_y_offset_m         = msg->cob_y_offset_m        ;
	desired_balance_param_.foot_roll_gyro_p_gain  = msg->foot_roll_gyro_p_gain ;
	desired_balance_param_.foot_roll_gyro_d_gain  = msg->foot_roll_gyro_d_gain ;
	desired_balance_param_.foot_pitch_gyro_p_gain = msg->foot_pitch_gyro_p_gain;
	desired_balance_param_.foot_pitch_gyro_d_gain = msg->foot_pitch_gyro_d_gain;

	balance_param_update_coeff_.changeTrajectory(0,0,0,0, balance_updating_duration_sec_, 1, 0, 0);

	balance_update_ = true;
	balance_updating_sys_time_sec_ = 0;
}

void MotionModule::updateBalanceParameter()
{
	if(balance_update_ == false)
		return;

	double coeff = 0.0;
	balance_updating_sys_time_sec_ += control_cycle_msec_ * 0.001;

	if(balance_updating_sys_time_sec_ > balance_updating_duration_sec_)
	{
		balance_updating_sys_time_sec_ = balance_updating_duration_sec_;

		balance_ctrl_.setCOBManualAdjustment(desired_balance_param_.cob_x_offset_m, desired_balance_param_.cob_y_offset_m, 0);
		balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_ = desired_balance_param_.foot_roll_gyro_p_gain;
		balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_ = desired_balance_param_.foot_roll_gyro_d_gain;

		balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_ = desired_balance_param_.foot_pitch_gyro_p_gain;
		balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_ = desired_balance_param_.foot_pitch_gyro_d_gain;

		previous_balance_param_ = desired_balance_param_;
		balance_update_ = false;
	}
	else
	{
		coeff = balance_param_update_coeff_.getPosition(balance_updating_sys_time_sec_);

		balance_ctrl_.setCOBManualAdjustment((desired_balance_param_.cob_x_offset_m - previous_balance_param_.cob_x_offset_m)*coeff + previous_balance_param_.cob_x_offset_m,
				(desired_balance_param_.cob_y_offset_m - previous_balance_param_.cob_y_offset_m)*coeff + previous_balance_param_.cob_y_offset_m, 0);

		balance_ctrl_.foot_roll_gyro_ctrl_.p_gain_ = (desired_balance_param_.foot_roll_gyro_p_gain - previous_balance_param_.foot_roll_gyro_p_gain)*coeff + previous_balance_param_.foot_roll_gyro_p_gain;
		balance_ctrl_.foot_roll_gyro_ctrl_.d_gain_ = (desired_balance_param_.foot_roll_gyro_d_gain - previous_balance_param_.foot_roll_gyro_d_gain)*coeff + previous_balance_param_.foot_roll_gyro_d_gain;

		balance_ctrl_.foot_pitch_gyro_ctrl_.p_gain_ = (desired_balance_param_.foot_pitch_gyro_p_gain - previous_balance_param_.foot_pitch_gyro_p_gain)*coeff + previous_balance_param_.foot_pitch_gyro_p_gain;
		balance_ctrl_.foot_pitch_gyro_ctrl_.d_gain_ = (desired_balance_param_.foot_pitch_gyro_d_gain - previous_balance_param_.foot_pitch_gyro_d_gain)*coeff + previous_balance_param_.foot_pitch_gyro_d_gain;
	}

}
void MotionModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
		std::map<std::string, double> sensors)
{

	if (enable_ == false)
	{
		return;
	}

	updateBalanceParameter();

	//// read current position ////
	if(new_count_ == 1)
	{
		new_count_ ++;
		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
			robotis_framework::Dynamixel* dxl_info = state_iter->second;
			if(dxl_info->id_ > 10 && dxl_info->id_ < 23)
			{
				if(gazebo_check == true)
					result_[joint_name]->goal_position_ = result_[joint_name]->present_position_; // 가제보 상 초기위치 0
			}
		} //
	}
	if(is_moving_l_ == false && is_moving_r_ == false) // desired pose
	{
		ROS_INFO("Motion Stay");
	}
	else
	{
		ROS_INFO("Motion Trajectory Start");
		// trajectory is working cartesian space control
		result_end_l_ = end_to_rad_l_->cal_end_point_to_rad(leg_end_point_l_);
		result_end_r_ = end_to_rad_r_->cal_end_point_to_rad(leg_end_point_r_);
		//<---  read   --->
		for(int id=11 ; id<23 ; id++)
		{
			if(gazebo_check == true)
				result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
			//else
			// result_[joint_id_to_name_[id]]->present_position_ = dxls[joint_id_to_name_[id]]->dxl_state_->present_position_; // real robot
		}
		is_moving_l_ = end_to_rad_l_-> is_moving_check;
		is_moving_r_ = end_to_rad_r_-> is_moving_check;
	}
	///////////////////////////////////////////////////// control //////////////////////////////////////////////////////////
	//////balance
	result_mat_l_ = robotis_framework::getTransformationXYZRPY(result_end_l_.coeff(0,0), result_end_l_.coeff(1,0), result_end_l_.coeff(2,0),
			result_end_l_.coeff(5,0), result_end_l_.coeff(4,0), result_end_l_.coeff(3,0));
	result_mat_r_ = robotis_framework::getTransformationXYZRPY(result_end_r_.coeff(0,0), result_end_r_.coeff(1,0), result_end_r_.coeff(2,0),
			result_end_r_.coeff(5,0), result_end_r_.coeff(4,0), result_end_r_.coeff(3,0));

	//future work : cob must be calculated.
	result_mat_cob_modified_ = result_mat_cob_;
	result_mat_l_modified_ = result_mat_l_;
	result_mat_r_modified_ = result_mat_r_;

	balance_ctrl_.setDesiredPose(result_mat_cob_, result_mat_r_, result_mat_l_);
	balance_ctrl_.process(0, &result_mat_cob_modified_, &result_mat_r_modified_, &result_mat_l_modified_);

	result_pose_l_modified_ = robotis_framework::getPose3DfromTransformMatrix(result_mat_l_modified_);
	result_pose_r_modified_ = robotis_framework::getPose3DfromTransformMatrix(result_mat_r_modified_);

	//IK
	l_kinematics_->InverseKinematics(result_pose_l_modified_.x, result_pose_l_modified_.y - 0.105, result_pose_l_modified_.z,
			result_pose_l_modified_.yaw, result_pose_l_modified_.pitch, result_pose_l_modified_.roll); // pX pY pZ alpha betta kamma
	r_kinematics_->InverseKinematics(result_pose_r_modified_.x, result_pose_r_modified_.y + 0.105, result_pose_r_modified_.z,
			result_pose_r_modified_.yaw, result_pose_r_modified_.pitch, result_pose_r_modified_.roll); // pX pY pZ alpha betta kamma

	//<---  test control --->

	result_[joint_id_to_name_[20]]->goal_position_ = r_kinematics_->joint_radian(5,0);
	result_[joint_id_to_name_[19]]->goal_position_ = -l_kinematics_->joint_radian(5,0);
	//<---  cartesian space control  --->
/*	result_[joint_id_to_name_[11]]->goal_position_ = -l_kinematics_->joint_radian(1,0);
	result_[joint_id_to_name_[13]]->goal_position_ =  l_kinematics_->joint_radian(2,0);
	result_[joint_id_to_name_[15]]->goal_position_ =  l_kinematics_->joint_radian(3,0);

	result_[joint_id_to_name_[17]]->goal_position_ = -l_kinematics_->joint_radian(4,0);
	result_[joint_id_to_name_[19]]->goal_position_ = -l_kinematics_->joint_radian(5,0);
	result_[joint_id_to_name_[21]]->goal_position_ =  l_kinematics_->joint_radian(6,0);

	result_[joint_id_to_name_[12]]->goal_position_ =  r_kinematics_->joint_radian(1,0);
	result_[joint_id_to_name_[14]]->goal_position_ =  r_kinematics_->joint_radian(2,0);
	result_[joint_id_to_name_[16]]->goal_position_ =  r_kinematics_->joint_radian(3,0);

	result_[joint_id_to_name_[18]]->goal_position_ =  r_kinematics_->joint_radian(4,0);
	result_[joint_id_to_name_[20]]->goal_position_ =  r_kinematics_->joint_radian(5,0);
	result_[joint_id_to_name_[22]]->goal_position_ =  r_kinematics_->joint_radian(6,0);

	result_[joint_id_to_name_[20]]->goal_position_ = r_kinematics_->joint_radian(5,0);
	result_[joint_id_to_name_[19]]->goal_position_ = -l_kinematics_->joint_radian(5,0);*/

	//	cop_cal->jointStateGetForTransForm(l_kinematics_->joint_radian, r_kinematics_->joint_radian);
	//	cop_cal->copCalculationResult();
	/*
	// cop
	cop_cal->jointStateGetForTransForm(l_kinematics_->joint_radian, r_kinematics_->joint_radian);
	cop_cal->copCalculationResult();

	//<---  test control --->

	 	result_[joint_id_to_name_[20]]->goal_position_ = r_kinematics_->joint_radian(5,0);
		result_[joint_id_to_name_[19]]->goal_position_ = -l_kinematics_->joint_radian(5,0);


	//<---  cartesian space control  --->
	result_[joint_id_to_name_[11]]->goal_position_ = -l_kinematics_->joint_radian(1,0);
	result_[joint_id_to_name_[13]]->goal_position_ =  l_kinematics_->joint_radian(2,0);
	result_[joint_id_to_name_[15]]->goal_position_ =  l_kinematics_->joint_radian(3,0);

	result_[joint_id_to_name_[17]]->goal_position_ = -l_kinematics_->joint_radian(4,0);
	result_[joint_id_to_name_[19]]->goal_position_ = -l_kinematics_->joint_radian(5,0);
	result_[joint_id_to_name_[21]]->goal_position_ =  l_kinematics_->joint_radian(6,0);

	result_[joint_id_to_name_[12]]->goal_position_ =  r_kinematics_->joint_radian(1,0);
	result_[joint_id_to_name_[14]]->goal_position_ =  r_kinematics_->joint_radian(2,0);
	result_[joint_id_to_name_[16]]->goal_position_ =  r_kinematics_->joint_radian(3,0);

	result_[joint_id_to_name_[18]]->goal_position_ =  r_kinematics_->joint_radian(4,0);
	result_[joint_id_to_name_[20]]->goal_position_ =  r_kinematics_->joint_radian(5,0);
	result_[joint_id_to_name_[22]]->goal_position_ =  r_kinematics_->joint_radian(6,0);

		result_[joint_id_to_name_[20]]->goal_position_ = r_kinematics_->joint_radian(5,0);
		result_[joint_id_to_name_[19]]->goal_position_ = -l_kinematics_->joint_radian(5,0);

	// real test //
	/*for(int id = 11; id<23 ; id++)
	{
		current_leg_pose_msg_.data.push_back(dxls[joint_id_to_name_[id]]->dxl_state_->present_position_);
	}
	current_leg_pose_pub.publish(current_leg_pose_msg_);
	current_leg_pose_msg_.data.clear();*/

	// gazebo
	for(int id = 1; id<7 ; id++)
	{
		current_leg_pose_msg_.data.push_back(l_kinematics_->joint_radian(id,0));
	}
	for(int id = 1; id<7 ; id++)
	{
		current_leg_pose_msg_.data.push_back(r_kinematics_->joint_radian(id,0));
	}
	current_leg_pose_pub.publish(current_leg_pose_msg_);
	current_leg_pose_msg_.data.clear();
	// l_ endpoint xyz
	/*	state_end_point_pose_msg_.x=  result_pose_l_modified_.x;
	state_end_point_pose_msg_.y=  result_pose_l_modified_.y;
	state_end_point_pose_msg_.z=  result_pose_l_modified_.z;
	state_end_point_pose_pub.publish(state_end_point_pose_msg_);
	// l_ endpoint radian alpha betta kamma
	state_end_point_orientation_msg_.x=  result_pose_l_modified_.roll;
	state_end_point_orientation_msg_.y=  result_pose_l_modified_.pitch;
	state_end_point_orientation_msg_.z=  result_pose_l_modified_.yaw;
	state_end_point_orientation_pub.publish(state_end_point_orientation_msg_);*/

	/*	// r_ endpoint xyz
	state_end_point_pose_msg_.x=  result_pose_r_modified_.x;
	state_end_point_pose_msg_.y=  result_pose_r_modified_.y;
	state_end_point_pose_msg_.z=  result_pose_r_modified_.z;
	state_end_point_pose_pub.publish(state_end_point_pose_msg_);
	// r_ endpoint radian alpha betta kamma
	state_end_point_orientation_msg_.x=  result_pose_r_modified_.roll;
	state_end_point_orientation_msg_.y=  result_pose_r_modified_.pitch;
	state_end_point_orientation_msg_.z=  result_pose_r_modified_.yaw;
	state_end_point_pose_pub.publish(state_end_point_orientation_msg_);*/
}
void MotionModule::stop()
{
	return;
}



