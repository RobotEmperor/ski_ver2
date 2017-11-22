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
	pose_ = 0;
	current_time_ = 0;
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
	l_kinematics_ = new heroehs_math::Kinematics;
	r_kinematics_ = new heroehs_math::Kinematics;
	end_to_rad_l_ = new heroehs_math::CalRad;
	end_to_rad_r_ = new heroehs_math::CalRad;
	one_joint_ = new heroehs_math::CalRad;


	//////////////////////////
	currentGyroX = 0;
	currentGyroY = 0;
	currentGyroZ = 0;
	//////////////////////////
	pre_motion_command_ = 0;
	motion_command_ = 0;
	motion_seq_ = 1;
	//////////////////////////
	result_rad_one_joint_= 0;
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

	center_change_ = new CenterChange;

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
	//

	balance_ctrl_.initialize(control_cycle_msec);
	previous_balance_param_.cob_x_offset_m = 0;
	previous_balance_param_.cob_y_offset_m = 0;
	previous_balance_param_.foot_roll_gyro_p_gain = 0;
	previous_balance_param_.foot_roll_gyro_d_gain = 0;
	previous_balance_param_.foot_pitch_gyro_p_gain = 0;
	previous_balance_param_.foot_pitch_gyro_d_gain = 0;

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
	zmp_point_pub = ros_node.advertise<geometry_msgs::PointStamped>("/zmp_point",100);

	/* subscribe topics */
	get_imu_data_sub_ = ros_node.subscribe("/imu/data", 100, &MotionModule::imuDataMsgCallback, this);
	get_ft_data_sub_ = ros_node.subscribe("/diana/force_torque_data", 100, &MotionModule::ftDataMsgCallback, this);

	// for gui
	set_balance_param_sub_ = ros_node.subscribe("/diana/balance_parameter", 5, &MotionModule::setBalanceParameterCallback, this);
	ros::Subscriber motion_num_msg_sub = ros_node.subscribe("/motion_num", 5, &MotionModule::desiredMotionMsgCallback, this);
	ros::Subscriber center_change_msg_sub = ros_node.subscribe("/diana/center_change", 5, &MotionModule::desiredCenterChangeMsgCallback, this);

	ros::WallDuration duration(control_cycle_msec_ / 1000.0);
	while(ros_node.ok())
		callback_queue.callAvailable(duration);
}
bool MotionModule::isRunning()
{
	return running_;
}
void MotionModule::desiredMotionMsgCallback(const std_msgs::Int32::ConstPtr& msg) // GUI 에서 motion_num topic을 sub 받아 실행 모션 번호 디텍트
{
	is_moving_l_ = true;
	is_moving_r_ = true;
	is_moving_one_joint_ = true;

	motion_command_ = msg->data;
	if(msg->data != 0)
	{
		parse_motion_data_(msg->data);
		motion_seq_= 1;
	}
	pre_motion_command_ = motion_command_;
	current_time_ = 0;
}
void MotionModule::desiredCenterChangeMsgCallback(const diana_msgs::CenterChange::ConstPtr& msg) // GUI 에서 motion_num topic을 sub 받아 실행 모션 번호 디텍트
{
	is_moving_l_ = true;
	is_moving_r_ = true;
	is_moving_one_joint_ = true;

	center_change_->calculateStepEndPointValue(msg->center_change,100); // 0.01 단위로 조정 가능.
	center_change_->parseMotionData("pflug_bogen");

	for(int m = 0 ; m<6 ; m++)
	{
		leg_end_point_l_(m,1) = center_change_->step_end_point_value[1][m];
		leg_end_point_r_(m,1) = center_change_->step_end_point_value[2][m];
		leg_end_point_l_(m,7) = msg->time_change;
		leg_end_point_r_(m,7) = msg->time_change;
	}
}
void MotionModule::imuDataMsgCallback(const sensor_msgs::Imu::ConstPtr& msg) // gyro data get
{
	currentGyroX = (double) msg->angular_velocity.x;
	currentGyroY = (double) -msg->angular_velocity.y;
	currentGyroZ = (double) msg->angular_velocity.z;

	balance_ctrl_.setCurrentGyroSensorOutput(currentGyroY, currentGyroX);
}
void MotionModule::ftDataMsgCallback(const diana_msgs::ForceTorque::ConstPtr& msg)// force torque sensor data get
{
	currentFX_l = (double) msg->fx_raw_l;
	currentFY_l = (double) msg->fy_raw_l;
	currentFZ_l = (double) msg->fz_raw_l;

	currentTX_l = (double) msg->torque_x_raw_l;
	currentTY_l = (double) msg->torque_y_raw_l;
	currentTZ_l = (double) msg->torque_z_raw_l;

	currentFX_r = (double) msg->fx_raw_r;
	currentFY_r = (double) msg->fy_raw_r;
	currentFZ_r = (double) msg->fz_raw_r;

	currentTX_r = (double) msg->torque_x_raw_r;
	currentTY_r = (double) msg->torque_y_raw_r;
	currentTZ_r = (double) msg->torque_z_raw_r;

	zmp_cal.ftSensorDataLeftGet(currentFX_l, currentFY_l, currentFZ_l, currentTX_l, currentTY_l, currentTZ_l);
	zmp_cal.ftSensorDataRightGet(currentFX_r, currentFY_r, currentFZ_r, currentTX_r, currentTY_r, currentTZ_r);

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
void MotionModule::parse_motion_data_(int motion_num_)
{
	int size_num_ = 0;
	int node_size_ = 0;
	ostringstream motion_num_str_temp;
	motion_num_str_temp << motion_num_;
	string motion_num_str = motion_num_str_temp.str();
	std::string path_ = ros::package::getPath("motion_module") + "/data/motion_"+ motion_num_str + ".yaml";// 로스 패키지에서 YAML파일의 경로를 읽어온다.
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
	// time load //
	size_num_ = doc["motion_time"].size();
	change_desired_time_.resize(size_num_,1);
	change_desired_time_.fill(0);
	for(int i=0; i<size_num_; i++)
	{
		change_desired_time_(i,0) = doc["motion_time"][i].as<double>();
	}
	// motion data load initialize//
	YAML::Node pose_node = doc["motion"];// YAML 에 string "motion"을 읽어온다.
	size_num_= doc["motion"].size();
	YAML::iterator it = pose_node.begin();
	node_size_ = it->second.size();
	change_desired_pose_.resize(size_num_,node_size_);
	change_desired_pose_.fill(0);
	pose_ = size_num_;

	// motion data load //
	for (YAML::iterator it = pose_node.begin(); it != pose_node.end(); ++it) //motion_node 벡터의 처음과 시작 끝까지 for 문으로 반복한다.
	{
		int pose_;
		double value_;
		// 한 줄에서 int 와 double 을 분리한다.
		pose_ = it->first.as<int>();

		for(int i=0; i<node_size_; i++)
		{
			change_desired_pose_(pose_-1,i) = it->second[i].as<double>();
		}
	}// YAML에 로드된 초기 포즈 값을 라디안으로 바꾸고, eigen matrix 에 id 개수만큼 열을 생성한다.

	//motion velocity calculation
	change_desired_final_vel_.resize(size_num_-1,node_size_);
	change_desired_final_vel_.fill(0);
	change_desired_initial_vel_.resize(size_num_-1,node_size_);
	change_desired_initial_vel_.fill(0);
	motion_vel_cal_leg_(size_num_, node_size_); //(노드의 개수 = 포즈의 개수, 노드의 배열의 요소 개수)

	for(int pose = 0; pose < size_num_ ; pose++)
	{
		for(int j=3;j<6;j++)
		{
			change_desired_pose_(pose,j) =	change_desired_pose_(pose,j)*DEGREE2RADIAN;
		}
		for(int j=9;j<12;j++)
		{
			change_desired_pose_(pose,j) =	change_desired_pose_(pose,j)*DEGREE2RADIAN;
		}
		change_desired_pose_(pose,12) = change_desired_pose_(pose,12)*DEGREE2RADIAN; //waist
	}

	//속도 계산 완료
}
void MotionModule::motion_vel_cal_leg_( int pose_num_, int node_num_)
{
	Eigen::MatrixXd motion_velocity_;
	motion_velocity_.resize(pose_num_-1,node_num_);
	motion_velocity_.fill(0);
	for(int i=0; i<pose_num_-1;i++) //
	{
		for(int j=0;j<3;j++)
		{
			motion_velocity_(i,j) = (change_desired_pose_(i+1,j) - change_desired_pose_(i,j))/change_desired_time_(i,0);
		}
		for(int j=3;j<6;j++)
		{
			motion_velocity_(i,j) = (change_desired_pose_(i+1,j)*DEGREE2RADIAN - change_desired_pose_(i,j)*DEGREE2RADIAN )/change_desired_time_(i,0);
		}
		for(int j=6;j<9;j++)
		{
			motion_velocity_(i,j) = (change_desired_pose_(i+1,j) - change_desired_pose_(i,j))/change_desired_time_(i,0);
		}
		for(int j=9;j<12;j++)
		{
			motion_velocity_(i,j) = (change_desired_pose_(i+1,j)*DEGREE2RADIAN - change_desired_pose_(i,j)*DEGREE2RADIAN)/change_desired_time_(i,0);
		}
		for(int j=0; j<12; j++)
		{
			if(i < pose_num_-2)
			{
				change_desired_initial_vel_(i+1,j) = change_desired_final_vel_(i,j);

				if(change_desired_pose_(i,j) == change_desired_pose_(i+1, j))
					change_desired_final_vel_(i,j) = 0;
				else
				{
					if(motion_velocity_(i,j)*motion_velocity_(i+1,j) > 0)
					{
						change_desired_final_vel_(i,j) = motion_velocity_(i+1,j);
					}
					else
						change_desired_final_vel_(i,j) = 0;
				}
			}
			else
				change_desired_final_vel_(i,j) = 0;
		}
	}//velocity_complete
}
void MotionModule::motion_generater_()
{
	for(int pose=1; pose<pose_; pose++)
	{
		if(motion_seq_ >= pose_)
			motion_seq_ = 0;

		if(motion_seq_ == pose)
		{
			ROS_INFO("real_ pose %d", motion_seq_);
			if(change_desired_time_(pose-1,0) > current_time_)
			{

				is_moving_l_ = true;
				is_moving_r_ = true;
				is_moving_one_joint_ = true;
				for(int m = 0 ; m<6 ; m++)
				{
					leg_end_point_l_(m,1) = change_desired_pose_(pose, m);
					leg_end_point_r_(m,1) = change_desired_pose_(pose, m+6);
					leg_end_point_l_(m,7) = change_desired_time_(pose-1, 0);
					leg_end_point_r_(m,7) = change_desired_time_(pose-1, 0);
				}
				for(int m = 0 ; m<6 ; m++)
				{
					leg_end_point_l_(m,2) = change_desired_initial_vel_(pose-1, m);
					leg_end_point_r_(m,2) = change_desired_initial_vel_(pose-1, m+6);
					leg_end_point_l_(m,3) = change_desired_final_vel_(pose-1, m);
					leg_end_point_r_(m,3) = change_desired_final_vel_(pose-1, m+6);
				}
				one_joint_ctrl_(0,1) = change_desired_pose_(pose,12);
				one_joint_ctrl_(0,7) = change_desired_time_(pose-1,0);
			}

			else if(current_time_ > change_desired_time_(pose-1,0))
			{
				motion_seq_++;
				current_time_ = 0;
				ROS_INFO("pose :: change!!, %d", motion_seq_);
			}
			else
				return;
		}
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

	current_time_ = current_time_+ 0.008;


	//motion_generater_();
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
			else
			{
				result_[joint_name]->goal_position_ = dxls[joint_name]->dxl_state_->present_position_; // 다이나믹셀에서 읽어옴
			}
		} // 등록된 다이나믹셀의 위치값을 읽어와서 goal position 으로 입력

	}
	if(is_moving_l_ == false && is_moving_r_ == false && is_moving_one_joint_ == false) // desired pose
	{
		//ROS_INFO("Motion Stay");
	}
	else
	{
		//ROS_INFO("Motion Trajectory Start");
		// trajectory is working cartesian space control
		result_end_l_ = end_to_rad_l_->cal_end_point_to_rad(leg_end_point_l_);
		result_end_r_ = end_to_rad_r_->cal_end_point_to_rad(leg_end_point_r_);
		result_rad_one_joint_ = one_joint_ -> cal_one_joint_rad(one_joint_ctrl_);
		//<---  read   --->
		for(int id=10 ; id<23 ; id++)
		{
			if(gazebo_check == true)
				result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
			//else
			//result_[joint_id_to_name_[id]]->present_position_ = dxls[joint_id_to_name_[id]]->dxl_state_->present_position_; // real robot
		}

		is_moving_l_ = end_to_rad_l_-> is_moving_check;
		is_moving_r_ = end_to_rad_r_-> is_moving_check;
		is_moving_one_joint_ = one_joint_ ->is_moving_check;

	}
	///////////////////////////////////////////////////// control //////////////////////////////////////////////////////////
	//////balance
	result_mat_l_ = robotis_framework::getTransformationXYZRPY(result_end_l_.coeff(0,0), result_end_l_.coeff(1,0), result_end_l_.coeff(2,0),
			result_end_l_.coeff(3,0), result_end_l_.coeff(4,0), result_end_l_.coeff(5,0));
	result_mat_r_ = robotis_framework::getTransformationXYZRPY(result_end_r_.coeff(0,0), result_end_r_.coeff(1,0), result_end_r_.coeff(2,0),
			result_end_r_.coeff(3,0), result_end_r_.coeff(4,0), result_end_r_.coeff(5,0));

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
			result_pose_l_modified_.roll, result_pose_l_modified_.pitch, result_pose_l_modified_.yaw); // pX pY pZ alpha betta kamma
	r_kinematics_->InverseKinematics(result_pose_r_modified_.x, result_pose_r_modified_.y + 0.105, result_pose_r_modified_.z,
			result_pose_r_modified_.roll, result_pose_r_modified_.pitch, result_pose_r_modified_.yaw); // pX pY pZ alpha betta kamma

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// zmp
	zmp_cal.jointStateGetForTransForm(l_kinematics_->joint_radian, r_kinematics_->joint_radian);
	zmp_cal.zmpCalculationResult();

	// display zmp to rviz
	zmp_point_msg_.header.stamp = ros::Time();
	std::string frame = "/pelvis";
	zmp_point_msg_.header.frame_id = frame.c_str();

	zmp_point_msg_.point.x = zmp_cal.cf_zmp_point(0,0);
	zmp_point_msg_.point.y = zmp_cal.cf_zmp_point(1,0);
	zmp_point_msg_.point.z = zmp_cal.cf_zmp_point(2,0);
	zmp_point_pub.publish(zmp_point_msg_);


	//<---  joint space control --->
	result_[joint_id_to_name_[1]]->goal_position_ = 0;// head
	result_[joint_id_to_name_[10]]->goal_position_ = -result_rad_one_joint_; // waist roll

	//<---  cartesian space control  --->
	result_[joint_id_to_name_[11]]->goal_position_ = -l_kinematics_->joint_radian(1,0);//
	result_[joint_id_to_name_[13]]->goal_position_ = l_kinematics_->joint_radian(2,0);
	result_[joint_id_to_name_[15]]->goal_position_ = l_kinematics_->joint_radian(3,0);

	result_[joint_id_to_name_[17]]->goal_position_ = -l_kinematics_->joint_radian(4,0);//
	result_[joint_id_to_name_[19]]->goal_position_ = -l_kinematics_->joint_radian(5,0);//
	result_[joint_id_to_name_[21]]->goal_position_ = l_kinematics_->joint_radian(6,0);

	result_[joint_id_to_name_[12]]->goal_position_ = r_kinematics_->joint_radian(1,0);
	result_[joint_id_to_name_[14]]->goal_position_ = r_kinematics_->joint_radian(2,0);
	result_[joint_id_to_name_[16]]->goal_position_ = r_kinematics_->joint_radian(3,0);

	result_[joint_id_to_name_[18]]->goal_position_ = r_kinematics_->joint_radian(4,0);
	result_[joint_id_to_name_[20]]->goal_position_ = r_kinematics_->joint_radian(5,0);
	result_[joint_id_to_name_[22]]->goal_position_ = r_kinematics_->joint_radian(6,0);

	// l_ endpoint xyz
	state_end_point_pose_msg_.x=  result_pose_l_modified_.x;
	state_end_point_pose_msg_.y=  result_pose_l_modified_.y;
	state_end_point_pose_msg_.z=  result_pose_l_modified_.z;
	state_end_point_pose_pub.publish(state_end_point_pose_msg_);
	// l_ endpoint radian alpha betta kamma
	state_end_point_orientation_msg_.x=  result_pose_l_modified_.roll;
	state_end_point_orientation_msg_.y=  result_pose_l_modified_.pitch;
	state_end_point_orientation_msg_.z=  result_pose_l_modified_.yaw;
	state_end_point_orientation_pub.publish(state_end_point_orientation_msg_);

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



