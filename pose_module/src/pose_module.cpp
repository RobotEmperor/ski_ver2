/*
 * pose_module.cpp
 *
 *  Created on: Oct 24, 2017
 *      Author: robotemperor
 */
#include <stdio.h>
#include "pose_module/pose_module.h"

using namespace pose_module;

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

	for(int joint_num_= 0; joint_num_< 6 ; joint_num_ ++)
	{
		leg_end_point_l_ (joint_num_, 7)  = traj_time_;
		leg_end_point_r_ (joint_num_, 7)  = traj_time_;
		waist_end_point_ (joint_num_, 7)  = traj_time_;
		head_end_point_  (joint_num_, 7)  = traj_time_;
		l_arm_end_point_ (joint_num_, 7)  = traj_time_;
		r_arm_end_point_ (joint_num_, 7)  = traj_time_;
	}
	ROS_INFO("< -------  Initialize Module : Pose Module !!  ------->");
}

bool PoseModule::isRunning()
{
	return running_;
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
		result_end_l_ = end_to_rad_l_         -> cal_end_point_to_rad(leg_end_point_l_);
		result_end_r_ = end_to_rad_r_         -> cal_end_point_to_rad(leg_end_point_r_);
		result_end_waist_ = end_to_rad_waist_ -> cal_end_point_to_rad(waist_end_point_);
		result_end_head_ = end_to_rad_head_   -> cal_end_point_to_rad(head_end_point_);
		result_end_l_arm_ = end_to_rad_l_arm_ -> cal_end_point_to_rad(l_arm_end_point_);
		result_end_r_arm_ = end_to_rad_r_arm_ -> cal_end_point_to_rad(r_arm_end_point_);

		ROS_INFO("Pose module :: Read position and Send goal position");
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
		//l_arm_kinematics_ -> ArmToOriginTransformation(waist_yaw_rad_ , waist_roll_rad_, l_arm_desired_point_x_, l_arm_desired_point_y_, l_arm_desired_point_z_);
		//r_arm_kinematics_ -> ArmToOriginTransformation(waist_yaw_rad_ , waist_roll_rad_, r_arm_desired_point_x_, r_arm_desired_point_y_, r_arm_desired_point_z_);

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

		is_moving_l_     = end_to_rad_l_      -> is_moving_check;
		is_moving_r_     = end_to_rad_r_      -> is_moving_check;
		is_moving_waist  = end_to_rad_waist_  -> is_moving_check;
		is_moving_head   = end_to_rad_head_   -> is_moving_check;
		is_moving_l_arm  = end_to_rad_l_arm_  -> is_moving_check;
		is_moving_r_arm  = end_to_rad_r_arm_  -> is_moving_check;
	}
	//cartesian space control LEG
	l_kinematics_-> InverseKinematics(result_end_l_(0,0), result_end_l_(1,0) - 0.105, result_end_l_(2,0), result_end_l_(3,0), result_end_l_(4,0), result_end_l_(5,0)); // pX pY pZ alpha betta kamma
	r_kinematics_-> InverseKinematics(result_end_r_(0,0), result_end_r_(1,0) + 0.105, result_end_r_(2,0), result_end_r_(3,0), result_end_r_(4,0), result_end_r_(5,0)); // pX pY pZ alpha betta kamma

	//cartesian space control WAIST
	waist_kinematics_-> XYZEulerAnglesSolution(result_end_waist_ (5,0) , 0, result_end_waist_ (3,0));
	//cartesian space control HEAD
	head_kinematics_ -> ZYXEulerAnglesSolution(result_end_head_(3,0),result_end_head_(4,0),result_end_head_(5,0));
	//cartesian space control ARM
	l_arm_kinematics_ -> InverseKinematicsArm(result_end_l_arm_(0,0), result_end_l_arm_(1,0), result_end_l_arm_(2,0));
	r_arm_kinematics_ -> InverseKinematicsArm(result_end_r_arm_(0,0), result_end_r_arm_(1,0), result_end_r_arm_(2,0));


	waist_yaw_rad_    = dxls["waist_yaw"]->dxl_state_->present_position_;
	waist_roll_rad_   = -dxls["waist_roll"]->dxl_state_->present_position_; // direction must define!;

	//l_arm_kinematics_ -> OriginToArmTransformationPoint(waist_yaw_rad_ , waist_roll_rad_,dxls["l_shoulder_pitch"]->dxl_state_->present_position_, -dxls["l_shoulder_roll"]->dxl_state_->present_position_, dxls["l_elbow_pitch"]->dxl_state_->present_position_);
	//r_arm_kinematics_ ->OriginToArmTransformationPoint(waist_yaw_rad_ , waist_roll_rad_,dxls["r_shoulder_pitch"]->dxl_state_->present_position_, dxls["r_shoulder_roll"]->dxl_state_->present_position_, dxls["r_elbow_pitch"]->dxl_state_->present_position_);
	l_arm_kinematics_ ->OriginToArmTransformationPoint(waist_kinematics_->xyz_euler_angle_z , waist_kinematics_->xyz_euler_angle_x, l_arm_kinematics_->joint_radian(1,0), l_arm_kinematics_->joint_radian(2,0), l_arm_kinematics_->joint_radian(3,0));
	r_arm_kinematics_ ->OriginToArmTransformationPoint(waist_kinematics_->xyz_euler_angle_z , waist_kinematics_->xyz_euler_angle_x, r_arm_kinematics_->joint_radian(1,0), r_arm_kinematics_->joint_radian(2,0), r_arm_kinematics_->joint_radian(3,0));

	cop_cal->jointStateGetForTransForm(l_kinematics_->joint_radian, r_kinematics_->joint_radian);
	cop_cal->copCalculationResult();
	//<---  catesian space control test --->


/*	result_[joint_id_to_name_[9]] -> goal_position_  = waist_kinematics_->xyz_euler_angle_z;// waist yaw
	result_[joint_id_to_name_[10]] -> goal_position_  = waist_kinematics_->xyz_euler_angle_x; // waist roll

	result_[joint_id_to_name_[19]] -> goal_position_  = -l_kinematics_->joint_radian(5,0);
	result_[joint_id_to_name_[20]] -> goal_position_  = r_kinematics_->joint_radian(5,0);

	result_[joint_id_to_name_[23]] -> goal_position_  = head_kinematics_->zyx_euler_angle_z;

	result_[joint_id_to_name_[1]]  -> goal_position_  = l_arm_kinematics_->joint_radian(1,0);
	result_[joint_id_to_name_[3]]  -> goal_position_  = -l_arm_kinematics_->joint_radian(2,0);
	result_[joint_id_to_name_[5]]  -> goal_position_  = l_arm_kinematics_->joint_radian(3,0);*/

	//<---  cartesian space control  --->

	result_[joint_id_to_name_[1]]  -> goal_position_  =  l_arm_kinematics_->joint_radian(1,0);
	result_[joint_id_to_name_[3]]  -> goal_position_  = -l_arm_kinematics_->joint_radian(2,0);
	result_[joint_id_to_name_[5]]  -> goal_position_  =  l_arm_kinematics_->joint_radian(3,0);

	result_[joint_id_to_name_[2]]  -> goal_position_  = -r_arm_kinematics_->joint_radian(1,0);
	result_[joint_id_to_name_[4]]  -> goal_position_  = -r_arm_kinematics_->joint_radian(2,0);
	result_[joint_id_to_name_[6]]  -> goal_position_  = -r_arm_kinematics_->joint_radian(3,0);

	result_[joint_id_to_name_[9]]->goal_position_    =  waist_kinematics_->xyz_euler_angle_z;// waist yaw
	result_[joint_id_to_name_[10]]->goal_position_   = -waist_kinematics_->xyz_euler_angle_x; // waist roll

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

	result_[joint_id_to_name_[23]]->goal_position_   = -head_kinematics_->zyx_euler_angle_z;
	result_[joint_id_to_name_[24]]->goal_position_   = -head_kinematics_->zyx_euler_angle_y;
	result_[joint_id_to_name_[25]]->goal_position_   = -head_kinematics_->zyx_euler_angle_x;

	//printf("%f  ::  %f  :: %f  ::  %f  :: %f  ::  %f  \n",l_kinematics_->joint_radian(1,0), l_kinematics_->joint_radian(2,0),
		//	l_kinematics_->joint_radian(3,0), l_kinematics_->joint_radian(4,0), l_kinematics_->joint_radian(5,0), l_kinematics_->joint_radian(6,0));

	//<---  publisher  --->
	current_arm_state_msg.data.push_back(l_arm_kinematics_ -> origin_to_arm_end_point_tf_(0,3));
	current_arm_state_msg.data.push_back(l_arm_kinematics_ -> origin_to_arm_end_point_tf_(1,3));
	current_arm_state_msg.data.push_back(l_arm_kinematics_ -> origin_to_arm_end_point_tf_(2,3));
	current_arm_state_msg.data.push_back(r_arm_kinematics_ -> origin_to_arm_end_point_tf_(0,3));
	current_arm_state_msg.data.push_back(r_arm_kinematics_ -> origin_to_arm_end_point_tf_(1,3));
	current_arm_state_msg.data.push_back(r_arm_kinematics_ -> origin_to_arm_end_point_tf_(2,3));

	current_arm_state_pub.publish(current_arm_state_msg);
	current_arm_state_msg.data.clear();
	// display cop to rviz
	cop_point_Fz_msg_.header.stamp = ros::Time();
	cop_point_Fy_msg_.header.stamp = ros::Time();
	cop_point_Fx_msg_.header.stamp = ros::Time();

	std::string frame = "/pelvis";
	cop_point_Fz_msg_.header.frame_id = frame.c_str();
	cop_point_Fy_msg_.header.frame_id = frame.c_str();
	cop_point_Fx_msg_.header.frame_id = frame.c_str();

	cop_point_Fz_msg_.point.x = cop_cal -> cop_fz_point_x;
	cop_point_Fz_msg_.point.y = cop_cal -> cop_fz_point_y;
	cop_point_Fz_msg_.point.z = 0;
	cop_point_Fz_pub.publish(cop_point_Fz_msg_);

	cop_point_Fy_msg_.point.x = cop_cal -> cop_fy_point_x;
	cop_point_Fy_msg_.point.y = 0;
	cop_point_Fy_msg_.point.z = cop_cal -> cop_fy_point_z;
	cop_point_Fy_pub.publish(cop_point_Fy_msg_);

	cop_point_Fx_msg_.point.x = 0;
	cop_point_Fx_msg_.point.y = cop_cal -> cop_fx_point_y;
	cop_point_Fx_msg_.point.z = cop_cal -> cop_fx_point_z;
	cop_point_Fx_pub.publish(cop_point_Fx_msg_);

	//<---  read   --->
		/*		for(int id=10 ; id<23 ; id++)
				{
					if(gazebo_check == true)
						result_[joint_id_to_name_[id]]->present_position_ = result_[joint_id_to_name_[id]]->goal_position_; // gazebo
					else
					{
						result_[joint_id_to_name_[id]]->present_position_ = dxls[joint_id_to_name_[id]]->dxl_state_->present_position_; // real robot
					}
				}
*/

}
void PoseModule::stop()
{
	return;
}


