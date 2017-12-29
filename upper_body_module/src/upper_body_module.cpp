/*
 * upper_body_module.cpp
 *
 *  Created on: Dec 04, 2017
 *      Author: robotemperor
 */
#include "upper_body_module/upper_body_module.h"

using namespace upper_body_module;

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
void UpperBodyModule::updateBalanceGyroParameter()
{
	Eigen::MatrixXd value;
	value.resize(1,8);
	value.fill(0);
	value(0,7) = updating_duration;
	value(0,1) = gyro_roll_p_gain;
	gyro_roll_function->kp_ = gain_roll_p_adjustment -> fifth_order_traj_gen_one_value(value);
	value(0,1) = gyro_roll_d_gain;
	gyro_roll_function->kd_ = gain_roll_d_adjustment -> fifth_order_traj_gen_one_value(value);

	value(0,1) = gyro_yaw_p_gain;
	gyro_yaw_function->kp_ = gain_yaw_p_adjustment -> fifth_order_traj_gen_one_value(value);
	value(0,1) = gyro_yaw_d_gain;
	gyro_yaw_function->kd_ = gain_yaw_d_adjustment -> fifth_order_traj_gen_one_value(value);

	value(0,1) = copFz_p_gain;
	cop_compensation_waist->pidControllerFz_x->kp_ = gain_copFz_p_adjustment -> fifth_order_traj_gen_one_value(value);
	cop_compensation_waist->pidControllerFz_y->kp_ = cop_compensation_waist->pidControllerFz_x->kp_;

	value(0,1) = copFz_d_gain;
	cop_compensation_waist->pidControllerFz_x->kd_ = gain_copFz_d_adjustment -> fifth_order_traj_gen_one_value(value);
	cop_compensation_waist->pidControllerFz_y->kd_ = cop_compensation_waist->pidControllerFz_x->kd_;
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
	updateBalanceGyroParameter();
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

	// compensation control Algorithm //
	// gyro roll compensation in joint space
	gyro_roll_function->PID_calculate(0,tf_current_gyro_x); // control value roll rad //
	gyro_yaw_function ->PID_calculate(0,tf_current_gyro_z); // control value yaw  rad //



	// cop
	cop_compensation_waist->reference_point_Fz_x = reference_cop_fz_x;
	cop_compensation_waist->reference_point_Fz_y = reference_cop_fz_y;
	cop_compensation_waist->centerOfPressureCompensationFz(current_cop_fz_x, current_cop_fz_y);


	// real test
/*	result_[joint_id_to_name_[9]] -> goal_position_  = waist_kinematics_ -> xyz_euler_angle_z + gyro_yaw_function ->PID_calculate(0,tf_current_gyro_z); // waist roll
	result_[joint_id_to_name_[10]]-> goal_position_  = - (waist_kinematics_ -> xyz_euler_angle_x + gyro_roll_function->PID_calculate(0,tf_current_gyro_x) + cop_compensation_waist->control_value_Fz_y); // waist roll


	result_[joint_id_to_name_[23]]-> goal_position_  = head_kinematics_ -> zyx_euler_angle_z;*/
	//gazebo

	result_[joint_id_to_name_[9]] -> goal_position_  = waist_kinematics_ -> xyz_euler_angle_z + gyro_yaw_function ->PID_calculate(0,tf_current_gyro_z); // waist roll
	result_[joint_id_to_name_[10]]-> goal_position_  = - (waist_kinematics_ -> xyz_euler_angle_x + gyro_roll_function->PID_calculate(0,tf_current_gyro_x) + cop_compensation_waist->control_value_Fz_y); // waist roll
	result_[joint_id_to_name_[23]]-> goal_position_  = -head_kinematics_ -> zyx_euler_angle_z;
  result_[joint_id_to_name_[24]]-> goal_position_  = -head_kinematics_ -> zyx_euler_angle_y;
	result_[joint_id_to_name_[25]]-> goal_position_  = -head_kinematics_ -> zyx_euler_angle_x;


	//arm module current position transmitted
	temp_waist_yaw_rad   =  dxls["waist_yaw"] -> dxl_state_->present_position_;
	temp_waist_roll_rad  = -dxls["waist_roll"]-> dxl_state_->present_position_; // direction must define!

	//transmit to arm module with waist position
	current_waist_pose_msg.data.push_back(temp_waist_yaw_rad);
	current_waist_pose_msg.data.push_back(temp_waist_roll_rad);
	current_waist_pose_pub.publish(current_waist_pose_msg);
	current_waist_pose_msg.data.clear();

	/*	// display cop to rviz
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
		cop_point_Fx_pub.publish(cop_point_Fx_msg_);*/
}
void UpperBodyModule::stop()
{
	return;
}



