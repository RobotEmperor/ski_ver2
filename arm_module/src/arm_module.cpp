/*
 * arm_module.cpp
 *
 *  Created on: Dec 13, 2017
 *      Author: robotemperor
 */
#include "arm_module/arm_module.h"
using namespace arm_module;
void ArmModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
	is_moving_l_arm_ = false;
	is_moving_r_arm_ = false;

	control_cycle_msec_ = control_cycle_msec;
	queue_thread_ = boost::thread(boost::bind(&ArmModule::queueThread, this));

	for (std::map<std::string, robotis_framework::Dynamixel*>::iterator it = robot->dxls_.begin();
			it != robot->dxls_.end(); it++)
	{
		std::string joint_name = it->first;
		robotis_framework::Dynamixel* dxl_info = it->second;

		joint_name_to_id_[joint_name] = dxl_info->id_;
		joint_id_to_name_[dxl_info->id_] = joint_name;
	}
	// arm initialize value in local frame
	// left //
	l_arm_end_point_.resize(6,8);
	l_arm_end_point_.fill(0);
	l_arm_end_point_(1,0) = 0.07; // y 초기값
	l_arm_end_point_(1,1) = 0.07 ; //
	l_arm_end_point_(2,0) = -0.4;
	l_arm_end_point_(2,1) = -0.4;
	end_to_rad_l_arm_->cal_end_point_tra_py->current_pose = 0.07;
	end_to_rad_l_arm_->current_pose_change(1,0) = 0.07;
	end_to_rad_l_arm_->cal_end_point_tra_pz->current_pose = -0.4;
	end_to_rad_l_arm_->current_pose_change(2,0) = -0.4;
	result_end_l_arm_.resize(6,1);
	result_end_l_arm_.fill(0);

	//right //
	r_arm_end_point_.resize(6,8);
	r_arm_end_point_.fill(0);
	result_end_r_arm_.resize(6,1);
	result_end_r_arm_.fill(0);
	r_arm_end_point_(1,0) = -0.07; // y 초기값
	r_arm_end_point_(1,1) = -0.07; //
	r_arm_end_point_(2,0) = -0.4;
	r_arm_end_point_(2,1) = -0.4;
	end_to_rad_r_arm_->cal_end_point_tra_py->current_pose = - 0.07;
	end_to_rad_r_arm_->current_pose_change(1,0) = -0.07;
	end_to_rad_r_arm_->cal_end_point_tra_pz->current_pose = -0.4;
	end_to_rad_r_arm_->current_pose_change(2,0) = -0.4;

	for(int joint_num_= 0; joint_num_< 6 ; joint_num_ ++)
	{
		l_arm_end_point_ (joint_num_, 7) = traj_time_test;
		r_arm_end_point_ (joint_num_, 7) = traj_time_test;
	}
	ROS_INFO("< -------  Initialize Module : Arm Module  !!  ------->");
}
bool ArmModule::isRunning()
{
	return running_;
}
void ArmModule::updateBalanceGyroParameter()
{
	Eigen::MatrixXd value;
	value.resize(1,8);
	value.fill(0);
	value(0,7) = updating_duration;
	value(0,1) = gyro_roll_p_gain;
	gyro_roll_function->kp_ = gain_roll_p_adjustment -> fifth_order_traj_gen_one_value(value);
	value(0,1) = gyro_roll_d_gain;
	gyro_roll_function->kd_ = gain_roll_d_adjustment -> fifth_order_traj_gen_one_value(value);

	value(0,1) = gyro_pitch_p_gain;
	gyro_pitch_function->kp_ = gain_pitch_p_adjustment -> fifth_order_traj_gen_one_value(value);
	value(0,1) = gyro_pitch_d_gain;
	gyro_pitch_function->kd_ = gain_pitch_d_adjustment -> fifth_order_traj_gen_one_value(value);

	value(0,1) = gyro_yaw_p_gain;
	gyro_yaw_function->kp_ = gain_yaw_p_adjustment -> fifth_order_traj_gen_one_value(value);
	value(0,1) = gyro_yaw_d_gain;
	gyro_yaw_function->kd_ = gain_yaw_d_adjustment -> fifth_order_traj_gen_one_value(value);
}
double ArmModule::limitCheckArmAngle(double calculated_value, double max, double min)
{
	if(calculated_value > (max*DEGREE2RADIAN))
		return (max*DEGREE2RADIAN);
	else if (calculated_value < (min*DEGREE2RADIAN))
		return (min*DEGREE2RADIAN);
	else
		return calculated_value;
}
double ArmModule::limitCheckArmPosition(double calculated_value, double max, double min)
{
	if(calculated_value > max)
		return max;
	else if (calculated_value < min)
		return min;
	else
		return calculated_value;
}
void ArmModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
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
		is_moving_l_arm_ = false;
		is_moving_r_arm_ = false;
		new_count_ ++;
		for (std::map<std::string, robotis_framework::Dynamixel*>::iterator state_iter = dxls.begin();
				state_iter != dxls.end(); state_iter++)
		{
			std::string joint_name = state_iter->first;
			if(!joint_name.compare("l_shoulder_pitch")|| !joint_name.compare("l_shoulder_roll") || !joint_name.compare("l_elbow_pitch") ||
					!joint_name.compare("r_shoulder_pitch") || !joint_name.compare("r_shoulder_roll") || !joint_name.compare("r_elbow_pitch"))
			{
				if(gazebo_check == true)
					result_[joint_name]->goal_position_ = result_[joint_name]->present_position_; // 가제보 상 초기위치 0
			}
		} // 등록된 다이나믹셀의 위치값을 읽어와서 goal position 으로 입력
		ROS_INFO("Arm Start");
		result_end_l_arm_ = end_to_rad_l_arm_ -> cal_end_point_to_rad(l_arm_end_point_);
		result_end_r_arm_ = end_to_rad_r_arm_ -> cal_end_point_to_rad(r_arm_end_point_);
	}
	if(is_moving_l_arm_ == false && is_moving_r_arm_ == false)
	{
		ROS_INFO("Arm Stay");
	}
	else
	{
		ROS_INFO("Arm Module run !!!!");
		l_arm_end_point_(1,1)=limitCheckArmPosition(l_arm_end_point_(1,1),  0.35,  0.07);
		r_arm_end_point_(1,1)=limitCheckArmPosition(r_arm_end_point_(1,1), -0.07, -0.35);

		result_end_l_arm_ = end_to_rad_l_arm_ -> cal_end_point_to_rad(l_arm_end_point_);
		result_end_r_arm_ = end_to_rad_r_arm_ -> cal_end_point_to_rad(r_arm_end_point_);

		is_moving_l_arm_  = end_to_rad_l_arm_  -> is_moving_check;
		is_moving_r_arm_  = end_to_rad_r_arm_  -> is_moving_check;
	}
	///////////////////////////////////////////////////// control //////////////////////////////////////////////////////////
	l_arm_kinematics_ -> InverseKinematicsArm(result_end_l_arm_(0,0), result_end_l_arm_(1,0), result_end_l_arm_(2,0));
	r_arm_kinematics_ -> InverseKinematicsArm(result_end_r_arm_(0,0), result_end_r_arm_(1,0), result_end_r_arm_(2,0));

	gyro_yaw_function  ->PID_calculate(0,tf_current_gyro_z);
	gyro_roll_function ->PID_calculate(0,tf_current_gyro_x);
	gyro_pitch_function->PID_calculate(0,tf_current_gyro_y);

	result_[joint_id_to_name_[1]]->goal_position_ =  -l_arm_kinematics_->joint_radian(1,0) + gyro_pitch_function->PID_calculate(0,tf_current_gyro_y) - gyro_yaw_function->PID_calculate(0,tf_current_gyro_z);// + gyro_yaw_function->PID_calculate(0,tf_current_gyro_z);
	result_[joint_id_to_name_[3]]->goal_position_ =  -limitCheckArmAngle(l_arm_kinematics_->joint_radian(2,0) + gyro_roll_function->PID_calculate(0,tf_current_gyro_x) + fabs(gyro_yaw_function->PID_calculate(0,tf_current_gyro_z)), 90, 5); // - gyro_yaw_function->PID_calculate(0,tf_current_gyro_z);
	result_[joint_id_to_name_[5]]->goal_position_ =  l_arm_kinematics_->joint_radian(3,0) - gyro_pitch_function->PID_calculate(0,tf_current_gyro_y) + gyro_yaw_function->PID_calculate(0,tf_current_gyro_z);


	result_[joint_id_to_name_[2]]->goal_position_ =  r_arm_kinematics_->joint_radian(1,0) - gyro_pitch_function->PID_calculate(0,tf_current_gyro_y) - gyro_yaw_function->PID_calculate(0,tf_current_gyro_z);
	result_[joint_id_to_name_[4]]->goal_position_ =  -limitCheckArmAngle(r_arm_kinematics_->joint_radian(2,0) + gyro_roll_function->PID_calculate(0,tf_current_gyro_x) - fabs(gyro_yaw_function->PID_calculate(0,tf_current_gyro_z)), -5, -90);
	result_[joint_id_to_name_[6]]->goal_position_ = -r_arm_kinematics_->joint_radian(3,0) + gyro_pitch_function->PID_calculate(0,tf_current_gyro_y) + gyro_yaw_function->PID_calculate(0,tf_current_gyro_z);

}
void ArmModule::stop()
{
	return;
}
