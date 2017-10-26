/*
 * end_to_point_rad_cal.cpp
 *
 *  Created on: Oct 25, 2017
 *      Author: robotemperor
 */

#include "heroehs_math/end_point_to_rad_cal.h"

using namespace heroehs_math;

CalRad::CalRad()
{
	cal_end_point_tra_px = new FifthOrderTrajectory;
	cal_end_point_tra_py = new FifthOrderTrajectory;
	cal_end_point_tra_pz = new FifthOrderTrajectory;

	cal_end_point_tra_alpha = new FifthOrderTrajectory;
	cal_end_point_tra_betta = new FifthOrderTrajectory;
	cal_end_point_tra_kamma = new FifthOrderTrajectory;

	kinematics = new Kinematics;

	current_pose_change_.resize(6,2);
	current_pose_change_.fill(0);

	is_moving_check = false;

}

CalRad::~CalRad()
{
}
Eigen::MatrixXd CalRad::cal_end_point_to_rad(Eigen::MatrixXd eP_) // end point 6 X 8 행렬을 생 6은 xyz roll pitch yaw 8은 trajectory 입력
{
	result_joint.resize(7,1); // 6DOF LEG
	result_joint.fill(0);

	Eigen::MatrixXd end_point_;
	end_point_.resize(6,1);
	end_point_.fill(0);


	if( cal_end_point_tra_px    -> detect_change_final_value(eP_(0,1), eP_(0,3), eP_(0,7))||
			cal_end_point_tra_py    -> detect_change_final_value(eP_(1,1), eP_(1,3), eP_(1,7))||
			cal_end_point_tra_pz    -> detect_change_final_value(eP_(2,1), eP_(2,3), eP_(2,7))||
			cal_end_point_tra_alpha -> detect_change_final_value(eP_(3,1), eP_(3,3), eP_(3,7))||
			cal_end_point_tra_betta -> detect_change_final_value(eP_(4,1), eP_(4,3), eP_(4,7))||
			cal_end_point_tra_kamma -> detect_change_final_value(eP_(5,1), eP_(5,3), eP_(5,7)) )
	{
		current_pose_change_(0,0) = cal_end_point_tra_px    -> current_pose;
		current_pose_change_(0,1) = cal_end_point_tra_px    -> current_velocity;
		cal_end_point_tra_px    -> current_time = 0;

		current_pose_change_(1,0) = cal_end_point_tra_py    -> current_pose;
		current_pose_change_(1,1) = cal_end_point_tra_py    -> current_velocity;
		cal_end_point_tra_py    -> current_time = 0;

		current_pose_change_(2,0) = cal_end_point_tra_pz    -> current_pose;
		current_pose_change_(2,1) = cal_end_point_tra_pz    -> current_velocity;
		cal_end_point_tra_pz    -> current_time = 0;

		current_pose_change_(3,0) = cal_end_point_tra_alpha -> current_pose;
		current_pose_change_(3,1) = cal_end_point_tra_alpha -> current_velocity;
		cal_end_point_tra_alpha    -> current_time = 0;

		current_pose_change_(4,0) = cal_end_point_tra_betta -> current_pose;
		current_pose_change_(4,1) = cal_end_point_tra_betta -> current_velocity;
		cal_end_point_tra_betta    -> current_time = 0;

		current_pose_change_(5,0) = cal_end_point_tra_kamma -> current_pose;
		current_pose_change_(5,1) = cal_end_point_tra_kamma -> current_velocity;
		cal_end_point_tra_kamma    -> current_time = 0;

		ROS_INFO("change!");
	}



	/*if( cal_end_point_tra_px    -> detect_change_final_value(eP_(0,1), eP_(0,3), eP_(0,7)))
	{
		current_pose_change_(0,0) = cal_end_point_tra_px    -> current_pose;
		current_pose_change_(0,1) = cal_end_point_tra_px    -> current_velocity;
		ROS_INFO("change!");
	}
	if(cal_end_point_tra_py    -> detect_change_final_value(eP_(1,1), eP_(1,3), eP_(1,7)))
	{
		current_pose_change_(1,0) = cal_end_point_tra_py    -> current_pose;
		current_pose_change_(1,1) = cal_end_point_tra_py    -> current_velocity;
		ROS_INFO("change!");
	}
	if(cal_end_point_tra_pz    -> detect_change_final_value(eP_(2,1), eP_(2,3), eP_(2,7)))
	{
		current_pose_change_(2,0) = cal_end_point_tra_pz    -> current_pose;
		current_pose_change_(2,1) = cal_end_point_tra_pz    -> current_velocity;
		ROS_INFO("change!");
	}
	if(cal_end_point_tra_alpha -> detect_change_final_value(eP_(3,1), eP_(3,3), eP_(3,7)))
	{
		current_pose_change_(3,0) = cal_end_point_tra_alpha -> current_pose;
		current_pose_change_(3,1) = cal_end_point_tra_alpha -> current_velocity;
		ROS_INFO("change!");
	}
	if(cal_end_point_tra_betta -> detect_change_final_value(eP_(4,1), eP_(4,3), eP_(4,7)))
	{
		current_pose_change_(4,0) = cal_end_point_tra_betta -> current_pose;
		current_pose_change_(4,1) = cal_end_point_tra_betta -> current_velocity;
		ROS_INFO("change!");
	}
	if(cal_end_point_tra_kamma -> detect_change_final_value(eP_(5,1), eP_(5,3), eP_(5,7)))
	{
		current_pose_change_(5,0) = cal_end_point_tra_kamma -> current_pose;
		current_pose_change_(5,1) = cal_end_point_tra_kamma -> current_velocity;
		ROS_INFO("change!");
	}*/

	end_point_(0,0) = cal_end_point_tra_px -> fifth_order_traj_gen(current_pose_change_(0,0), eP_(0,1), eP_(0,2), eP_(0,3), eP_(0,4), eP_(0,5), eP_(0,6), eP_(0,7));// initial pose, final pose, initial vel, final vel, initial acc, final acc, initial time, final time
	end_point_(1,0) = cal_end_point_tra_py -> fifth_order_traj_gen(current_pose_change_(1,0), eP_(1,1), eP_(1,2), eP_(1,3), eP_(1,4), eP_(1,5), eP_(1,6), eP_(1,7));
	end_point_(2,0) = cal_end_point_tra_pz -> fifth_order_traj_gen(current_pose_change_(2,0), eP_(2,1), eP_(2,2), eP_(2,3), eP_(2,4), eP_(2,5), eP_(2,6), eP_(2,7));

	end_point_(3,0) = cal_end_point_tra_alpha -> fifth_order_traj_gen(current_pose_change_(3,0), eP_(3,1), eP_(3,2), eP_(3,3), eP_(3,4), eP_(3,5), eP_(3,6), eP_(3,7));
	end_point_(4,0) = cal_end_point_tra_betta -> fifth_order_traj_gen(current_pose_change_(4,0), eP_(4,1), eP_(4,2), eP_(4,3), eP_(4,4), eP_(4,5), eP_(4,6), eP_(4,7));
	end_point_(5,0) = cal_end_point_tra_kamma -> fifth_order_traj_gen(current_pose_change_(5,0), eP_(5,1), eP_(5,2), eP_(5,3), eP_(5,4), eP_(5,5), eP_(5,6), eP_(5,7));

	kinematics->InverseKinematics(end_point_(0,0), end_point_(1,0), end_point_(2,0), end_point_(3,0), end_point_(4,0), end_point_(5,0)); // pX pY pZ alpha betta kamma

	result_joint = kinematics->joint_radian;
	is_moving_check = cal_end_point_tra_px->is_moving_traj;

	return result_joint;
}



