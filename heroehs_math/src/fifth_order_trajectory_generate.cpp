/*
 * fifth_order_trajectory_generate.cpp
 *
 *  Created on: 2017. 10. 23.
 *      Author: RobotEmperor
 */


#include "heroehs_math/fifth_order_trajectory_generate.h"


using namespace heroehs_math;


FifthOrderTrajectory::FifthOrderTrajectory()
{
	current_time = 0;
	is_moving_traj = false;

}

FifthOrderTrajectory::~FifthOrderTrajectory()
{

}

bool FifthOrderTrajectory::detect_change_trajectory_final_pose(double pose_)
{
	if(pose_ != final_pose)
		return true;
	else
		return false;

}
bool FifthOrderTrajectory::detect_change_trajectory_final_velocity(double velocity_)
{
	if(velocity_ != final_velocity)
		return true;
	else
		return false;

}
bool FifthOrderTrajectory::detect_change_trajectory_final_time(double time_)
{
	if(time_ != final_time)
		return true;
	else
		return false;

}


double FifthOrderTrajectory::fifth_order_traj_gen(double initial_value_, double final_value_,
		double initial_velocity_, double final_velocity_ ,
		double initial_acc_, double final_acc_,
		double initial_time_, double final_time_)
{

	if(current_time == 0)
	{

		initial_pose = initial_value_;
		initial_velocity = initial_velocity_;
		initial_acc = initial_acc_;

		final_pose = final_value_;
		final_velocity = final_velocity_;
		final_acc = final_acc_;

		final_time = final_time_;

		//ROS_INFO("1");
	}

	if(detect_change_trajectory_final_pose(final_value_)||detect_change_trajectory_final_velocity(final_velocity_)||detect_change_trajectory_final_time(final_time_))
	{
		initial_pose = current_pose;
		initial_velocity = current_velocity;
		initial_acc = current_acc;
		final_time = final_time_;

		current_time = 0;

		//ROS_INFO("2");

	}


	current_time = current_time + 0.008;


	if(current_time > final_time)
	{
		is_moving_traj = false;
		return current_pose;
	}

	else
	{
		double a[6] = {0,0,0,0,0,0};
		double d_t = 0;
		double trajectory_final_value = 0;

		d_t = final_time_ - initial_time_;
		a[0] = initial_value_;
		a[1] = initial_velocity_;
		a[2] = initial_acc_/2;
		a[3] = (20*(final_value_ - initial_value_) - (8*final_velocity_ + 12*initial_velocity_)*d_t - (3*final_acc_ - initial_acc_)*pow(d_t,2))/(2*pow(d_t,3));
		a[4] = (30*(initial_value_ - final_value_) + (14*final_velocity_ + 16*initial_velocity_)*d_t + (3*final_acc_ - 2*initial_acc_)*pow(d_t,2))/(2*pow(d_t,4));
		a[5] = (12*(final_value_ - initial_value_) - 6*(final_velocity_ + initial_velocity_)*d_t - (final_acc_ - initial_acc_)*pow(d_t,2))/(2*pow(d_t,5));

		trajectory_final_value = a[0] + a[1]*current_time + a[2]*(pow(current_time,2)) + a[3]*(pow(current_time,3)) + a[4]*(pow(current_time,4)) + a[5]*(pow(current_time,5));

		current_pose = trajectory_final_value;
		current_velocity = a[1] + 2*a[2]*current_time + 3*a[3]*(pow(current_time,2)) + 4*a[4]*(pow(current_time,3)) + 5*a[5]*(pow(current_time,4));
		current_acc = 2*a[2] + 6*a[3]*current_time + 12*a[4]*pow(current_time,2) + 20*a[5]*pow(current_time,4);

		is_moving_traj = true;

		return trajectory_final_value;
	}

}
