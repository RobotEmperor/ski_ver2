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


bool FifthOrderTrajectory::detect_change_final_value(double pose_, double velocity_, double time_)
{
	if(pose_ != final_pose || velocity_ != final_velocity || time_ != final_time  )
	{
		final_pose = pose_;
		final_velocity = velocity_;
		final_time = time_;
		current_time = 0;
				return true;
	}
	else
		return false;

}


double FifthOrderTrajectory::fifth_order_traj_gen(double initial_value_, double final_value_,
		double initial_velocity_, double final_velocity_ ,
		double initial_acc_, double final_acc_,
		double initial_time_, double final_time_)
{

	current_time = current_time + 0.008;

	if(current_time > final_time_)
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
