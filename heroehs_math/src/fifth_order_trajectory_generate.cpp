
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

}

FifthOrderTrajectory::~FifthOrderTrajectory()
{

}

bool FifthOrderTrajectory::detect_change_trajectory_final_pose(double pose_)
{

}
bool FifthOrderTrajectory::detect_change_trajectory_final_time(double time_)
{

}
bool FifthOrderTrajectory::detect_change_trajectory_final_velocity(double velocity_)
{

}

double FifthOrderTrajectory::fifth_order_traj_gen(double initial_value_, double final_value_,double initial_velocity_, double final_velocity_ , double initial_time_, double final_time_)
{
	current_time = current_time + 0.008;

	double a[6] = {0,0,0,0,0,0};
	double d_t = 0;
	double trajectory_final_value = 0;

	d_t = final_time_ - initial_time_;
	a[0] = initial_value_;
	a[1] = initial_velocity_;
	a[2] = 0;
	a[3] = (20*(final_value_ - initial_value_) - (8*final_velocity_ + 12*initial_velocity_)*d_t)/(2*pow(d_t,3));
	a[4] = (30*(initial_value_ - final_value_) + (14*final_velocity_ + 16*initial_velocity_)*d_t)/(2*pow(d_t,4));
	a[5] = (12*(final_value_ - initial_value_) - 6*(final_velocity_ + initial_velocity_)*d_t)/(2*pow(d_t,5));

	trajectory_final_value = a[0] + a[1]*current_time + a[2]*(pow(current_time,2)) + a[3]*(pow(current_time,3)) + a[4]*(pow(current_time,4)) + a[5]*(pow(current_time,5));

	return trajectory_final_value;


}
