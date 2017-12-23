
/*
 * fifth_order_trajectory_generate.h
 *
 *  Created on: 2017. 10. 23.
 *      Author: RobotEmperor
 */
#ifndef HEROEHS_MATH_FIFTH_ORDER_TRAJECTORY_GENERATE_H_
#define HEROEHS_MATH_FIFTH_ORDER_TRAJECTORY_GENERATE_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include "robotis_math/robotis_math.h"



namespace heroehs_math
{

class FifthOrderTrajectory

{
public:
	FifthOrderTrajectory();
	~FifthOrderTrajectory();

	double fifth_order_traj_gen(double initial_value_, double final_value_,
			double initial_velocity_, double final_velocity_ ,
			double initial_acc, double final_acc,
			double initial_time_, double final_time_);


	bool   detect_change_final_value(double pose, double velocity_, double time_);
	double fifth_order_traj_gen_one_value(Eigen::MatrixXd joint_);

	bool is_moving_traj;

	double a[6];
	double d_t;
	double trajectory_final_value;


	double initial_time;
	double initial_pose;
	double initial_velocity;
	double initial_acc;


	double current_time;
	double current_pose;
	double current_velocity;
	double current_acc;


	double final_time;
	double final_pose;
	double final_velocity;
	double final_acc;



};




}

#endif /* HEROEHS_MATH_FIFTH_ORDER_TRAJECTORY_GENERATE_H_ */


