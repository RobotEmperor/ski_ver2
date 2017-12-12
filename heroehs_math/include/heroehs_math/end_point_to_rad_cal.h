/*
 * end_point_to_rad_cal.h
 *
 *  Created on: Oct 25, 2017
 *      Author: robotemperor
 */

#ifndef SKI_VER2_HEROEHS_MATH_INCLUDE_HEROEHS_MATH_END_POINT_TO_RAD_CAL_H_
#define SKI_VER2_HEROEHS_MATH_INCLUDE_HEROEHS_MATH_END_POINT_TO_RAD_CAL_H_

#include "heroehs_math/fifth_order_trajectory_generate.h"
#include "heroehs_math/kinematics.h"


namespace heroehs_math
{
class CalRad
{
public:
	CalRad();
	~CalRad();
	bool is_moving_check;

	Eigen::MatrixXd cal_end_point_to_rad(Eigen::MatrixXd eP_);
	double cal_one_joint_rad(Eigen::MatrixXd joint_);


	FifthOrderTrajectory *cal_end_point_tra_px;
	FifthOrderTrajectory *cal_end_point_tra_py;
	FifthOrderTrajectory *cal_end_point_tra_pz;

	FifthOrderTrajectory *cal_end_point_tra_alpha;
	FifthOrderTrajectory *cal_end_point_tra_betta;
	FifthOrderTrajectory *cal_end_point_tra_kamma;

	FifthOrderTrajectory *cal_one_joint_traj_rad;


	Eigen::MatrixXd current_pose_change;
	Eigen::MatrixXd current_one_joint_pose;



private:
	Eigen::MatrixXd result_joint;



};
}










#endif /* SKI_VER2_HEROEHS_MATH_INCLUDE_HEROEHS_MATH_END_POINT_TO_RAD_CAL_H_ */
