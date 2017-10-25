
/*
 * kinematics.h
 *
 *  Created on: 2017. 10. 23.
 *      Author: RobotEmperor
 */
#ifndef HEROEHS_MATH_KINEMATICS_H_
#define HEROEHS_MATH_KINEMATICS_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include "robotis_math/robotis_math.h"

using namespace Eigen;
using namespace std;


namespace heroehs_math
{
class Kinematics
{
public:
	Kinematics();
	~Kinematics();

	void FowardKnematics(double joint[7]);
	void InverseKinematics(double pX_, double pY_, double pZ_, double z_alpha_, double y_betta_, double x_kamma_);

	Eigen::MatrixXd P;


	Eigen::MatrixXd joint_radian;

private:

	//forward_kinematics
	Eigen::MatrixXd H[8];


  // inverse_kinematics
	double r11_,r12_,r13_,r21_,r22_,r23_,r31_,r32_,r33_; // euler angle
	double real_theta[7] , dh_alpha[7] , dh_link[7] , dh_link_d[7]; // dh_convention variables

	// equation variables
	double sin_theta5_ , cos_theta5_ , alpha_5_ , betta_5_ , kamma_5_, csai_5_;
	double sin_theta1_ , cos_theta1_;
	double sin_theta2_ , cos_theta2_;
	double cos_theta2_temp_1_ , cos_theta2_temp_2_ , cos_theta2_temp_3_;
	double sin_theta3_ , cos_theta3_;

	///geometical solution link ///
	double l05_xz_;
	double l05_yz_;
	double l06_yz_;






};
}
#endif /* HEROEHS_MATH_KINEMATICS_H_ */
