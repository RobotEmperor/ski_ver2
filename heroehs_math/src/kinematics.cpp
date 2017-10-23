
/*
 * fifth_order_trajectory_generate.cpp
 *
 *  Created on: 2017. 10. 23.
 *      Author: RobotEmperor
 */

#include "heroehs_math/kinematics.h"

using namespace heroehs_math;

Kinematics::Kinematics()
{
	joint_radian.resize(7,1);
	joint_radian.fill(0);

	// kinematics variables //
	sin_theta5_ = 0.0;
	cos_theta5_ = 0.0;
	alpha_5_ = 0.0;
	betta_5_ = 0.0;
	kamma_5_ = 0.0;
	csai_5_ = 0.0;

	sin_theta1_ = 0.0;
	cos_theta1_ = 0.0;

	sin_theta2_ = 0.0;
	cos_theta2_ = 0.0;

	cos_theta2_temp_1_ = 0.0;
	cos_theta2_temp_2_ = 0.0;
	cos_theta2_temp_3_ = 0.0;

	sin_theta3_ = 0.0;
	cos_theta3_ = 0.0;

	l05_xz_ = 0.0;
	l05_yz_ = 0.0;
	l06_yz_ = 0.0;

	r11_= 0.0;
	r12_= 0.0;
	r13_= 0.0;

	r21_= 0.0;
	r22_= 0.0;
	r23_= 0.0;

	r31_= 0.0;
	r32_= 0.0;
	r33_= 0.0;

	// DH convention variables
	dh_alpha[0] = 0;
	dh_alpha[1] = M_PI/2;
	dh_alpha[2] = -M_PI/2;
	dh_alpha[3] = -M_PI/2;
	dh_alpha[4] = 0;
	dh_alpha[5] = M_PI/2;
	dh_alpha[6] = 0;

	dh_link[0] = 0;
	dh_link[1] = 0;
	dh_link[2] = 0;
	dh_link[3] = 0;
	dh_link[4] = 0.235;
	dh_link[5] = 0;
	dh_link[6] = 0.12;

	dh_link_d[0] = 0;
	dh_link_d[1] = 0;
	dh_link_d[2] = 0;
	dh_link_d[3] = 0.225;
	dh_link_d[4] = 0;
	dh_link_d[5] = 0;
	dh_link_d[6] = 0;

	real_theta[0] = 0;
	real_theta[1] = 0;
	real_theta[2] = 0;
	real_theta[3] = 0;
	real_theta[4] = 0;
	real_theta[5] = 0;
	real_theta[6] = 0;
}


Kinematics::~Kinematics()
{

}

void Kinematics::FowardKnematics()
{

}

void Kinematics::InverseKinematics(double pX_, double pY_, double pZ_, double z_alpha_, double y_betta_, double x_kamma_)
{
	bool check_invertible_ = false;

	Eigen::Matrix4d P_;
	Eigen::Matrix4d P_inverse_;
	P_inverse_.fill(0);
	P_.fill(0);

	// Euler angle initialize
	P_(0,0) = cos(z_alpha_)*cos(y_betta_); //r11
	r11_ = cos(z_alpha_)*cos(y_betta_);

	P_(0,1) = cos(z_alpha_)*sin(y_betta_)*sin(x_kamma_) - sin(z_alpha_)*cos(x_kamma_);// r12
	r12_ =  cos(z_alpha_)*sin(y_betta_)*sin(x_kamma_) - sin(z_alpha_)*cos(x_kamma_);

	P_(0,2) = cos(z_alpha_)*sin(y_betta_)*cos(x_kamma_) + sin(z_alpha_)*sin(x_kamma_); //r13
	r13_ = cos(z_alpha_)*sin(y_betta_)*cos(x_kamma_) + sin(z_alpha_)*sin(x_kamma_);

	P_(0,3) = pX_;

	P_(1,0) = sin(z_alpha_)*cos(y_betta_); // r21
	r21_ =  sin(z_alpha_)*cos(y_betta_);

	P_(1,1) = sin(z_alpha_)*sin(y_betta_)*sin(x_kamma_) + cos(z_alpha_)*cos(x_kamma_); //r22
	r22_ = sin(z_alpha_)*sin(y_betta_)*sin(x_kamma_) + cos(z_alpha_)*cos(x_kamma_);

	P_(1,2) = sin(z_alpha_)*sin(y_betta_)*cos(x_kamma_) - cos(z_alpha_)*sin(x_kamma_);  // r23
	r23_ = sin(z_alpha_)*sin(y_betta_)*cos(x_kamma_) - cos(z_alpha_)*sin(x_kamma_);

	P_(1,3) = pY_;


	P_(2,0) = -sin(y_betta_); // r31
	r31_ = -sin(y_betta_);

	P_(2,1) = cos(y_betta_)*sin(x_kamma_);  //r32
	r32_ = cos(y_betta_)*sin(x_kamma_);

	P_(2,2) = cos(y_betta_)*cos(x_kamma_);  // r33
	r33_ = cos(y_betta_)*cos(x_kamma_);

	P_(2,3) = pZ_;

	P_(3,0) = 0;
	P_(3,1) = 0;
	P_(3,2) = 0;
	P_(3,3) = 1;
	//
	P_inverse_ = P_.inverse();

	// check invertible ////////////////////////////////////
	P_.computeInverseWithCheck(P_inverse_, check_invertible_);
	if(check_invertible_ == false)
		return;

	//// inverse kinematics process run! ///
	// theta 4 //
	l05_xz_ = sqrt(pow(pX_+ (dh_link[6]*r13_), 2)+ pow(pY_+ (dh_link[6]*r23_), 2) + pow(pZ_+ (dh_link[6]*r33_), 2));

	if((pow(l05_xz_,2) - pow(dh_link_d[3],2) - pow(dh_link[4],2))/(2*dh_link_d[3]*dh_link[4]) > 1.0 || (pow(l05_xz_,2) - pow(dh_link_d[3],2) - pow(dh_link[4],2))/(2*dh_link_d[3]*dh_link[4]) < -1.0)
	{
		real_theta[4] = 0;
	}
	else
	{
		real_theta[4] = acos((pow(l05_xz_,2) - pow(dh_link_d[3],2) - pow(dh_link[4],2))/(2*dh_link_d[3]*dh_link[4])); //get theta 4
	}
	real_theta[4] = floor(100000.*(real_theta[4]+0.000005))/100000.;

	/////////////
	// theta 6 //
	l06_yz_ = sqrt(pow(P_inverse_(1,3),2) + pow(P_inverse_(2,3),2));

	l05_yz_ = sqrt(pow(P_inverse_(1,3),2) + pow(P_inverse_(2,3)-dh_link[6],2));

	if((pow(l06_yz_,2) - pow(l05_yz_,2) - pow(dh_link[6],2))/ (2*l05_yz_*dh_link[6]) > 1.0 || (pow(l06_yz_,2) - pow(l05_yz_,2) - pow(dh_link[6],2))/ (2*l05_yz_*dh_link[6]) < -1.0)
	{
		real_theta[6] = 0;

	}
	else
	{
		if(P_inverse_(1,3) > 0)
			real_theta[6] =  acos((pow(l06_yz_,2) - pow(l05_yz_,2) - pow(dh_link[6],2))/ (2*l05_yz_*dh_link[6]));
		else if (P_inverse_(1,3) == 0)
			real_theta[6] = 0;
		else if (P_inverse_(1,3) < 0)
			real_theta[6] =  -acos((pow(l06_yz_,2) - pow(l05_yz_,2) - pow(dh_link[6],2))/ (2*l05_yz_*dh_link[6])); // get theta 6;
	}
	real_theta[6] = floor(100000.*(real_theta[6]+0.000005))/100000.;

	/////////////
	// theta 5 //

	alpha_5_= dh_link_d[3]*(cos(real_theta[6])*r33_*cos(real_theta[4]) - r31_*sin(real_theta[4]) + r32_*sin(real_theta[6])*cos(real_theta[4])) + dh_link[4]*(r33_*cos(real_theta[6]) + r32_*sin(real_theta[6]));
	betta_5_= -dh_link_d[3]*(cos(real_theta[6])*r33_*sin(real_theta[4]) + r31_*cos(real_theta[4]) + sin(real_theta[6])*r32_*sin(real_theta[4])) - dh_link[4]*r31_;
	kamma_5_= dh_link_d[3]*(cos(real_theta[6])*r13_*cos(real_theta[4]) - r11_*sin(real_theta[4]) + r12_*sin(real_theta[6])*cos(real_theta[4])) + dh_link[4]*(r13_*cos(real_theta[6]) + r12_*sin(real_theta[6]));
	csai_5_= -dh_link_d[3]*(cos(real_theta[6])*r13_*sin(real_theta[4]) + r11_*cos(real_theta[4]) + sin(real_theta[6])*r12_*sin(real_theta[4])) - dh_link[4]*r11_;

	sin_theta5_ = (((-pZ_- r33_*dh_link[6])*kamma_5_) - ((-pX_- r13_*dh_link[6])*alpha_5_))/(betta_5_*kamma_5_ - alpha_5_*csai_5_);
	cos_theta5_ = (((-pZ_- r33_*dh_link[6])*csai_5_) - ((-pX_- r13_*dh_link[6])*betta_5_))/( -betta_5_*kamma_5_ + alpha_5_*csai_5_);

	real_theta[5] = floor(100000.*(atan2(sin_theta5_,cos_theta5_)+0.000005))/100000.;

	/////////////
	// theta 1 //

	sin_theta1_ = - (r11_*cos(real_theta[4])*sin(real_theta[5]) + r11_*cos(real_theta[5])*sin(real_theta[4]) - r13_*cos(real_theta[6])*cos(real_theta[4])*cos(real_theta[5]) + r13_*cos(real_theta[6])*sin(real_theta[4])*sin(real_theta[5]) - r12_*sin(real_theta[6])*cos(real_theta[4])*cos(real_theta[5]) + r12_*sin(real_theta[6])*sin(real_theta[4])*sin(real_theta[5]));
	cos_theta1_ = - (r31_*cos(real_theta[4])*sin(real_theta[5]) + r31_*cos(real_theta[5])*sin(real_theta[4]) - r33_*cos(real_theta[6])*cos(real_theta[4])*cos(real_theta[5]) + r33_*cos(real_theta[6])*sin(real_theta[4])*sin(real_theta[5]) - r32_*sin(real_theta[6])*cos(real_theta[4])*cos(real_theta[5]) + r32_*sin(real_theta[6])*sin(real_theta[4])*sin(real_theta[5]));

	real_theta[1] = floor(100000.*(atan2(sin_theta1_,cos_theta1_)+0.000005))/100000.;

	/////////////
	// theta 2 //

	cos_theta2_temp_1_ = r33_*cos(real_theta[1])*cos(real_theta[4])*cos(real_theta[5])*cos(real_theta[6]) - r33_*cos(real_theta[1])*sin(real_theta[4])*sin(real_theta[5])*cos(real_theta[6]) + r13_*sin(real_theta[1])*cos(real_theta[4])*cos(real_theta[5])*cos(real_theta[6]) - r13_*sin(real_theta[1])*sin(real_theta[4])*sin(real_theta[5])*cos(real_theta[6]);
	cos_theta2_temp_2_ = -r31_*cos(real_theta[1])*cos(real_theta[4])*sin(real_theta[5]) -r31_*cos(real_theta[1])*sin(real_theta[4])*cos(real_theta[5]) -r11_*sin(real_theta[1])*cos(real_theta[4])*sin(real_theta[5]) -r11_*sin(real_theta[1])*sin(real_theta[4])*cos(real_theta[5]);
	cos_theta2_temp_3_ = r32_*cos(real_theta[1])*cos(real_theta[4])*cos(real_theta[5])*sin(real_theta[6]) - r32_*cos(real_theta[1])*sin(real_theta[4])*sin(real_theta[5])*sin(real_theta[6]) + r12_*sin(real_theta[1])*cos(real_theta[4])*cos(real_theta[5])*sin(real_theta[6]) - r12_*sin(real_theta[1])*sin(real_theta[4])*sin(real_theta[5])*sin(real_theta[6]);

	sin_theta2_ = r21_*cos(real_theta[4])*sin(real_theta[5]) + r21_*sin(real_theta[4])*cos(real_theta[5]) - r23_*cos(real_theta[4])*cos(real_theta[5])*cos(real_theta[6]) + r23_*sin(real_theta[4])*sin(real_theta[5])*cos(real_theta[6]) - r22_*cos(real_theta[4])*cos(real_theta[5])*sin(real_theta[6]) + r22_*sin(real_theta[4])*sin(real_theta[5])*sin(real_theta[6]);
	//cos_theta2_ = (cos(real_theta[6])*((r33_*cos(real_theta[1])) + (r13_*sin(real_theta[1])))*(cos(real_theta[4])*cos(real_theta[5]) - sin(real_theta[4])*sin(real_theta[5]))) - (((r31_*cos(real_theta[1])) + (r11_*sin(real_theta[1])))*(cos(real_theta[4])*sin(real_theta[5]) + cos(real_theta[5])*sin(real_theta[4]))) + (sin(real_theta[6])*((r32_*cos(real_theta[1])) + (r12_*sin(real_theta[1])))*(cos(real_theta[4])*cos(real_theta[5]) - sin(real_theta[4])*sin(real_theta[5])));
	cos_theta2_ = cos_theta2_temp_1_  + cos_theta2_temp_2_ + cos_theta2_temp_3_;

	real_theta[2] = floor(100000.*(atan2(sin_theta2_,cos_theta2_)+0.000005))/100000.;

	/////////////
	// theta 3 //

	sin_theta3_ = r21_*cos(real_theta[4]+real_theta[5]) + r23_*cos(real_theta[6])*sin(real_theta[4]+real_theta[5]) + r22_*sin(real_theta[6])*sin(real_theta[4]+real_theta[5]);
	cos_theta3_ = r22_*cos(real_theta[6]) - r23_*sin(real_theta[6]);

	real_theta[3] = floor(100000.*(atan2(sin_theta3_,cos_theta3_)+0.000005))/100000.;


	joint_radian << 0 , real_theta[1], real_theta[2] , real_theta[3] , real_theta[4] , real_theta[5] , real_theta[6];

}
