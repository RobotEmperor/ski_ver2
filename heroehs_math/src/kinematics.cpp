
/*
 * fifth_order_trajectory_generate.cpp
 *
 *  Created on: 2017. 10. 23.
 *      Author: RobotEmperor
 */

#include "heroehs_math/kinematics.h"

using namespace heroehs_math;
//////////////////////////////////////////////////////////Leg Kinematics/////////////////////////////////////////////////////////////////////////
Kinematics::Kinematics()
{
	joint_radian.resize(7,1);
	joint_radian.fill(0);

	// kinematics variables //
	P_inverse_.fill(0);
	P_.fill(0);

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
	dh_link[4] = 0.225;
	dh_link[5] = 0;
	dh_link[6] = 0.135;

	total_length_ = 0.585;
	sensor_length_ = 0.135;

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

	for(int i=0; i<8;i++)
	{
		H[i].resize(4,4);
		H[i].fill(0);
	}
	center_to_sensor_transform_right.resize(4,4);
	center_to_sensor_transform_right.fill(0);
	center_to_sensor_transform_left.resize(4,4);
	center_to_sensor_transform_left.fill(0);
	center_to_foot_transform_left_leg.resize(4,4);
	center_to_foot_transform_left_leg.fill(0);
	center_to_foot_transform_right_leg.resize(4,4);
	center_to_foot_transform_right_leg.fill(0);
	H_ground_to_center.resize(4,4);
	H_ground_to_center.fill(0);

	H[7] << 0 , 0, -1, 0,
			0 , 1,  0, 0,
			1 , 0,  0, 0,
			0 , 0,  0, 1;

	H_ground_to_center << 1 , 0, 0, 0,
			0 , 1, 0, 0,
			0 , 0, 1, total_length_,
			0 , 0, 0, 1;
}
Kinematics::~Kinematics()
{
}
void Kinematics::FowardKnematics(double joint[7], std::string left_right)
{
	double sum_theta[7] = {0,0,0,0,0,0,0};
	double offset_theta[7] = {0, 0, (M_PI)/2, -(M_PI)/2, (M_PI)/2, 0, 0};

	for(int i=1; i<7; i++)
	{
		sum_theta[i] = joint[i]+ offset_theta[i];
	}
	for(int i=1; i<7; i++)
	{

		H[i](0,0) = floor(100000.*(cos(sum_theta[i])+0.000005))/100000.;
		H[i](0,1) = floor(100000.*(-cos(dh_alpha[i])*sin(sum_theta[i])+0.000005))/100000.;
		H[i](0,2) = floor(100000.*(sin(dh_alpha[i])*sin(sum_theta[i])+0.000005))/100000.;
		H[i](0,3) = floor(100000.*(dh_link[i]*cos(sum_theta[i])+0.000005))/100000.;

		H[i](1,0) = floor(100000.*(sin(sum_theta[i])+0.000005))/100000.;
		H[i](1,1) = floor(100000.*(cos(dh_alpha[i])*cos(sum_theta[i])+0.000005))/100000.;
		H[i](1,2) = floor(100000.*(-sin(dh_alpha[i])*cos(sum_theta[i])+0.000005))/100000.;
		H[i](1,3) = floor(100000.*(dh_link[i]*sin(sum_theta[i])+0.000005))/100000.;

		H[i](2,0) =0;
		H[i](2,1) = floor(100000.*(sin(dh_alpha[i])+0.000005))/100000.;
		H[i](2,2) = floor(100000.*(cos(dh_alpha[i])+0.000005))/100000.;
		H[i](2,3) = -dh_link_d[i];

		H[i](3,0) =0;
		H[i](3,1) =0;
		H[i](3,2) =0;
		H[i](3,3) =1;

		H[i](0,0) = cos(sum_theta[i]);
		H[i](0,1) = -cos(dh_alpha[i])*sin(sum_theta[i]);
		H[i](0,2) = sin(dh_alpha[i])*sin(sum_theta[i]);
		H[i](0,3) = dh_link[i]*cos(sum_theta[i]);

		H[i](1,0) = sin(sum_theta[i]);
		H[i](1,1) = cos(dh_alpha[i])*cos(sum_theta[i]);
		H[i](1,2) = -sin(dh_alpha[i])*cos(sum_theta[i]);
		H[i](1,3) = dh_link[i]*sin(sum_theta[i]);

		H[i](2,0) =0;
		H[i](2,1) = sin(dh_alpha[i]);
		H[i](2,2) = cos(dh_alpha[i]);
		H[i](2,3) = -dh_link_d[i];

		H[i](3,0) =0;
		H[i](3,1) =0;
		H[i](3,2) =0;
		H[i](3,3) =1;
	}

	H[0](0,0) = 0;
	H[0](0,1) = -1;
	H[0](0,2) = 0;
	H[0](0,3) = 0;

	H[0](1,0) = 0;
	H[0](1,1) = 0;
	H[0](1,2) = 1;
	H[0](1,3) = -0.105;

	H[0](2,0) = -1;
	H[0](2,1) = 0;
	H[0](2,2) = 0;
	H[0](2,3) = 0;

	H[0](3,0) = 0;
	H[0](3,1) = 0;
	H[0](3,2) = 0;
	H[0](3,3) = 1;
	//// foot frame 을 Global frame 과 일치 시킨다.
	if(!left_right.compare("left")) // left
	{
		H[0](1,3) = 0.105;
		center_to_foot_transform_left_leg = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6]*H[7];
	}
	else // right
	{
		H[0](1,3) = -0.105;
		center_to_foot_transform_right_leg = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6]*H[7];
	}
}
void Kinematics::FowardKnematicsCenterToSensorRight(double joint[7])
{
	double sum_theta[7] = {0,0,0,0,0,0,0};
	double offset_theta[7] = {0, 0, (M_PI)/2, -(M_PI)/2, (M_PI)/2, 0, 0};

	for(int i=1; i<7; i++)
	{
		sum_theta[i] = joint[i]+ offset_theta[i];
	}
	for(int i=1; i<7; i++)
	{

		H[i](0,0) = floor(100000.*(cos(sum_theta[i])+0.000005))/100000.;
		H[i](0,1) = floor(100000.*(-cos(dh_alpha[i])*sin(sum_theta[i])+0.000005))/100000.;
		H[i](0,2) = floor(100000.*(sin(dh_alpha[i])*sin(sum_theta[i])+0.000005))/100000.;
		H[i](0,3) = floor(100000.*(dh_link[i]*cos(sum_theta[i])+0.000005))/100000.;

		H[i](1,0) = floor(100000.*(sin(sum_theta[i])+0.000005))/100000.;
		H[i](1,1) = floor(100000.*(cos(dh_alpha[i])*cos(sum_theta[i])+0.000005))/100000.;
		H[i](1,2) = floor(100000.*(-sin(dh_alpha[i])*cos(sum_theta[i])+0.000005))/100000.;
		H[i](1,3) = floor(100000.*(dh_link[i]*sin(sum_theta[i])+0.000005))/100000.;

		H[i](2,0) =0;
		H[i](2,1) = floor(100000.*(sin(dh_alpha[i])+0.000005))/100000.;
		H[i](2,2) = floor(100000.*(cos(dh_alpha[i])+0.000005))/100000.;
		H[i](2,3) = -dh_link_d[i];

		H[i](3,0) =0;
		H[i](3,1) =0;
		H[i](3,2) =0;
		H[i](3,3) =1;

		H[i](0,0) = cos(sum_theta[i]);
		H[i](0,1) = -cos(dh_alpha[i])*sin(sum_theta[i]);
		H[i](0,2) = sin(dh_alpha[i])*sin(sum_theta[i]);
		H[i](0,3) = dh_link[i]*cos(sum_theta[i]);

		H[i](1,0) = sin(sum_theta[i]);
		H[i](1,1) = cos(dh_alpha[i])*cos(sum_theta[i]);
		H[i](1,2) = -sin(dh_alpha[i])*cos(sum_theta[i]);
		H[i](1,3) = dh_link[i]*sin(sum_theta[i]);

		H[i](2,0) =0;
		H[i](2,1) = sin(dh_alpha[i]);
		H[i](2,2) = cos(dh_alpha[i]);
		H[i](2,3) = -dh_link_d[i];

		H[i](3,0) =0;
		H[i](3,1) =0;
		H[i](3,2) =0;
		H[i](3,3) =1;
	}

	//// sensor transformation
	H[6](0,3) = floor(100000.*(sensor_length_*cos(sum_theta[6])+0.000005))/100000.;
	H[6](1,3) = floor(100000.*(sensor_length_*sin(sum_theta[6])+0.000005))/100000.;
	H[6](0,3) = sensor_length_*cos(sum_theta[6]);
	H[6](1,3) = sensor_length_*sin(sum_theta[6]);

	//////// 베이스 좌표계와 일치시킨다.////// origin frame
	H[0](0,0) = 0;
	H[0](0,1) = -1;
	H[0](0,2) = 0;
	H[0](0,3) = 0;

	H[0](1,0) = 0;
	H[0](1,1) = 0;
	H[0](1,2) = 1;
	H[0](1,3) = -0.105;

	H[0](2,0) = -1;
	H[0](2,1) = 0;
	H[0](2,2) = 0;
	H[0](2,3) = 0;

	H[0](3,0) = 0;
	H[0](3,1) = 0;
	H[0](3,2) = 0;
	H[0](3,3) = 1;
	//// foot frame 을 Global frame 과 일치 시킨다.

	/////////////////////////////////

	center_to_sensor_transform_right = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6]*H[7];
}

void Kinematics::FowardKnematicsCenterToSensorLeft(double joint[7])
{
	double sum_theta[7] = {0,0,0,0,0,0,0};
	double offset_theta[7] = {0, 0, (M_PI)/2, -(M_PI)/2, (M_PI)/2, 0, 0};

	for(int i=1; i<7; i++)
	{
		sum_theta[i] = joint[i]+ offset_theta[i];
	}
	for(int i=1; i<7; i++)
	{

		H[i](0,0) = floor(100000.*(cos(sum_theta[i])+0.000005))/100000.;
		H[i](0,1) = floor(100000.*(-cos(dh_alpha[i])*sin(sum_theta[i])+0.000005))/100000.;
		H[i](0,2) = floor(100000.*(sin(dh_alpha[i])*sin(sum_theta[i])+0.000005))/100000.;
		H[i](0,3) = floor(100000.*(dh_link[i]*cos(sum_theta[i])+0.000005))/100000.;

		H[i](1,0) = floor(100000.*(sin(sum_theta[i])+0.000005))/100000.;
		H[i](1,1) = floor(100000.*(cos(dh_alpha[i])*cos(sum_theta[i])+0.000005))/100000.;
		H[i](1,2) = floor(100000.*(-sin(dh_alpha[i])*cos(sum_theta[i])+0.000005))/100000.;
		H[i](1,3) = floor(100000.*(dh_link[i]*sin(sum_theta[i])+0.000005))/100000.;

		H[i](2,0) =0;
		H[i](2,1) = floor(100000.*(sin(dh_alpha[i])+0.000005))/100000.;
		H[i](2,2) = floor(100000.*(cos(dh_alpha[i])+0.000005))/100000.;
		H[i](2,3) = -dh_link_d[i];

		H[i](3,0) =0;
		H[i](3,1) =0;
		H[i](3,2) =0;
		H[i](3,3) =1;

		H[i](0,0) = cos(sum_theta[i]);
		H[i](0,1) = -cos(dh_alpha[i])*sin(sum_theta[i]);
		H[i](0,2) = sin(dh_alpha[i])*sin(sum_theta[i]);
		H[i](0,3) = dh_link[i]*cos(sum_theta[i]);

		H[i](1,0) = sin(sum_theta[i]);
		H[i](1,1) = cos(dh_alpha[i])*cos(sum_theta[i]);
		H[i](1,2) = -sin(dh_alpha[i])*cos(sum_theta[i]);
		H[i](1,3) = dh_link[i]*sin(sum_theta[i]);

		H[i](2,0) =0;
		H[i](2,1) = sin(dh_alpha[i]);
		H[i](2,2) = cos(dh_alpha[i]);
		H[i](2,3) = -dh_link_d[i];

		H[i](3,0) =0;
		H[i](3,1) =0;
		H[i](3,2) =0;
		H[i](3,3) =1;
	}

	//// sensor transformation
	H[6](0,3) = floor(100000.*(sensor_length_*cos(sum_theta[6])+0.000005))/100000.;
	H[6](1,3) = floor(100000.*(sensor_length_*sin(sum_theta[6])+0.000005))/100000.;
	H[6](0,3) = sensor_length_*cos(sum_theta[6]);
	H[6](1,3) = sensor_length_*sin(sum_theta[6]);

	//////// 베이스 좌표계와 일치시킨다.////// origin frame
	H[0](0,0) = 0;
	H[0](0,1) = -1;
	H[0](0,2) = 0;
	H[0](0,3) = 0;

	H[0](1,0) = 0;
	H[0](1,1) = 0;
	H[0](1,2) = 1;
	H[0](1,3) = 0.105;

	H[0](2,0) = -1;
	H[0](2,1) = 0;
	H[0](2,2) = 0;
	H[0](2,3) = 0;

	H[0](3,0) = 0;
	H[0](3,1) = 0;
	H[0](3,2) = 0;
	H[0](3,3) = 1;
	//// foot frame 을 Global frame 과 일치 시킨다.
	// H[7]
	/////////////////////////////////

	center_to_sensor_transform_left = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6]*H[7];
}
Eigen::MatrixXd Kinematics::CenterToGroundTransformation(Eigen::MatrixXd point)
{
	Eigen::MatrixXd H_center_to_ground;
	H_center_to_ground.resize(4,4);

	H_center_to_ground << 1 , 0, 0, 0,
			0 , 1, 0, 0,
			0 , 0, 1, -total_length_,
			0 , 0, 0, 1;

	return H_center_to_ground*point;
}

void Kinematics::InverseKinematics(double pX_, double pY_, double pZ_, double z_alpha_, double y_betta_, double x_kamma_)
{

	bool check_invertible_ = false;

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
		return;
	}
	else
	{
		if(isnan(acos((pow(l05_xz_,2) - pow(dh_link_d[3],2) - pow(dh_link[4],2))/(2*dh_link_d[3]*dh_link[4]))))
			return;
		else
			real_theta[4] = acos((pow(l05_xz_,2) - pow(dh_link_d[3],2) - pow(dh_link[4],2))/(2*dh_link_d[3]*dh_link[4])); //get theta 4
	}

	real_theta[4] = floor(100000.*(real_theta[4]+0.000005))/100000.;

	/////////////
	// theta 6 //
	l06_yz_ = sqrt(pow(P_inverse_(1,3),2) + pow(P_inverse_(2,3),2));

	l05_yz_ = sqrt(pow(P_inverse_(1,3),2) + pow(P_inverse_(2,3)-dh_link[6],2));

	if(floor(100000.*((pow(l06_yz_,2) - pow(l05_yz_,2) - pow(dh_link[6],2))/ (2*l05_yz_*dh_link[6]))+0.000005)/100000. > 1.0
			|| floor(100000.*((pow(l06_yz_,2) - pow(l05_yz_,2) - pow(dh_link[6],2))/ (2*l05_yz_*dh_link[6]))+0.000005)/100000.  < -1.0)
	{
		printf("real_theta   %f :: \n",(pow(l06_yz_,2) - pow(l05_yz_,2) - pow(dh_link[6],2))/ (2*l05_yz_*dh_link[6]));
		return;
	}
	else
	{
		if(P_inverse_(1,3) > 0)
			real_theta[6] =  acos((pow(l06_yz_,2) - pow(l05_yz_,2) - pow(dh_link[6],2))/ (2*l05_yz_*dh_link[6]));
		else if (P_inverse_(1,3) == 0)
			real_theta[6] = 0;
		else if (P_inverse_(1,3) < 0)
			real_theta[6] =  -acos((pow(l06_yz_,2) - pow(l05_yz_,2) - pow(dh_link[6],2))/ (2*l05_yz_*dh_link[6])); // get theta 6;

		if(isnan(real_theta[6]))
			return;
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

	if(isnan(floor(100000.*(atan2(sin_theta5_,cos_theta5_)+0.000005))/100000.))
		return;
	else
		real_theta[5] = floor(100000.*(atan2(sin_theta5_,cos_theta5_)+0.000005))/100000.;


	/////////////
	// theta 1 //

	sin_theta1_ = - (r11_*cos(real_theta[4])*sin(real_theta[5]) + r11_*cos(real_theta[5])*sin(real_theta[4]) - r13_*cos(real_theta[6])*cos(real_theta[4])*cos(real_theta[5]) + r13_*cos(real_theta[6])*sin(real_theta[4])*sin(real_theta[5]) - r12_*sin(real_theta[6])*cos(real_theta[4])*cos(real_theta[5]) + r12_*sin(real_theta[6])*sin(real_theta[4])*sin(real_theta[5]));
	cos_theta1_ = - (r31_*cos(real_theta[4])*sin(real_theta[5]) + r31_*cos(real_theta[5])*sin(real_theta[4]) - r33_*cos(real_theta[6])*cos(real_theta[4])*cos(real_theta[5]) + r33_*cos(real_theta[6])*sin(real_theta[4])*sin(real_theta[5]) - r32_*sin(real_theta[6])*cos(real_theta[4])*cos(real_theta[5]) + r32_*sin(real_theta[6])*sin(real_theta[4])*sin(real_theta[5]));

	if(isnan(floor(100000.*(atan2(sin_theta1_,cos_theta1_)+0.000005))/100000.))
		return;
	else
		real_theta[1] = floor(100000.*(atan2(sin_theta1_,cos_theta1_)+0.000005))/100000.;

	/////////////
	// theta 2 //

	cos_theta2_temp_1_ = r33_*cos(real_theta[1])*cos(real_theta[4])*cos(real_theta[5])*cos(real_theta[6]) - r33_*cos(real_theta[1])*sin(real_theta[4])*sin(real_theta[5])*cos(real_theta[6]) + r13_*sin(real_theta[1])*cos(real_theta[4])*cos(real_theta[5])*cos(real_theta[6]) - r13_*sin(real_theta[1])*sin(real_theta[4])*sin(real_theta[5])*cos(real_theta[6]);
	cos_theta2_temp_2_ = -r31_*cos(real_theta[1])*cos(real_theta[4])*sin(real_theta[5]) -r31_*cos(real_theta[1])*sin(real_theta[4])*cos(real_theta[5]) -r11_*sin(real_theta[1])*cos(real_theta[4])*sin(real_theta[5]) -r11_*sin(real_theta[1])*sin(real_theta[4])*cos(real_theta[5]);
	cos_theta2_temp_3_ = r32_*cos(real_theta[1])*cos(real_theta[4])*cos(real_theta[5])*sin(real_theta[6]) - r32_*cos(real_theta[1])*sin(real_theta[4])*sin(real_theta[5])*sin(real_theta[6]) + r12_*sin(real_theta[1])*cos(real_theta[4])*cos(real_theta[5])*sin(real_theta[6]) - r12_*sin(real_theta[1])*sin(real_theta[4])*sin(real_theta[5])*sin(real_theta[6]);

	sin_theta2_ = r21_*cos(real_theta[4])*sin(real_theta[5]) + r21_*sin(real_theta[4])*cos(real_theta[5]) - r23_*cos(real_theta[4])*cos(real_theta[5])*cos(real_theta[6]) + r23_*sin(real_theta[4])*sin(real_theta[5])*cos(real_theta[6]) - r22_*cos(real_theta[4])*cos(real_theta[5])*sin(real_theta[6]) + r22_*sin(real_theta[4])*sin(real_theta[5])*sin(real_theta[6]);
	//cos_theta2_ = (cos(real_theta[6])*((r33_*cos(real_theta[1])) + (r13_*sin(real_theta[1])))*(cos(real_theta[4])*cos(real_theta[5]) - sin(real_theta[4])*sin(real_theta[5]))) - (((r31_*cos(real_theta[1])) + (r11_*sin(real_theta[1])))*(cos(real_theta[4])*sin(real_theta[5]) + cos(real_theta[5])*sin(real_theta[4]))) + (sin(real_theta[6])*((r32_*cos(real_theta[1])) + (r12_*sin(real_theta[1])))*(cos(real_theta[4])*cos(real_theta[5]) - sin(real_theta[4])*sin(real_theta[5])));
	cos_theta2_ = cos_theta2_temp_1_  + cos_theta2_temp_2_ + cos_theta2_temp_3_;

	if(isnan(floor(100000.*(atan2(sin_theta2_,cos_theta2_)+0.000005))/100000.))
		return;
	else
		real_theta[2] = floor(100000.*(atan2(sin_theta2_,cos_theta2_)+0.000005))/100000.;

	/////////////
	// theta 3 //

	sin_theta3_ = r21_*cos(real_theta[4]+real_theta[5]) + r23_*cos(real_theta[6])*sin(real_theta[4]+real_theta[5]) + r22_*sin(real_theta[6])*sin(real_theta[4]+real_theta[5]);
	cos_theta3_ = r22_*cos(real_theta[6]) - r23_*sin(real_theta[6]);

	if(isnan(floor(100000.*(atan2(sin_theta3_,cos_theta3_)+0.000005))/100000.))
		return;
	else
		real_theta[3] = floor(100000.*(atan2(sin_theta3_,cos_theta3_)+0.000005))/100000.;

	for(int i = 1; i<7;i++)
		real_theta_public[i] = real_theta[i];

	joint_radian << 0 , real_theta[1], real_theta[2] , real_theta[3] , real_theta[4] , real_theta[5] , real_theta[6];
}
//////////////////////////////////////////////////////////Arm Kinematics/////////////////////////////////////////////////////////////////////////
KinematicsArm::KinematicsArm()
{
	joint_radian.resize(4,1);
	joint_radian.fill(0);
	// arm kinematics
	for(int i=0; i<4; i++)
	{

		H_arm[i].resize(4,4);
		H_arm[i].fill(0);
	}
	P_inverse_.fill(0);
	P_.fill(0);

	r11_arm = 0.0;
	r12_arm = 0.0;
	r13_arm = 0.0;

	r21_arm = 0.0;
	r22_arm = 0.0;
	r23_arm = 0.0;

	r31_arm = 0.0;
	r32_arm = 0.0;
	r33_arm = 0.0;

	// DH convention variables
	dh_alpha_arm[0] = 0;
	dh_alpha_arm[1] = M_PI/2;
	dh_alpha_arm[2] = -M_PI/2;
	dh_alpha_arm[3] = 0;

	dh_link_arm[0] = 0;
	dh_link_arm[1] = 0;
	dh_link_arm[2] = 0.22;  // must modify
	dh_link_arm[3] = 0.25; // must modify

	dh_link_d_arm[0] = 0;
	dh_link_d_arm[1] = 0;
	dh_link_d_arm[2] = 0;
	dh_link_d_arm[3] = 0;

	real_theta_arm[0] = 0;
	real_theta_arm[1] = 0;
	real_theta_arm[2] = 0;
	real_theta_arm[3] = 0;

	origin_to_waist_tf_.fill(0);
	origin_to_waist_tf_(3,3) = 1;
	origin_to_arm_tf_.fill(0);
	origin_to_arm_end_point_tf_.fill(0);
	arm_to_origin_tf_.fill(0);
	waist_to_arm_tf_.fill(0);
	waist_to_arm_tf_<< 1,0,0,    0,
			0,1,0,0.205,
			0,0,1,0.205,
			0,0,0,    1;
	origin_desired_point_.resize(4,1);
	origin_desired_point_.fill(0);
	arm_desired_point_.resize(4,1);
	arm_desired_point_.fill(0);
	arm_point_from_origin.resize(4,1);
	arm_point_from_origin.fill(0);
}
KinematicsArm::~KinematicsArm()
{
}
void KinematicsArm::ArmToOriginTransformationCommand(double waist_yaw, double waist_roll, double x, double y, double z)
{
	origin_desired_point_(0,0) = x;
	origin_desired_point_(1,0) = y;
	origin_desired_point_(2,0) = z;
	origin_desired_point_(3,0) = 1;

	origin_to_waist_tf_(0,0) = cos(waist_yaw);
	origin_to_waist_tf_(0,1) = -sin(waist_yaw);

	origin_to_waist_tf_(1,0) = cos(waist_roll)*sin(waist_yaw);
	origin_to_waist_tf_(1,1) = cos(waist_roll)*cos(waist_yaw);
	origin_to_waist_tf_(1,2) = -sin(waist_roll);

	origin_to_waist_tf_(2,0) = sin(waist_roll)*sin(waist_yaw);
	origin_to_waist_tf_(2,1) = sin(waist_roll)*cos(waist_yaw);
	origin_to_waist_tf_(2,2) = cos(waist_roll);
	origin_to_waist_tf_(2,3) = 0.2;


	origin_to_arm_tf_  = origin_to_waist_tf_ * waist_to_arm_tf_;
	arm_to_origin_tf_  = origin_to_arm_tf_.inverse();
	arm_desired_point_ = arm_to_origin_tf_ * origin_desired_point_;
	printf("X : %f //  Y:  %f  // Z: %f  \n",arm_desired_point_(0,0), arm_desired_point_(1,0), arm_desired_point_(2,0));
}
void KinematicsArm::OriginToArmTransformationPoint(double waist_yaw, double waist_roll, double shoulder_pitch, double shoulder_roll, double elbow_pitch)
{
	double temp_joint[4] ={0, shoulder_pitch, shoulder_roll, elbow_pitch};
	origin_to_waist_tf_(0,0) = cos(waist_yaw);
	origin_to_waist_tf_(0,1) = -sin(waist_yaw);

	origin_to_waist_tf_(1,0) = cos(waist_roll)*sin(waist_yaw);
	origin_to_waist_tf_(1,1) = cos(waist_roll)*cos(waist_yaw);
	origin_to_waist_tf_(1,2) = -sin(waist_roll);

	origin_to_waist_tf_(2,0) = sin(waist_roll)*sin(waist_yaw);
	origin_to_waist_tf_(2,1) = sin(waist_roll)*cos(waist_yaw);
	origin_to_waist_tf_(2,2) = cos(waist_roll);
	origin_to_waist_tf_(2,3) = 0.2;
	FowardKinematicsArm(temp_joint,"left");

	origin_to_arm_end_point_tf_  = origin_to_waist_tf_ * waist_to_arm_tf_* P_;
	//printf("%f  %f  %f %f \n",origin_to_arm_end_point_tf_(0,3),origin_to_arm_end_point_tf_(1,3),origin_to_arm_end_point_tf_(2,3), origin_to_arm_end_point_tf_(3,3));
}
void KinematicsArm::FowardKinematicsArm(double joint[4], std::string left_right)
{
	//dh_link_arm[2] = 0.22;  // must modify
	//dh_link_arm[3] = 0.25; // must modify
	double sum_theta[4] = {0,0,0,0};
	double offset_theta[4] = {0, (M_PI)/2,0,0};
	for(int i=1; i<4; i++)
	{
		sum_theta[i] = joint[i]+ offset_theta[i];
	}
	for(int i=1; i<4; i++)
	{
		H_arm[i](0,0) = floor(100000.*(cos(sum_theta[i])+0.000005))/100000.;
		H_arm[i](0,1) = floor(100000.*(-cos(dh_alpha_arm[i])*sin(sum_theta[i])+0.000005))/100000.;
		H_arm[i](0,2) = floor(100000.*(sin(dh_alpha_arm[i])*sin(sum_theta[i])+0.000005))/100000.;
		H_arm[i](0,3) = floor(100000.*(dh_link_arm[i]*cos(sum_theta[i])+0.000005))/100000.;

		H_arm[i](1,0) = floor(100000.*(sin(sum_theta[i])+0.000005))/100000.;
		H_arm[i](1,1) = floor(100000.*(cos(dh_alpha_arm[i])*cos(sum_theta[i])+0.000005))/100000.;
		H_arm[i](1,2) = floor(100000.*(-sin(dh_alpha_arm[i])*cos(sum_theta[i])+0.000005))/100000.;
		H_arm[i](1,3) = floor(100000.*(dh_link_arm[i]*sin(sum_theta[i])+0.000005))/100000.;

		H_arm[i](2,0) =0;
		H_arm[i](2,1) = floor(100000.*(sin(dh_alpha_arm[i])+0.000005))/100000.;
		H_arm[i](2,2) = floor(100000.*(cos(dh_alpha_arm[i])+0.000005))/100000.;
		H_arm[i](2,3) = -dh_link_d_arm[i];

		H_arm[i](3,0) =0;
		H_arm[i](3,1) =0;
		H_arm[i](3,2) =0;
		H_arm[i](3,3) =1;

		H_arm[i](0,0) = cos(sum_theta[i]);
		H_arm[i](0,1) = -cos(dh_alpha_arm[i])*sin(sum_theta[i]);
		H_arm[i](0,2) = sin(dh_alpha_arm[i])*sin(sum_theta[i]);
		H_arm[i](0,3) = dh_link_arm[i]*cos(sum_theta[i]);

		H_arm[i](1,0) = sin(sum_theta[i]);
		H_arm[i](1,1) = cos(dh_alpha_arm[i])*cos(sum_theta[i]);
		H_arm[i](1,2) = -sin(dh_alpha_arm[i])*cos(sum_theta[i]);
		H_arm[i](1,3) = dh_link_arm[i]*sin(sum_theta[i]);

		H_arm[i](2,0) =0;
		H_arm[i](2,1) = sin(dh_alpha_arm[i]);
		H_arm[i](2,2) = cos(dh_alpha_arm[i]);
		H_arm[i](2,3) = -dh_link_d_arm[i];

		H_arm[i](3,0) =0;
		H_arm[i](3,1) =0;
		H_arm[i](3,2) =0;
		H_arm[i](3,3) =1;
	}
	H_arm[0](0,0) = 1;
	H_arm[0](0,1) = 0;
	H_arm[0](0,2) = 0;
	H_arm[0](0,3) = 0;

	H_arm[0](1,0) = 0;
	H_arm[0](1,1) = 0;
	H_arm[0](1,2) = 1;
	H_arm[0](1,3) = 0;

	H_arm[0](2,0) = 0;
	H_arm[0](2,1) = -1;
	H_arm[0](2,2) = 0;
	H_arm[0](2,3) = 0;

	H_arm[0](3,0) = 0;
	H_arm[0](3,1) = 0;
	H_arm[0](3,2) = 0;
	H_arm[0](3,3) = 1;

		P_ = H_arm[0]*H_arm[1]*H_arm[2]*H_arm[3];
		/*
  printf("forward_kinematics arm\n");
	printf("%f  %f  %f %f \n",P_(0,0),P_(0,1),P_(0,2),P_(0,3));
	printf("%f  %f  %f %f \n",P_(1,0),P_(1,1),P_(1,2),P_(1,3));
	printf("%f  %f  %f %f \n",P_(2,0),P_(2,1),P_(2,2),P_(2,3));
	printf("%f  %f  %f %f \n",P_(3,0),P_(3,1),P_(3,2),P_(3,3));*/
}

void KinematicsArm::InverseKinematicsArm(double pX_, double pY_, double pZ_)
{
	//dh_link_arm[2] = 0.22;  // must modify
	//dh_link_arm[3] = 0.25; // must modify

	double l03 = 0;
	double temp_a = 0;
	double temp_b = 0;
	double temp_sin_theta1 = 0;
	double temp_cos_theta1 = 0;

	l03 = sqrt(pow(pX_,2) + pow(pY_,2) + pow(pZ_,2));


	if(isnan(-acos((pow(l03,2)-pow(dh_link_arm[2],2)-pow(dh_link_arm[3],2))/(2*dh_link_arm[2]*dh_link_arm[3]))))
		return;

	else
		joint_radian(3,0) = -acos((pow(l03,2)-pow(dh_link_arm[2],2)-pow(dh_link_arm[3],2))/(2*dh_link_arm[2]*dh_link_arm[3]));


	if(isnan(asin((pY_)/(dh_link_arm[2] + dh_link_arm[3]*cos(joint_radian(3,0))))))
		return;
	else
		joint_radian(2,0) = asin((pY_)/(dh_link_arm[2] + dh_link_arm[3]*cos(joint_radian(3,0))));

	temp_a = dh_link_arm[2]*cos(joint_radian(2,0)) + dh_link_arm[3]*cos(joint_radian(2,0))*cos(joint_radian(3,0));
	temp_b = - dh_link_arm[3]*sin(joint_radian(3,0));

	temp_sin_theta1 = (-temp_a*pX_ - temp_b*pZ_)/(pow(pZ_,2) + pow(pX_,2));
	temp_cos_theta1 = (-temp_a*pZ_ + temp_b*pX_)/(pow(pZ_,2) + pow(pX_,2));

	if(isnan( atan2(temp_sin_theta1,temp_cos_theta1)))
		return;
	else
		joint_radian(1,0) = atan2(temp_sin_theta1,temp_cos_theta1);
}
//////////////////////////////////////////////////////////Euler Aangle Kinematics/////////////////////////////////////////////////////////////////////////
KinematicsEulerAngle::KinematicsEulerAngle()
{
	zyx_euler_angle_matrix_.resize(3,3);
	zyx_euler_angle_matrix_.fill(0);

	xyz_euler_angle_matrix_.resize(3,3);
	xyz_euler_angle_matrix_.fill(0);

	zyx_euler_angle_x = 0;
	zyx_euler_angle_y = 0;
	zyx_euler_angle_z = 0;
	xyz_euler_angle_x = 0;
	xyz_euler_angle_y = 0;
	xyz_euler_angle_z = 0;
}
KinematicsEulerAngle::~KinematicsEulerAngle()
{

}
void KinematicsEulerAngle::ZYXEulerAngles(double z, double y, double x)
{
	zyx_euler_angle_matrix_<< cos(y)*cos(z), cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y),
			cos(y)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(x)*sin(y)*sin(z) - cos(z)*sin(x) ,
			-sin(y)      , cos(y)*sin(x)                       , cos(x)*cos(y);
	//printf("zyx angles \n");
	//printf("%f     %f     %f\n",zyx_euler_angle_matrix_(0,0),zyx_euler_angle_matrix_(0,1),zyx_euler_angle_matrix_(0,2));
	//printf("%f     %f     %f\n",zyx_euler_angle_matrix_(1,0),zyx_euler_angle_matrix_(1,1),zyx_euler_angle_matrix_(1,2));
	//printf("%f     %f     %f\n",zyx_euler_angle_matrix_(2,0),zyx_euler_angle_matrix_(2,1),zyx_euler_angle_matrix_(2,2));
}

void KinematicsEulerAngle::XYZEulerAngles(double x, double y, double z)
{
	xyz_euler_angle_matrix_ << cos(y)*cos(z)                       , -cos(y)*sin(z)                      , sin(y)        ,
			cos(x)*sin(z) + cos(z)*sin(x)*sin(y), cos(x)*cos(z) - sin(x)*sin(y)*sin(z), -cos(y)*sin(x),
			sin(x)*sin(z) - cos(x)*cos(z)*sin(y), cos(z)*sin(x) + cos(x)*sin(y)*sin(z),  cos(x)*cos(y);

	//printf("xyz angles \n");
	//printf("%f     %f     %f\n",xyz_euler_angle_matrix_(0,0),xyz_euler_angle_matrix_(0,1),xyz_euler_angle_matrix_(0,2));
	//printf("%f     %f     %f\n",xyz_euler_angle_matrix_(1,0),xyz_euler_angle_matrix_(1,1),xyz_euler_angle_matrix_(1,2));
	//printf("%f     %f     %f\n",xyz_euler_angle_matrix_(2,0),xyz_euler_angle_matrix_(2,1),xyz_euler_angle_matrix_(2,2));

}
void KinematicsEulerAngle::ZYXEulerAnglesSolution(double z, double y, double x)
{
	ZYXEulerAngles(z, y, x);

	zyx_euler_angle_z = atan2(zyx_euler_angle_matrix_(1,0),zyx_euler_angle_matrix_(0,0));
	zyx_euler_angle_y = atan2(-zyx_euler_angle_matrix_(2,0),sqrt(pow(zyx_euler_angle_matrix_(2,1),2) + pow(zyx_euler_angle_matrix_(2,2),2)));
	zyx_euler_angle_x = atan2(zyx_euler_angle_matrix_(2,1),zyx_euler_angle_matrix_(2,2));

	//printf("zyx angles forward angles \n");
	//ZYXEulerAngles(zyx_euler_angle_z, zyx_euler_angle_y, zyx_euler_angle_x);

}
void KinematicsEulerAngle::XYZEulerAnglesSolution(double x, double y, double z)
{
	XYZEulerAngles(x, y, z);

	xyz_euler_angle_x = atan2(-xyz_euler_angle_matrix_(1,2),xyz_euler_angle_matrix_(2,2));
	xyz_euler_angle_y = atan2(xyz_euler_angle_matrix_(0,2),sqrt(pow(xyz_euler_angle_matrix_(0,0),2) + pow(xyz_euler_angle_matrix_(0,1),2)));
	xyz_euler_angle_z = atan2(-xyz_euler_angle_matrix_(0,1),xyz_euler_angle_matrix_(0,0));

	//printf("xyz angles forward angles \n");
	//XYZEulerAngles(xyz_euler_angle_x, xyz_euler_angle_y, xyz_euler_angle_z);
}
