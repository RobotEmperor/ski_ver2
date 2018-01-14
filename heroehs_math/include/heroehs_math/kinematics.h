
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

	void FowardKnematics(double joint[7] , std::string left_right);
	void FowardKnematicsCenterToSensorRight(double joint[7]);
	void FowardKnematicsCenterToSensorLeft(double joint[7]);
	void InverseKinematics(double pX_, double pY_, double pZ_, double z_alpha_, double y_betta_, double x_kamma_);



	Eigen::MatrixXd tf_origin_to_waist;
	Eigen::MatrixXd tf_waist_to_head;
	Eigen::MatrixXd tf_flag_to_head;

	Eigen::MatrixXd CenterToGroundTransformation(Eigen::MatrixXd point);


	Eigen::MatrixXd joint_radian;
	Eigen::MatrixXd center_to_sensor_transform_right;
	Eigen::MatrixXd center_to_sensor_transform_left;
	Eigen::MatrixXd center_to_foot_transform_left_leg;
	Eigen::MatrixXd center_to_foot_transform_right_leg;

	double real_theta_public[7];

	Eigen::MatrixXd RotationZedToHead(double zed_x, double zed_y, double zed_z);
	void TransformationOriginToWaist(double x, double y, double z, double roll, double pitch, double yaw);
	void TransformationWaistToHead(double x, double y, double z, double roll, double pitch, double yaw);
	void TransformateHeadPointOnOrigin(double x, double y, double z);

	double head_point_on_origin_x, head_point_on_origin_y, head_point_on_origin_z;
	double origin_on_flag_x, origin_on_flag_y, origin_on_flag_z;

private:

	//forward_kinematics leg
	Eigen::MatrixXd H[8];
	Eigen::MatrixXd H_ground_to_center;

	// inverse_kinematics leg
	double r11_,r12_,r13_,r21_,r22_,r23_,r31_,r32_,r33_; // euler angle
	double real_theta[7] , dh_alpha[7] , dh_link[7] , dh_link_d[7]; // dh_convention variables
	double total_length_;
	double sensor_length_;
	Eigen::Matrix4d P_;
	Eigen::Matrix4d P_inverse_;

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

class KinematicsArm
{
public:
	KinematicsArm();
	~KinematicsArm();
	// leg kinematics
	void FowardKinematicsArm(double joint[4] , std::string left_right);
	void InverseKinematicsArm(double pX_, double pY_, double pZ_);
	Eigen::MatrixXd joint_radian;
	Eigen::Matrix4d P_;

	//Arm desired point
	void ArmToOriginTransformationCommand(double waist_yaw,double waist_roll, double x, double y, double z);
	void OriginToArmTransformationPoint(double waist_yaw, double waist_roll, double shoulder_pitch, double shoulder_roll, double elbow_pitch);

	Eigen::MatrixXd arm_desired_point_;
	Eigen::MatrixXd arm_point_from_origin;
	Eigen::Matrix4d origin_to_arm_end_point_tf_;

private:
	//forward_kinematics arm
	Eigen::MatrixXd H_arm[4];

	// inverse_kinematics arm
	double r11_arm,r12_arm,r13_arm,r21_arm,r22_arm,r23_arm,r31_arm,r32_arm,r33_arm; // euler angle
	double real_theta_arm[4] , dh_alpha_arm[4] , dh_link_arm[4] , dh_link_d_arm[4]; // dh_convention variables
	//Eigen::Matrix4d P_;
  Eigen::Matrix4d P_inverse_;

  // transformation matrix
	Eigen::Matrix4d origin_to_waist_tf_;
	Eigen::Matrix4d waist_to_arm_tf_;
	Eigen::Matrix4d origin_to_arm_tf_;
	Eigen::Matrix4d arm_to_origin_tf_;
	Eigen::MatrixXd origin_desired_point_;
	//Eigen::Matrix4d origin_to_arm_end_point_tf_;
};

class KinematicsEulerAngle
{
public:
	KinematicsEulerAngle();
	~KinematicsEulerAngle();

	void ZYXEulerAnglesSolution(double z, double y, double x);
	void ZYXEulerAngles(double z, double y, double x);
	void XYZEulerAnglesSolution(double x, double y, double z);
	void XYZEulerAngles(double z, double y, double x);

	double zyx_euler_angle_x;
	double zyx_euler_angle_y;
	double zyx_euler_angle_z;

	double xyz_euler_angle_x;
	double xyz_euler_angle_y;
	double xyz_euler_angle_z;
private:
	//zyx euler angle
	Eigen::MatrixXd zyx_euler_angle_matrix_;

	//xyz euler angle
	Eigen::MatrixXd xyz_euler_angle_matrix_;
};
}
#endif /* HEROEHS_MATH_KINEMATICS_H_ */
